import threading
from types import SimpleNamespace

import roamerx_edge.app as app_module
from roamerx_edge.app import EdgeAgentApplication


class FakeNavigation:
    def __init__(self):
        self.attempts = 0

    def latest_trusted_pose(self):
        return {"x": 1.0, "y": 2.0, "yaw": 0.3}

    def set_initial_pose(self, _pose):
        self.attempts += 1
        if self.attempts <= 3:
            raise RuntimeError("not converged")


class FakeTaskExecutor:
    def is_paused_for_localization(self):
        return True

    def has_active_task(self):
        return True


class IdleTaskExecutor:
    def __init__(self):
        self.loss_notifications = 0

    def on_localization_lost(self):
        self.loss_notifications += 1

    def is_paused_for_localization(self):
        return False

    def has_active_task(self):
        return False


def test_recovery_retries_a_new_cycle_after_three_quick_failures(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.navigation = FakeNavigation()
    application.task_executor = FakeTaskExecutor()
    application.navigation_stack_adapter = SimpleNamespace(restarts=0)

    def restart():
        application.navigation_stack_adapter.restarts += 1

    application.navigation_stack_adapter.restart_localization = restart
    application.config = SimpleNamespace(
        safety=SimpleNamespace(
            localization_recovery_attempts=3,
            localization_recovery_retry_seconds=0.5,
            localization_recovery_cycle_seconds=30.0,
        ),
        robot=SimpleNamespace(current_map_id="map-1", current_map_version="v1"),
    )
    application.store = SimpleNamespace(load_last_trusted_pose=lambda *_args: None)
    application._localization_recovery_lock = threading.Lock()
    application._localization_recovery_lock.acquire()
    sleeps = []
    monkeypatch.setattr(app_module.time, "sleep", sleeps.append)

    application._recover_task_localization()

    assert application.navigation.attempts == 4
    assert application.navigation_stack_adapter.restarts == 2
    assert 30.0 in sleeps
    assert application._localization_recovery_lock.acquire(blocking=False)


def test_localization_loss_does_not_start_task_recovery_without_active_task(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.task_executor = IdleTaskExecutor()
    application._localization_recovery_lock = threading.Lock()

    def unexpected_thread(**_kwargs):
        raise AssertionError("recovery thread should not be created")

    monkeypatch.setattr(app_module.threading, "Thread", unexpected_thread)

    application._handle_task_localization_loss()

    assert application.task_executor.loss_notifications == 1
    assert application._localization_recovery_lock.acquire(blocking=False)


def test_mapping_divergence_alert_emits_once(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application._mapping_divergence_notified = False
    alerts = []
    application.alerts = SimpleNamespace(
        emit_system_alert=lambda *args, **kwargs: alerts.append((args, kwargs)) or "event-1"
    )
    started = []

    class FakeThread:
        def __init__(self, target=None, daemon=None, name=None):
            started.append(name)
            self.target = target

        def start(self):
            pass

    monkeypatch.setattr(app_module.threading, "Thread", FakeThread)

    payload = {
        "state": "mapping",
        "map_name": "园区",
        "mapping_session_id": "sess-1",
        "save_progress": {
            "error_code": "SLAM_DIVERGED",
            "error": "pose anomaly detected",
            "slam_health": {"state": "diverged", "warning": "speed=12"},
        },
    }
    application._observe_mapping_health(payload)
    application._observe_mapping_health(payload)

    assert len(alerts) == 1
    assert alerts[0][0][0] == "slam_diverged"
    assert started == ["mapping-diverged-speech"]

    application._observe_mapping_health({"state": "idle", "save_progress": {}})
    assert application._mapping_divergence_notified is False
