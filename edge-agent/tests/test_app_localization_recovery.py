import threading
from types import SimpleNamespace

import roamerx_edge.app as app_module
from roamerx_edge.app import EdgeAgentApplication


class FakeNavigation:
    def __init__(self):
        self.attempts = 0
        self.seeds = []
        self.motion_holds = 0

    def latest_trusted_pose(self):
        return {"x": 1.0, "y": 2.0, "yaw": 0.3}

    def active_relocalize(self, seed):
        self.attempts += 1
        self.seeds.append(dict(seed))
        if self.attempts <= 1:
            raise RuntimeError("not converged")
        return {"mode": "stationary_bounded_search", "accepted": True}

    def cancel_navigation(self):
        self.motion_holds += 1
        return True

    def stop_motion(self):
        return None


class FakeTaskExecutor:
    context = None

    def is_paused_for_localization(self):
        return True

    def has_active_task(self):
        return True

    def on_localization_lost(self):
        return None


class IdleTaskExecutor:
    def __init__(self):
        self.loss_notifications = 0
        self.context = None

    def on_localization_lost(self):
        self.loss_notifications += 1

    def is_paused_for_localization(self):
        return False

    def has_active_task(self):
        return False


class StuckTaskExecutor:
    """Never leaves the paused state, so the recovery loop only ends on its own terms."""

    context = None

    def is_paused_for_localization(self):
        return True

    def has_active_task(self):
        return True


def _wire_recovery_collaborators(application, *, max_cycles=0):
    """Attach the collaborators `_recover_task_localization` reaches for."""
    application.config = SimpleNamespace(
        safety=SimpleNamespace(
            localization_recovery_attempts=3,
            localization_recovery_retry_seconds=0.5,
            localization_recovery_cycle_seconds=30.0,
            localization_recovery_max_cycles=max_cycles,
        ),
        robot=SimpleNamespace(current_map_id="map-1", current_map_version="v1"),
    )
    application.store = SimpleNamespace(load_last_trusted_pose=lambda *_args: None)
    application.recovery_states = []
    application.telemetry = SimpleNamespace(
        on_localization_recovery=application.recovery_states.append
    )
    application.emitted_alerts = []
    application.alerts = SimpleNamespace(
        emit_system_alert=lambda *args, **kwargs: application.emitted_alerts.append((args, kwargs))
    )
    application._localization_recovery_lock = threading.Lock()
    application._localization_recovery_lock.acquire()


def test_recovery_retries_a_new_cycle_after_three_quick_failures(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.navigation = FakeNavigation()
    application.task_executor = FakeTaskExecutor()
    application.navigation_stack_adapter = SimpleNamespace(restarts=0)
    _wire_recovery_collaborators(application)
    sleeps = []
    monkeypatch.setattr(app_module.time, "sleep", sleeps.append)

    application._recover_task_localization()

    assert application.navigation.attempts == 2
    assert application.navigation.seeds[0]["source"] == "last_trusted"
    assert application.navigation.motion_holds >= 1
    assert 30.0 in sleeps
    assert application._localization_recovery_lock.acquire(blocking=False)
    # One cycle failed outright, so exactly one progress report, then a clear on exit.
    assert application.recovery_states[0]["cycle"] == 1
    assert application.recovery_states[0]["max_cycles"] is None
    assert application.recovery_states[-1] is None


def test_recovery_escalates_once_the_cycle_budget_is_spent(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.navigation = SimpleNamespace(
        latest_trusted_pose=lambda: None,
        localization_diagnostics=lambda: {},
    )
    application.task_executor = StuckTaskExecutor()
    application.navigation_stack_adapter = SimpleNamespace(restart_localization=lambda: None)
    _wire_recovery_collaborators(application, max_cycles=2)
    monkeypatch.setattr(app_module.time, "sleep", lambda _seconds: None)

    application._recover_task_localization("ndt_degraded")

    # Without a cycle bound this loop would never return; the bound is the whole point.
    assert [state["cycle"] for state in application.recovery_states if state] == [1, 2]
    assert len(application.emitted_alerts) == 1
    args, kwargs = application.emitted_alerts[0]
    assert args[0] == "localization_recovery_failed"
    assert args[1] == "critical"
    assert kwargs["attributes"]["recovery_cycles"] == 2
    assert kwargs["attributes"]["reason"] == "ndt_degraded"
    assert application._localization_recovery_lock.acquire(blocking=False)


class RecordingThread:
    created = []

    def __init__(self, target=None, daemon=None, name=None, args=(), kwargs=None):
        self.target = target
        self.args = args
        self.name = name
        RecordingThread.created.append(self)

    def start(self):
        pass


def test_localization_loss_starts_auto_relocalize_without_active_task(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.task_executor = IdleTaskExecutor()
    application.navigation = SimpleNamespace(localization_diagnostics=lambda: {"quality": "bad"})
    application.config = SimpleNamespace(
        robot=SimpleNamespace(current_map_id="map-1", current_map_version="v1")
    )
    application._localization_alert_notified = False
    alerts = []
    application.alerts = SimpleNamespace(
        emit_system_alert=lambda *args, **kwargs: alerts.append((args, kwargs))
    )
    application._localization_recovery_lock = threading.Lock()
    RecordingThread.created = []
    monkeypatch.setattr(app_module.threading, "Thread", RecordingThread)

    application._handle_task_localization_loss()
    application._handle_task_localization_loss()

    assert application.task_executor.loss_notifications == 2
    assert [thread.name for thread in RecordingThread.created] == ["task-localization-restart"]
    assert not application._localization_recovery_lock.acquire(blocking=False)
    assert len(alerts) == 1
    assert alerts[0][0][0] == "localization_lost"
    assert alerts[0][0][1] == "high"
    assert alerts[0][1]["attributes"]["localization_quality"] == "bad"
    assert alerts[0][1]["component"] == "localization"


def test_mapping_session_skips_auto_relocalize(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.task_executor = FakeTaskExecutor()
    application.navigation = SimpleNamespace(localization_diagnostics=lambda: {})
    application.mapping_adapter = SimpleNamespace(status=lambda: {"process_alive": True})
    application.config = SimpleNamespace(
        robot=SimpleNamespace(current_map_id="map-1", current_map_version="v1")
    )
    application._localization_alert_notified = False
    application.alerts = SimpleNamespace(emit_system_alert=lambda *args, **kwargs: None)
    application._localization_recovery_lock = threading.Lock()

    def unexpected_thread(**_kwargs):
        raise AssertionError("recovery thread should not be created while mapping")

    monkeypatch.setattr(app_module.threading, "Thread", unexpected_thread)

    application._handle_task_localization_loss()

    assert application._localization_recovery_lock.acquire(blocking=False)


def test_ndt_degradation_alerts_at_a_lower_severity(monkeypatch):
    application = object.__new__(EdgeAgentApplication)
    application.task_executor = IdleTaskExecutor()
    application.navigation = SimpleNamespace(localization_diagnostics=lambda: {})
    application.config = SimpleNamespace(
        robot=SimpleNamespace(current_map_id="map-1", current_map_version="v1")
    )
    application._localization_alert_notified = False
    alerts = []
    application.alerts = SimpleNamespace(
        emit_system_alert=lambda *args, **kwargs: alerts.append(args)
    )
    application._localization_recovery_lock = threading.Lock()
    RecordingThread.created = []
    monkeypatch.setattr(app_module.threading, "Thread", RecordingThread)

    application._handle_task_localization_loss("ndt_degraded")

    assert alerts[0][0] == "ndt_degraded"
    assert alerts[0][1] == "medium"
    assert [thread.name for thread in RecordingThread.created] == ["task-localization-restart"]


def test_recovered_callback_rearms_alerting_and_clears_recovery_state():
    application = object.__new__(EdgeAgentApplication)
    application._localization_alert_notified = True
    cleared = []
    application.telemetry = SimpleNamespace(on_localization_recovery=cleared.append)
    resumed = []
    application.task_executor = SimpleNamespace(
        on_localization_recovered=lambda: resumed.append(True)
    )

    application._handle_task_localization_recovered()

    assert application._localization_alert_notified is False
    assert cleared == [None]
    assert resumed == [True]


def test_low_battery_terminal_context_uses_idempotent_cancel_without_force_exit():
    application = object.__new__(EdgeAgentApplication)
    context = SimpleNamespace(task_execution_id="task-1", state="failed", docking={})
    calls = []
    application.task_executor = SimpleNamespace(
        context=context,
        has_active_task=lambda: False,
        cancel_task=lambda execution_id: calls.append(("cancel", execution_id)) or {
            "final_task_state": "failed",
            "already_terminal": True,
        },
        force_exit=lambda _execution_id: calls.append(("force", _execution_id)),
    )
    application.navigation = SimpleNamespace(latest_pose=lambda: None)
    application.safety_state = SimpleNamespace(
        current_map_id="map-1", current_map_version="v1"
    )
    application.config = SimpleNamespace(
        robot=SimpleNamespace(agent_version="test"),
        charge_control=SimpleNamespace(
            low_battery_start_percent=20,
            low_battery_rearm_percent=25,
        ),
    )
    alerts = []
    application.mqtt = SimpleNamespace(publish_alert=alerts.append)

    application._handle_low_battery_charge("episode-1", 19)

    assert calls == [("cancel", "task-1")]
    assert alerts[0]["attributes"]["action"] == "return_charge_requested"


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


def test_mapping_divergence_event_requests_passive_then_rescues():
    application = object.__new__(EdgeAgentApplication)
    calls = []
    application.navigation = SimpleNamespace(
        confirmed_remote_teleop_action=lambda *args, **kwargs: calls.append(("passive", args, kwargs))
    )
    application.mapping_adapter = SimpleNamespace(
        auto_rescue_diverged_mapping=lambda event: calls.append(("rescue", event))
    )
    application._mapping_rescue_lock = threading.Lock()
    event = {"writer_flushed": True, "last_healthy_keyframe": 12}

    application._handle_mapping_divergence_event(event)

    assert calls[0][0] == "passive"
    assert calls[0][1][0] == "passive"
    assert calls[1] == ("rescue", event)


def test_mapping_divergence_event_does_not_rescue_before_flush():
    application = object.__new__(EdgeAgentApplication)
    calls = []
    application.navigation = SimpleNamespace(
        confirmed_remote_teleop_action=lambda *args, **kwargs: calls.append("passive")
    )
    application.mapping_adapter = SimpleNamespace(
        auto_rescue_diverged_mapping=lambda event: calls.append("rescue")
    )
    application._mapping_rescue_lock = threading.Lock()

    application._handle_mapping_divergence_event({"writer_flushed": False})

    assert calls == ["passive"]
