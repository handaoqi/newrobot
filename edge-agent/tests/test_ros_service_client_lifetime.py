import threading
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from roamerx_edge.protocol import ProtocolError
from roamerx_edge import ros_adapter as ros_adapter_module
from roamerx_edge.ros_adapter import RosAdapter, RosRuntime


class _PendingFuture:
    def __init__(self):
        self._callbacks = []

    def done(self):
        return False

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        raise AssertionError("pending future must not be consumed")


class _DoneFuture:
    def __init__(self, result=None):
        self._result = result

    def done(self):
        return True

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._result

    def exception(self):
        return None


class _FakeClient:
    def __init__(self, future):
        self.future = future
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class _FakeGetParameters:
    class Request:
        def __init__(self):
            self.names = []


class _FakeSetParameters:
    class Request:
        def __init__(self):
            self.parameters = []


def _adapter_with_client(future, *, destroyed):
    adapter = object.__new__(RosAdapter)
    client = _FakeClient(future)
    adapter._nav_service_callback_group = None
    adapter.create_client = lambda *args, **kwargs: client
    adapter.destroy_client = lambda value: destroyed.append(value)
    adapter._remote_param_is_cooling_down = lambda _name: False
    adapter._mark_remote_param_unavailable = lambda _name: None
    adapter._parameter_value_to_python = lambda value: value
    adapter._parameter_message = lambda name, value: (name, value)
    return adapter, client


def test_destroy_client_if_idle_skips_in_flight_handles():
    destroyed = []
    adapter = object.__new__(RosAdapter)
    adapter.destroy_client = lambda client: destroyed.append(client)
    pending = _PendingFuture()
    adapter._destroy_client_if_idle("client", pending)
    assert destroyed == []
    adapter._destroy_client_if_idle("client", _DoneFuture())
    assert destroyed == ["client"]


def test_get_remote_parameters_does_not_destroy_client_on_timeout(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "ROS_AVAILABLE", True)
    monkeypatch.setattr(ros_adapter_module, "GetParameters", _FakeGetParameters)
    destroyed = []
    adapter, client = _adapter_with_client(_PendingFuture(), destroyed=destroyed)
    adapter._wait_for_future = lambda future, timeout_seconds: False

    with pytest.raises(ProtocolError) as exc:
        adapter._get_remote_parameters("/planner_server", ["planner_plugins"], attempts=2)

    assert "get_parameters timed out" in str(exc.value)
    assert client.requests
    assert destroyed == []


def test_get_remote_parameters_reuses_client_after_completed_call(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "ROS_AVAILABLE", True)
    monkeypatch.setattr(ros_adapter_module, "GetParameters", _FakeGetParameters)
    destroyed = []
    adapter, client = _adapter_with_client(
        _DoneFuture(SimpleNamespace(values=["ThetaStar"])),
        destroyed=destroyed,
    )

    result = adapter._get_remote_parameters("/planner_server", ["planner_plugins"])
    repeated = adapter._get_remote_parameters("/planner_server", ["planner_plugins"])

    assert result == {"planner_plugins": "ThetaStar"}
    assert repeated == result
    assert len(client.requests) == 2
    assert destroyed == []


def test_set_remote_parameters_does_not_destroy_client_on_timeout(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "SetParameters", _FakeSetParameters)
    destroyed = []
    adapter, client = _adapter_with_client(_PendingFuture(), destroyed=destroyed)
    adapter._wait_for_future = lambda future, timeout_seconds: False
    adapter._remote_param_values_match = lambda node, values: False

    with pytest.raises(ProtocolError) as exc:
        adapter._set_remote_parameters(
            "/planner_server",
            {"planner_plugins": ["ThetaStar"]},
            code="NAV_PROFILE_APPLY_FAILED",
            attempts=1,
        )

    assert "parameter request timed out" in str(exc.value)
    assert client.requests
    assert destroyed == []


def test_ros_runtime_keeps_spinning_after_callback_error(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "rclpy", SimpleNamespace(ok=lambda: True))
    runtime = object.__new__(RosRuntime)
    runtime._stopped = threading.Event()
    calls = {"n": 0}

    class FakeExecutor:
        def spin_once(self, timeout_sec=None):
            calls["n"] += 1
            if calls["n"] == 1:
                raise RuntimeError("cannot use Destroyable because destruction was requested")
            runtime._stopped.set()

    runtime.executor = FakeExecutor()
    runtime._spin()
    assert calls["n"] == 2


def test_ros_runtime_reports_a_permanently_failed_executor(monkeypatch):
    monkeypatch.setattr(ros_adapter_module, "rclpy", SimpleNamespace(ok=lambda: True))
    runtime = object.__new__(RosRuntime)
    runtime._stopped = threading.Event()
    runtime._MAX_CONSECUTIVE_SPIN_FAILURES = 2
    failures = []
    runtime._unexpected_exit_callback = failures.append

    class FailedExecutor:
        def spin_once(self, timeout_sec=None):
            raise RuntimeError("executor wait set is invalid")

    runtime.executor = FailedExecutor()
    runtime._spin()

    assert failures == ["consecutive_spin_failures:RuntimeError"]
    assert runtime._stopped.is_set() is False
