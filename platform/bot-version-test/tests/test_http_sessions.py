from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


SRC_DIR = Path(__file__).resolve().parents[1] / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from bike_bot.telemetry import TelemetryClient  # noqa: E402


class TelemetrySessionTests(unittest.TestCase):
    @staticmethod
    def make_config():
        return SimpleNamespace(
            robot=SimpleNamespace(code="robot-1"),
            storage=SimpleNamespace(telemetry_log_path="telemetry.jsonl"),
        )

    @patch("bike_bot.telemetry.requests.Session")
    def test_reuses_one_session_per_stable_worker_thread(self, session_factory):
        created_sessions = []

        def make_session():
            session = Mock()
            created_sessions.append(session)
            return session

        session_factory.side_effect = make_session
        client = TelemetryClient(self.make_config(), Mock())

        main_session = client._session()
        self.assertIs(main_session, client._session())
        worker_sessions = []

        def use_session_twice():
            first = client._session()
            worker_sessions.extend([first, client._session()])

        worker = threading.Thread(target=use_session_twice)
        worker.start()
        worker.join(timeout=1)

        self.assertEqual(worker_sessions, [worker_sessions[0], worker_sessions[0]])
        self.assertIsNot(main_session, worker_sessions[0])
        # Dedicated person reporter, main worker, secondary worker.
        self.assertEqual(len(created_sessions), 3)

        client.close()
        for session in created_sessions:
            session.close.assert_called_once_with()

    @patch("bike_bot.telemetry.requests.Session")
    def test_person_report_reuses_dedicated_session(self, session_factory):
        person_session = Mock()
        person_session.post.return_value = Mock()
        session_factory.return_value = person_session
        client = TelemetryClient(self.make_config(), Mock())
        client.config.telemetry = SimpleNamespace(
            timeout_seconds=5,
            verify_tls=True,
        )
        client._person_report_lock.acquire()

        client._post_person_detections(
            "https://platform.example/api/device/person-detections/",
            {"detections": []},
            {"X-Device-Code": "robot-1"},
        )

        person_session.post.assert_called_once()
        self.assertFalse(client._person_report_lock.locked())
        client.close()


if __name__ == "__main__":
    unittest.main()
