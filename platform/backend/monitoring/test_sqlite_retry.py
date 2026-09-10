from django.db import connection
from django.db.utils import OperationalError
from django.test import SimpleTestCase, TestCase, override_settings

from .services.sqlite_retry import with_sqlite_lock_retry


class SqliteRetryTests(SimpleTestCase):
    @override_settings(SQLITE_LOCK_RETRY_ATTEMPTS=4, SQLITE_LOCK_RETRY_BACKOFF_SECONDS=0)
    def test_retries_transient_lock_until_operation_succeeds(self):
        calls = 0

        def operation():
            nonlocal calls
            calls += 1
            if calls < 3:
                raise OperationalError("database is locked")
            return "ok"

        self.assertEqual(with_sqlite_lock_retry(operation, label="test"), "ok")
        self.assertEqual(calls, 3)

    @override_settings(SQLITE_LOCK_RETRY_ATTEMPTS=2, SQLITE_LOCK_RETRY_BACKOFF_SECONDS=0)
    def test_does_not_retry_non_lock_operational_error(self):
        calls = 0

        def operation():
            nonlocal calls
            calls += 1
            raise OperationalError("disk I/O error")

        with self.assertRaises(OperationalError):
            with_sqlite_lock_retry(operation, label="test")
        self.assertEqual(calls, 1)


class SqlitePragmaTests(TestCase):
    def test_configure_sqlite_connection_sets_busy_timeout(self):
        if connection.vendor != "sqlite":
            self.skipTest("sqlite only")
        from .services.sqlite_retry import configure_sqlite_connection

        configure_sqlite_connection(None, connection)
        with connection.cursor() as cursor:
            cursor.execute("PRAGMA busy_timeout")
            timeout = int(cursor.fetchone()[0])
        self.assertGreaterEqual(timeout, 1000)
