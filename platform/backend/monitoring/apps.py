from django.apps import AppConfig
from django.db.backends.signals import connection_created


class MonitoringConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "monitoring"

    def ready(self) -> None:
        from .services.sqlite_retry import configure_sqlite_connection

        connection_created.connect(
            configure_sqlite_connection,
            dispatch_uid="monitoring.configure_sqlite_connection",
        )
