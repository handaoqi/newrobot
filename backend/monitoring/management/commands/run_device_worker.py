from __future__ import annotations

import logging
import threading

from django.conf import settings
from django.core.management.base import BaseCommand
from django.utils import timezone

from monitoring.models import RemoteCommand, Robot, RobotSession
from monitoring.mqtt_client import PlatformMqttClient
from monitoring.services.command_service import CommandService

LOGGER = logging.getLogger(__name__)


class Command(BaseCommand):
    help = "Run the standalone MQTT device worker."

    def handle(self, *args, **options):
        client = PlatformMqttClient()
        stop_event = threading.Event()

        def housekeeping():
            while not stop_event.wait(2):
                try:
                    client.publish_pending_commands()
                    self._expire_commands()
                    self._mark_offline_robots()
                except Exception:
                    LOGGER.exception("device worker housekeeping failed")

        thread = threading.Thread(target=housekeeping, daemon=True, name="device-housekeeping")
        thread.start()
        try:
            client.connect()
            client.loop_forever()
        except KeyboardInterrupt:
            self.stdout.write("device worker stopping")
        finally:
            stop_event.set()
            client.disconnect()
            thread.join(timeout=3)

    @staticmethod
    def _expire_commands():
        now = timezone.now()
        for command in RemoteCommand.objects.filter(
            status__in=["created", "published", "accepted", "executing"],
            expires_at__lt=now,
        ):
            CommandService.mark_timeout(command)

    @staticmethod
    def _mark_offline_robots():
        cutoff = timezone.now() - timezone.timedelta(
            seconds=getattr(settings, "DEVICE_OFFLINE_TIMEOUT_SECONDS", 30)
        )
        stale_sessions = RobotSession.objects.filter(
            disconnected_at__isnull=True,
            last_heartbeat_at__lt=cutoff,
        )
        robot_ids = list(stale_sessions.values_list("robot_id", flat=True))
        stale_sessions.update(disconnected_at=timezone.now(), disconnect_reason="heartbeat_timeout")
        Robot.objects.filter(pk__in=robot_ids).update(
            connection_status="offline",
            status="offline",
        )
