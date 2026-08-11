import json
import uuid

from django.contrib.auth import get_user_model
from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .dev_message_handlers import handle_dev_mqtt_message
from .models import DevelopmentAgentState, DevelopmentTask, DevelopmentTaskEvent, Robot


class RemoteDevelopmentApiTests(TestCase):
    def setUp(self):
        self.user = get_user_model().objects.create_user(username="developer", password="test")
        self.robot = Robot.objects.create(code="rx-dev-api", name="机器狗", location="现场", area="现场")
        self.client = APIClient()
        self.client.force_authenticate(self.user)

    def test_create_and_cancel_task(self):
        response = self.client.post(
            "/api/development/tasks/",
            {"robot": self.robot.id, "workspace": "robot-main", "prompt": "检查导航代码"},
            format="json",
        )
        self.assertEqual(response.status_code, 201)
        task = DevelopmentTask.objects.get(pk=response.data["id"])
        self.assertEqual(task.operator, self.user)

        cancel = self.client.post(f"/api/development/tasks/{task.id}/cancel/", {}, format="json")
        self.assertEqual(cancel.status_code, 200)
        task.refresh_from_db()
        self.assertEqual(task.status, "cancelled")

    def test_only_one_active_task_per_robot(self):
        DevelopmentTask.objects.create(
            robot=self.robot,
            workspace="robot-main",
            prompt="first",
            operator=self.user,
        )
        response = self.client.post(
            "/api/development/tasks/",
            {"robot": self.robot.id, "workspace": "robot-main", "prompt": "second"},
            format="json",
        )
        self.assertEqual(response.status_code, 409)


class RemoteDevelopmentMqttTests(TestCase):
    def setUp(self):
        self.robot = Robot.objects.create(code="rx-dev-mqtt", name="机器狗", location="现场", area="现场")
        self.task = DevelopmentTask.objects.create(
            robot=self.robot,
            workspace="robot-main",
            prompt="检查代码",
            status="published",
        )

    def topic(self, suffix):
        return f"robots/{self.robot.code}/dev/{suffix}"

    def test_presence_and_task_lifecycle(self):
        handle_dev_mqtt_message(
            self.topic("presence"),
            {"status": "online", "agent_version": "0.1.0", "workspaces": ["robot-main"]},
        )
        self.assertEqual(DevelopmentAgentState.objects.get(robot=self.robot).status, "online")

        event = {
            "task_id": str(self.task.id),
            "sequence": 1,
            "type": "status",
            "status": "running",
            "text": "Codex 已启动",
            "timestamp": timezone.now().isoformat(),
        }
        event_topic = self.topic(f"tasks/{self.task.id}/events")
        handle_dev_mqtt_message(event_topic, event)
        handle_dev_mqtt_message(event_topic, event)
        self.task.refresh_from_db()
        self.assertEqual(self.task.status, "running")
        self.assertEqual(DevelopmentTaskEvent.objects.filter(task=self.task).count(), 1)

        handle_dev_mqtt_message(
            self.topic(f"tasks/{self.task.id}/result"),
            {
                "task_id": str(self.task.id),
                "status": "succeeded",
                "exit_code": 0,
                "last_message": "完成",
                "codex_thread_id": "thread-123",
                "timestamp": timezone.now().isoformat(),
            },
        )
        self.task.refresh_from_db()
        self.assertEqual(self.task.status, "succeeded")
        self.assertEqual(self.task.exit_code, 0)
        self.assertEqual(self.task.codex_thread_id, "thread-123")
        self.assertEqual(self.task.events.count(), 2)
