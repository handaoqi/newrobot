import json
import uuid
from unittest.mock import patch

from django.contrib.auth import get_user_model
from django.test import TestCase
from django.utils import timezone
from rest_framework.test import APIClient

from .dev_message_handlers import _VOICE_WAKE_UNTIL, handle_dev_mqtt_message
from .models import DevelopmentAgentState, DevelopmentTask, DevelopmentTaskEvent, Robot, VoiceRecognitionEvent


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

    def test_main_conversation_returns_one_thread_in_chronological_order(self):
        old_thread_task = DevelopmentTask.objects.create(
            robot=self.robot,
            workspace="robot-main",
            prompt="旧会话",
            status="succeeded",
            codex_thread_id="thread-old",
            operator=self.user,
        )
        first = DevelopmentTask.objects.create(
            robot=self.robot,
            workspace="robot-main",
            prompt="第一条连续指令",
            status="succeeded",
            codex_thread_id="thread-main",
            operator=self.user,
        )
        second = DevelopmentTask.objects.create(
            robot=self.robot,
            workspace="robot-main",
            prompt="第二条连续指令",
            status="succeeded",
            codex_thread_id="thread-main",
            operator=self.user,
        )
        DevelopmentTaskEvent.objects.create(
            task=first,
            sequence=1,
            event_type="output",
            text="第一条输出",
        )
        DevelopmentTaskEvent.objects.create(
            task=second,
            sequence=1,
            event_type="output",
            text="第二条输出",
        )

        response = self.client.get(
            f"/api/development/conversations/main/?robot={self.robot.id}"
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data["codex_thread_id"], "thread-main")
        prompts = [turn["task"]["prompt"] for turn in response.data["turns"]]
        self.assertEqual(prompts, ["第一条连续指令", "第二条连续指令"])
        self.assertNotIn(str(old_thread_task.id), [
            turn["task"]["id"] for turn in response.data["turns"]
        ])
        self.assertEqual(response.data["turns"][0]["events"][0]["text"], "第一条输出")

    def test_voice_recognition_history_returns_text_without_audio(self):
        event = VoiceRecognitionEvent.objects.create(
            robot=self.robot,
            transcript="小菜阳检查导航服务",
            command="检查导航服务",
            asr_engine="nx-sensevoice",
            outcome="ignored",
        )

        response = self.client.get(f"/api/development/voice-recognitions/?robot={self.robot.id}")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.data[0]["id"], event.id)
        self.assertEqual(response.data[0]["transcript"], "小菜阳检查导航服务")
        self.assertNotIn("audio", response.data[0])


class RemoteDevelopmentMqttTests(TestCase):
    def setUp(self):
        _VOICE_WAKE_UNTIL.clear()
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

    def test_terminal_result_broadcasts_compact_codex_summary_once(self):
        self.task.status = "running"
        self.task.save(update_fields=["status", "updated_at"])
        agent_message = {
            "type": "item.completed",
            "item": {"type": "agent_message", "text": "已完成语音链路修复，服务运行正常。"},
        }
        handle_dev_mqtt_message(
            self.topic(f"tasks/{self.task.id}/events"),
            {
                "task_id": str(self.task.id),
                "sequence": 1,
                "type": "output",
                "stream": "stdout",
                "text": json.dumps(agent_message, ensure_ascii=False),
                "timestamp": timezone.now().isoformat(),
            },
        )
        published = []
        with patch(
            "monitoring.dev_message_handlers.transaction.on_commit",
            side_effect=lambda callback: callback(),
        ), patch(
            "monitoring.dev_message_handlers.tts_service.synthesize_speech",
            return_value=("tts-audio/summary.mp3", False),
        ) as synthesize:
            result = handle_dev_mqtt_message(
                self.topic(f"tasks/{self.task.id}/result"),
                {
                    "task_id": str(self.task.id),
                    "status": "succeeded",
                    "exit_code": 0,
                    "timestamp": timezone.now().isoformat(),
                },
                publish=lambda *args: published.append(args),
            )

        self.assertEqual(result["status"], "succeeded")
        synthesize.assert_called_once_with("Codex任务已完成。以下是本次执行结果。已完成语音链路修复，服务运行正常。")
        self.assertEqual(published[0][0], self.topic("voice/ack"))
        self.assertEqual(published[0][1]["kind"], "task_summary")
        self.assertGreaterEqual(published[0][1]["suppress_seconds"], 6)

        duplicate = handle_dev_mqtt_message(
            self.topic(f"tasks/{self.task.id}/result"),
            {"task_id": str(self.task.id), "status": "succeeded"},
        )
        self.assertTrue(duplicate["duplicate"])

    def test_nx_local_asr_transcript_creates_task_without_cloud_transcription(self):
        self.task.status = "succeeded"
        self.task.save(update_fields=["status", "updated_at"])

        with patch("monitoring.dev_message_handlers.asr_service.transcribe_audio") as cloud_asr:
            result = handle_dev_mqtt_message(
                self.topic("voice/audio"),
                {
                    "asr_engine": "nx-sensevoice",
                    "transcript": "小太阳，检查导航服务",
                },
            )

        self.assertEqual(result["status"], "accepted")
        self.assertFalse(cloud_asr.called)
        created = DevelopmentTask.objects.exclude(pk=self.task.pk).get()
        self.assertEqual(created.prompt, "检查导航服务")
        recognition = VoiceRecognitionEvent.objects.get(robot=self.robot)
        self.assertEqual(recognition.outcome, "accepted")
        self.assertEqual(recognition.transcript, "小太阳，检查导航服务")
        self.assertEqual(recognition.task, created)

    def test_empty_nx_transcript_is_recorded_as_no_speech(self):
        result = handle_dev_mqtt_message(
            self.topic("voice/audio"),
            {"asr_engine": "nx-sensevoice", "transcript": ""},
        )

        self.assertEqual(result["status"], "no_speech")
        recognition = VoiceRecognitionEvent.objects.get(robot=self.robot)
        self.assertEqual(recognition.outcome, "no_speech")
        self.assertEqual(recognition.transcript, "")

    def test_observed_nx_wake_word_substitutions_create_task(self):
        self.task.status = "succeeded"
        self.task.save(update_fields=["status", "updated_at"])

        # These are not aliases: the matcher derives their pinyin at runtime.
        # The first two cases are homophones/near-initials not previously seen
        # in the recognition history; the remaining values are observed ASR
        # substitutions.
        for wake_word in ("晓太洋", "小代阳", "小菜阳", "要太阳", "老太阳"):
            with self.subTest(wake_word=wake_word):
                result = handle_dev_mqtt_message(
                    self.topic("voice/audio"),
                    {
                        "asr_engine": "nx-sensevoice",
                        "transcript": f"{wake_word}，创建测试任务",
                    },
                )
                self.assertEqual(result["status"], "accepted")
                created = DevelopmentTask.objects.get(pk=result["task_id"])
                self.assertEqual(created.prompt, "创建测试任务")
                created.status = "succeeded"
                created.save(update_fields=["status", "updated_at"])

    def test_fuzzy_wake_word_rejects_unrelated_pinyin_finals(self):
        self.task.status = "succeeded"
        self.task.save(update_fields=["status", "updated_at"])

        for transcript in ("有太阳创建测试任务", "呃太阳创建测试任务", "狗太阳创建测试任务"):
            with self.subTest(transcript=transcript):
                result = handle_dev_mqtt_message(
                    self.topic("voice/audio"),
                    {"asr_engine": "nx-sensevoice", "transcript": transcript},
                )
                self.assertEqual(result["status"], "ignored")
        self.assertEqual(DevelopmentTask.objects.exclude(pk=self.task.pk).count(), 0)

    def test_short_alias_and_non_prefix_wake_phrase_do_not_create_task(self):
        self.task.status = "succeeded"
        self.task.save(update_fields=["status", "updated_at"])

        for transcript in ("小泰检查导航", "请小太阳检查导航"):
            result = handle_dev_mqtt_message(
                self.topic("voice/audio"),
                {"asr_engine": "nx-sensevoice", "transcript": transcript},
            )
            self.assertEqual(result["status"], "ignored")
        self.assertEqual(DevelopmentTask.objects.exclude(pk=self.task.pk).count(), 0)

    def test_wake_word_arms_and_acknowledges_without_creating_task(self):
        published = []
        with patch("monitoring.dev_message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/wake.mp3", False)):
            result = handle_dev_mqtt_message(
                self.topic("voice/audio"),
                {"asr_engine": "nx-sensevoice", "transcript": "小太阳"},
                publish=lambda *args: published.append(args),
            )

        self.assertEqual(result["status"], "armed")
        self.assertEqual(DevelopmentTask.objects.filter(robot=self.robot).count(), 1)
        self.assertEqual(published[0][0], self.topic("voice/ack"))

    def test_armed_window_keeps_short_garbled_wake_retry_out_of_codex(self):
        self.task.status = "succeeded"
        self.task.save(update_fields=["status", "updated_at"])
        published = []

        with patch("monitoring.dev_message_handlers.tts_service.synthesize_speech", return_value=("tts-audio/wake.mp3", False)):
            first = handle_dev_mqtt_message(
                self.topic("voice/audio"),
                {"asr_engine": "nx-sensevoice", "transcript": "小太阳"},
                publish=lambda *args: published.append(args),
            )
            retry = handle_dev_mqtt_message(
                self.topic("voice/audio"),
                {"asr_engine": "nx-sensevoice", "transcript": "小要大呀"},
                publish=lambda *args: published.append(args),
            )

        self.assertEqual(first["status"], "armed")
        self.assertEqual(retry["status"], "armed")
        self.assertEqual(DevelopmentTask.objects.filter(robot=self.robot).count(), 1)
        self.assertEqual(VoiceRecognitionEvent.objects.latest("id").outcome, "armed")
