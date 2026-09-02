import os
from unittest.mock import patch

from django.contrib.auth import get_user_model
from django.core.management import call_command
from django.core.management.base import CommandError
from django.test import TestCase, override_settings

from .bootstrap import DEFAULT_SPEECH_TEMPLATES
from .models import AlertSkillBinding, MapData, PatrolTask, Robot, SpeechTemplate
from .views import ensure_demo_seed


class PlatformInitializationTests(TestCase):
    @override_settings(ENABLE_DEMO_SEED=False)
    def test_page_seed_hook_is_disabled_by_default(self):
        ensure_demo_seed()

        self.assertEqual(get_user_model().objects.count(), 0)
        self.assertEqual(Robot.objects.count(), 0)
        self.assertEqual(MapData.objects.count(), 0)

    @patch.dict(
        os.environ,
        {
            "PLATFORM_OPERATOR_USERNAME": "operator",
            "PLATFORM_OPERATOR_PASSWORD": "test-only-password",
            "PLATFORM_ROBOT_CODE": "RX-INIT-01",
            "PLATFORM_ROBOT_NAME": "Initialization Robot",
        },
        clear=False,
    )
    def test_initialization_is_idempotent_and_does_not_create_demo_routes(self):
        call_command("initialize_platform")
        call_command("initialize_platform")

        self.assertEqual(get_user_model().objects.filter(username="operator").count(), 1)
        self.assertEqual(Robot.objects.filter(code="RX-INIT-01").count(), 1)
        self.assertEqual(AlertSkillBinding.objects.count(), 5)
        expected_names = {template[0] for template in DEFAULT_SPEECH_TEMPLATES}
        self.assertEqual(len(expected_names), 12)
        self.assertEqual(SpeechTemplate.objects.count(), 12)
        self.assertFalse(SpeechTemplate.objects.filter(name="低电量自动回充").exists())
        actual_templates = {
            name: (text, category)
            for name, text, category in SpeechTemplate.objects.filter(
                name__in=expected_names
            ).values_list("name", "text", "category__name")
        }
        self.assertEqual(
            actual_templates,
            {name: (text, category) for name, text, category in DEFAULT_SPEECH_TEMPLATES},
        )
        self.assertEqual(MapData.objects.count(), 0)
        self.assertEqual(PatrolTask.objects.count(), 0)

    @patch.dict(os.environ, {"PLATFORM_OPERATOR_PASSWORD": ""}, clear=False)
    def test_first_operator_requires_explicit_password(self):
        with self.assertRaisesMessage(CommandError, "PLATFORM_OPERATOR_PASSWORD"):
            call_command("initialize_platform", skip_robot=True)
