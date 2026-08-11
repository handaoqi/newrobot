from __future__ import annotations

from monitoring.models import AlertSkillBinding, SpeechTemplate


def resolve_alert_template(skill_key: str, fallback_name: str) -> SpeechTemplate | None:
    binding = AlertSkillBinding.objects.select_related("template").filter(skill_key=skill_key).first()
    if binding is not None:
        return binding.template if binding.enabled else None
    return SpeechTemplate.objects.filter(name=fallback_name).first()
