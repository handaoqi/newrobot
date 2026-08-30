from __future__ import annotations

from django.db import transaction

from .models import AlertSkillBinding, SpeechCategory, SpeechTemplate


DEFAULT_SPEECH_TEMPLATES = (
    ("重点路段", "您好，当前区域为巡检重点路段，请勿长时间占道停留。", "监测告警"),
    ("注意避让", "您好，系统检测到现场存在安全风险，请注意避让并配合引导。", "监测告警"),
    ("驶离提醒", "您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。", "监测告警"),
    ("森林火灾", "共享森林美景，严防森林火灾", "巡检智能播报"),
    (
        "公园南门",
        "太阳宫南门入口设置蓝色健身步道起点，常举办春日牡丹主题亲子游园活动；临近便民休息座椅、饮水点，适配日常散步、亲子休闲",
        "巡检智能播报",
    ),
    ("赏花", "赏花不采花，文明你我他", "巡检智能播报"),
    (
        "无界公园介绍",
        "太阳宫公园是朝阳区首批试点改造的无界公园之一。无界公园的核心理念：拆除公园围墙、围栏，打破城市道路与绿地的物理边界，公园绿化景观与城市街区无缝融合，市民可随时随地就近进入绿地，不用专门寻找出入口",
        "巡检智能播报",
    ),
    ("发现障碍物", "前方巡检线路有障碍物,正在避让", "巡检智能播报"),
    ("后退尝试避障", "请注意避障离开巡检路线，并配合公园管理引导", "巡检智能播报"),
    ("劝阻离开线路", "任务受阻无法绕行，请您配合离开巡检线路", "巡检智能播报"),
    (
        "自动充电失败",
        "自动充电失败，请在机器人管理页查看问题提示，或手工更换电池",
        "设备管控",
    ),
    (
        "低电量停车告警",
        "当前电量低于百分之二十，导航任务已停止，请及时人工处理或手动回充。",
        "设备管控",
    ),
)

DEFAULT_ALERT_SKILLS = (
    ("bicycle_alert", "驶离提醒"),
    ("obstacle_detected", "发现障碍物"),
    ("avoidance", "后退尝试避障"),
    ("dissuasion", "劝阻离开线路"),
    ("low_battery_return_charge", "低电量停车告警"),
)


@transaction.atomic
def initialize_platform_configuration() -> dict[str, int]:
    """Create missing system configuration without overwriting operator edits."""
    created = {"categories": 0, "templates": 0, "alert_skills": 0}
    templates: dict[str, SpeechTemplate] = {}
    for name, text, category_name in DEFAULT_SPEECH_TEMPLATES:
        category, category_created = SpeechCategory.objects.get_or_create(name=category_name)
        created["categories"] += int(category_created)
        template, template_created = SpeechTemplate.objects.get_or_create(
            name=name,
            defaults={"text": text, "category": category},
        )
        created["templates"] += int(template_created)
        if template.category_id is None:
            template.category = category
            template.save(update_fields=["category", "updated_at"])
        templates[name] = template

    for skill_key, template_name in DEFAULT_ALERT_SKILLS:
        binding, binding_created = AlertSkillBinding.objects.get_or_create(
            skill_key=skill_key,
            defaults={"template": templates[template_name], "enabled": True},
        )
        created["alert_skills"] += int(binding_created)
        if binding.template_id is None:
            binding.template = templates[template_name]
            binding.save(update_fields=["template", "updated_at"])
    return created
