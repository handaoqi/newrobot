from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0009_mapdata_patrolroute_track_zone"),
    ]

    operations = [
        migrations.AlterField(
            model_name="robotcommand",
            name="action",
            field=models.CharField(
                choices=[
                    ("shake_hand", "握手"),
                    ("stand_up", "站立"),
                    ("lie_down", "趴下"),
                    ("move_forward", "前进"),
                    ("move_backward", "后退"),
                    ("move_left", "左移"),
                    ("move_right", "右移"),
                    ("turn_left", "左转"),
                    ("turn_right", "右转"),
                    ("takeover_enter", "进入远程接管"),
                    ("takeover_exit", "退出远程接管"),
                    ("move_stop", "停止移动"),
                    ("passive", "软急停"),
                    ("jump", "原地跳"),
                    ("front_jump", "向前跳"),
                    ("backflip", "后空翻"),
                    ("two_leg_stand", "双腿站立"),
                    ("cancel_two_leg_stand", "取消双腿站立"),
                    ("attitude_control", "姿态控制"),
                    ("play_audio", "播放音频"),
                ],
                max_length=32,
            ),
        ),
        migrations.AlterField(
            model_name="robotcommand",
            name="status",
            field=models.CharField(
                choices=[
                    ("queued", "待发送"),
                    ("sent", "已发送"),
                    ("running", "执行中"),
                    ("finished", "已完成"),
                    ("failed", "发送失败"),
                ],
                default="queued",
                max_length=16,
            ),
        ),
    ]
