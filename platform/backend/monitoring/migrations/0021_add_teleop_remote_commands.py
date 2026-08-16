from django.db import migrations, models


class Migration(migrations.Migration):
    dependencies = [
        ("monitoring", "0020_robotstatuslatest_localization_quality"),
    ]

    operations = [
        migrations.AlterField(
            model_name="remotecommand",
            name="command_type",
            field=models.CharField(
                choices=[
                    ("task.start", "启动任务"),
                    ("task.pause", "暂停任务"),
                    ("task.resume", "继续任务"),
                    ("task.cancel", "终止任务"),
                    ("mapping.start", "开始建图"),
                    ("mapping.save", "停止并保存地图"),
                    ("mapping.cancel", "取消建图"),
                    ("mapping.status", "查询建图状态"),
                    ("nav.status", "查询导航状态"),
                    ("nav.start", "启动导航栈"),
                    ("nav.restart", "重启导航栈"),
                    ("nav.recover", "恢复导航栈"),
                    ("nav.stop", "停止导航栈"),
                    ("nav.initial_pose", "设置初始定位"),
                    ("map.activate", "切换活动地图"),
                    ("teleop.takeover_enter", "进入远程接管"),
                    ("teleop.takeover_exit", "退出远程接管"),
                    ("teleop.stand_up", "站立"),
                    ("teleop.lie_down", "趴下"),
                    ("teleop.move_forward", "前进"),
                    ("teleop.move_backward", "后退"),
                    ("teleop.move_left", "左移"),
                    ("teleop.move_right", "右移"),
                    ("teleop.turn_left", "左转"),
                    ("teleop.turn_right", "右转"),
                    ("teleop.move_stop", "停止移动"),
                    ("teleop.passive", "软急停"),
                ],
                max_length=32,
            ),
        ),
    ]
