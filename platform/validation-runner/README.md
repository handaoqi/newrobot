# RoamerX Validation Runner

独立 CPU Runner 通过平台租约领取 `bag_replay` 作业。默认只做 MCAP 契约、时间、TF、
里程计和控制检查；当受信任的 `ValidationProfile.runner_config.launch_command` 存在时，
会在独立 ROS Domain 中启动算法栈，只回放 Profile 的输入白名单，并录制派生输出后检查。

```bash
export ROAMERX_PLATFORM_URL=https://platform.example.com
export ROAMERX_VALIDATION_RUNNER_TOKEN='<与平台一致的随机密钥>'
export ROAMERX_VALIDATION_RUNNER_ID=cpu-runner-1
python3 runner.py
```

Runner 不接受作业中的 shell、环境变量、路径或镜像参数。可执行命令只来自管理员维护的
Profile，且入口仅允许 `ros2` 或固定的 `/opt/roamerx/bin/validation-launch`。
`matrix_scenario` 由 `SimulationBackend` 接口预留；没有 GPU Runner 时平台返回
`RUNNER_UNAVAILABLE`。
