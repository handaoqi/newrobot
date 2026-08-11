# RK3588 Controller Deployment

The RK3588 controller contains vendor-supplied motion-control software and is
not rebuilt from this repository. A new robot must start from the approved
factory image containing `robot-launch`, motion-control eggs, camera services,
power telemetry, and the GENISOM SDK.

Project-managed additions, such as the charging-pile runtime and USB audio
configuration, should be installed from a versioned device asset bundle. Do
not place device credentials or vendor binaries in Git. Verify the controller
with `robot-launch list` before enabling navigation from the NX computer.
