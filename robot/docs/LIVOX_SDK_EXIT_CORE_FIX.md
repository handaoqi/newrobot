# Livox driver exit core fix

## Problem

`livox_driver_node` could receive SIGTERM, finish normal ROS shutdown, and then
segfault while destroying a `spdlog::logger`. Livox-SDK2 embedded its vendored
header-only spdlog/fmt implementation while ROS 2 loaded the distribution
`libspdlog.so.1` and `libfmt.so.8` into the same process.

## Reproducible build

The deployed SDK is based on Livox-SDK2 tag `v1.3.1` (`f5d9375`). Apply the
repository patch before configuring the build:

```bash
git clone --branch v1.3.1 https://github.com/Livox-SDK/Livox-SDK2.git
git -C Livox-SDK2 apply \
  /home/dogrobot/robot/patches/livox-sdk2-v1.3.1-system-spdlog.patch
cmake -S Livox-SDK2 -B Livox-SDK2/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build Livox-SDK2/build --parallel
```

The resulting `build/sdk_core/liblivox_lidar_sdk_shared.so` must dynamically
depend on the system `libspdlog.so.1` and `libfmt.so.8`. Verify all Livox API
symbols required by `/opt/robot-driver/install/livox_driver/lib/liblivox_driver.so`
before deployment.

## Deployment record

On 2026-09-10 the rebuilt shared library was atomically installed at
`/usr/local/lib/liblivox_lidar_sdk_shared.so`. The previous library is retained
at `/usr/local/lib/liblivox_lidar_sdk_shared.so.roamerx-backup-20260910-2323`.

Validation covered SDK logger teardown, a real `livox_driver_node` SIGTERM,
the complete sensor restart script, `/front_lidar` at about 10 Hz, and
`/front_lidar/imu` at about 200 Hz. No new Livox core was generated.
