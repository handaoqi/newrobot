# Hardware-Specific Runtime Dependencies

The NX validated baseline is Jetson Linux R36.4.7 (aarch64), CUDA 12.6 and
TensorRT 10.3. These packages are included in `installed-system-packages.lock`
but must come from the matching approved JetPack image/repository, not generic
Ubuntu mirrors.

Additional non-Debian dependencies:

| Component | Validated path | Source |
| --- | --- | --- |
| Genisom L1 SDK | `runtime/nx-edge/install/genisom_l1_sdk` | Extracted from the tracked vendor archives |
| LCM CMake/runtime | `/usr/local/lib/lcm` | Approved NX base image |
| ZSIBOT static library | `/usr/local/lib/libzsibot.a` | Approved NX base image/vendor SDK |
| Livox Mid-360 driver | ROS workspace/base image | Build from the repository package or approved image |
| eCAL 5.13.3 | system package | Approved arm64 package source |

Do not copy CUDA, TensorRT, LCM or eCAL binaries from x86 or a different
JetPack release. Reflash/restore the approved NX image first, then install this
repository's package manifests and vendor SDK.

