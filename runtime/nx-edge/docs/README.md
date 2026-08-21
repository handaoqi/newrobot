# NX Edge Runtime

Canonical source and ROS build output remain in `/home/dogrobot`. Mutable data
and machine-local configuration live here:

| Path | Contents |
| --- | --- |
| `data/jszr` | 3D/2D maps, map replay and active-map links |
| `data/rosbags` | Mapping and navigation recordings |
| `data/edge-agent` | Edge SQLite database and power-mode state |
| `data/voice` | ASR virtual environment and large speech models |
| `data/cache/roamerx` | TensorRT/ONNX generated engine cache |
| `data/robot-state` | Vendor parameters, caches and application logs |
| `data/ros-home` | ROS home state and logs |
| `conf` | Edge, RTK and monitoring configuration and credentials |
| `install` | OS dependency manifests, model files and vendor SDKs |

The migration script leaves compatibility links under `/home/robot`, allowing
older vendor binaries and historical map YAML files to continue resolving while
new code uses the canonical runtime paths.
