# NX Installation Assets

- `apt-packages.txt`: non-ROS operating-system dependencies.
- `ros-packages.txt`: ROS Humble binary dependencies.
- `python-requirements.txt`: Python service dependencies.
- `installed-system-packages.lock`: package/version snapshot of the validated NX.
- `models/`: deployable inference models kept under version control when size allows.
- `vendor-archives/`: original vendor SDK archives.
- `genisom_l1_sdk/`: installed vendor SDK runtime, created locally and ignored by Git.

Use `../scripts/install-system-deps.sh`; do not manually copy host Python
site-packages between NX devices because JetPack/CUDA ABI versions must match.

