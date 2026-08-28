FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap \
      ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/roamerx-validation
COPY requirements.txt ./
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY analyzer.py backends.py runner.py ./
ENTRYPOINT ["python3", "/opt/roamerx-validation/runner.py"]
