# ==================================
# Stage 1 — Build bridge
# ==================================
FROM ros:humble as builder

RUN apt update && apt install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    curl

RUN rosdep update

WORKDIR /workspace

# Clone correct branch
RUN git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds -b release/1.7.2

WORKDIR /workspace/zenoh-plugin-ros2dds

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Build only bridge package
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select zenoh_bridge_ros2dds --cmake-args -DCMAKE_BUILD_TYPE=Release


# ==================================
# Stage 2 — Runtime
# ==================================
FROM ros:humble

WORKDIR /workspace

COPY --from=builder /workspace/zenoh-plugin-ros2dds/install /workspace/install

ENV ZENOH_CONFIG=/workspace/config.json

ENTRYPOINT ["/workspace/install/zenoh_bridge_ros2dds/lib/zenoh_bridge_ros2dds/zenoh_bridge_ros2dds"]
CMD ["-c", "/workspace/config.json"]
