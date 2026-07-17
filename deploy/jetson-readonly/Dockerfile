# Supplied by build.sh after it verifies an offline-loaded ARM64 image ID.  The
# local tag embeds that complete ID; upstream registry provenance is recorded
# separately in the final image labels.
ARG ROS_IMAGE

FROM --platform=linux/arm64 ${ROS_IMAGE} AS builder

ARG TARGETARCH
ENV DEBIAN_FRONTEND=noninteractive

RUN test "${TARGETARCH}" = "arm64"
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        binutils \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        ros-humble-cv-bridge \
        ros-humble-diagnostic-msgs \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-rclcpp \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-generator-dds-idl \
        ros-humble-sensor-msgs \
        ros-humble-std-msgs \
        ros-humble-std-srvs \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/ros2/src

# The Dockerfile-specific ignore file exposes only this source allowlist to the
# build.  In particular, Unitree/Robonix examples and motion services are not
# in the build context.
COPY third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api unitree_api
COPY third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go unitree_go
COPY packages/go2_chassis/ros2_ws/src/go2_chassis_adapter go2_chassis_adapter
COPY packages/go2_chassis/include/go2_chassis /work/include/go2_chassis
COPY packages/go2_sensors go2_sensors
COPY packages/go2_description go2_description

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && colcon build \
        --merge-install \
        --install-base /opt/robonix/overlay \
        --cmake-args -DBUILD_TESTING=OFF \
        --packages-select \
          unitree_api \
          unitree_go \
          go2_chassis_adapter \
          go2_sensors \
          go2_description

# Build only the VideoClient camera reader from the vendor SDK.  BUILD_EXAMPLES
# is forced off by this package, and the final stage receives only the camera
# component, not the SDK archive, headers, examples, or any sport daemon.
COPY third_party/unitree_sdk2 /work/unitree_sdk2
RUN cmake \
      -S /work/ros2/src/go2_sensors/camera_daemon \
      -B /work/camera-build \
      -DUNITREE_SDK2_ROOT=/work/unitree_sdk2 \
      -DBUILD_EXAMPLES=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/robonix/camera \
    && cmake --build /work/camera-build --parallel --target go2_camera_daemon \
    && cmake --install /work/camera-build --component go2_camera_runtime \
    && ! nm -C --defined-only /opt/robonix/camera/bin/go2_camera_daemon \
         | grep -E 'SportClient|SportModeClient|MotionSwitcherClient|LowCmd' \
    && ! strings /opt/robonix/camera/bin/go2_camera_daemon \
         | grep -E '/api/sport/request|(^|/)lowcmd($|[^[:alnum:]_])'

FROM --platform=linux/arm64 ${ROS_IMAGE} AS runtime

ARG TARGETARCH
ARG VCS_REF=unknown
ARG BASE_IMAGE_ID=unknown
ARG BASE_IMAGE_UPSTREAM_DIGEST=unknown
ENV DEBIAN_FRONTEND=noninteractive

RUN test "${TARGETARCH}" = "arm64"
RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        iproute2 \
        ros-humble-cv-bridge \
        ros-humble-diagnostic-msgs \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-rclcpp \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-robot-state-publisher \
        ros-humble-rosidl-default-runtime \
        ros-humble-sensor-msgs \
        ros-humble-std-msgs \
        ros-humble-std-srvs \
        ros-humble-tf2 \
        ros-humble-tf2-ros \
    && for forbidden in \
         ros-humble-navigation2 \
         ros-humble-nav2-bringup \
         ros-humble-slam-toolbox \
         ros-humble-rtabmap-ros; do \
         if dpkg-query --show "${forbidden}" >/dev/null 2>&1; then \
           echo "forbidden runtime package installed: ${forbidden}" >&2; \
           exit 1; \
         fi; \
       done \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid 10001 robonix \
    && useradd \
       --uid 10001 \
       --gid 10001 \
       --no-create-home \
       --home-dir /tmp/home \
       --shell /usr/sbin/nologin \
       robonix \
    && install -d -o root -g root -m 0755 /opt/robonix/profile

COPY --from=builder /opt/robonix/overlay /opt/robonix/overlay
COPY --from=builder /opt/robonix/camera /opt/robonix/camera
COPY deploy/jetson-readonly/config /opt/robonix/profile/config
COPY deploy/jetson-readonly/entrypoint.sh /opt/robonix/profile/entrypoint.sh
COPY deploy/jetson-readonly/healthcheck.sh /opt/robonix/profile/healthcheck.sh
COPY deploy/jetson-readonly/validate-network.sh /opt/robonix/profile/validate-network.sh
COPY deploy/jetson-readonly/verify-runtime.sh /opt/robonix/profile/verify-runtime.sh

RUN chmod 0555 \
      /opt/robonix/profile/entrypoint.sh \
      /opt/robonix/profile/healthcheck.sh \
      /opt/robonix/profile/validate-network.sh \
      /opt/robonix/profile/verify-runtime.sh \
    && /opt/robonix/profile/verify-runtime.sh \
    && rm /opt/robonix/profile/verify-runtime.sh

LABEL org.opencontainers.image.title="Robonix Go2 Jetson read-only runtime" \
      org.opencontainers.image.description="CPU-only passive state, sensors, camera and description profile" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.source="https://github.com/syswonder/robot-unitree-go2" \
      org.opencontainers.image.base.name="docker.io/library/ros:humble-ros-base-jammy" \
      org.opencontainers.image.base.digest="${BASE_IMAGE_UPSTREAM_DIGEST}" \
      io.robonix.go2.base.image-id="${BASE_IMAGE_ID}" \
      io.robonix.go2.profile="jetson-readonly" \
      io.robonix.go2.motion="disabled"

ENV HOME=/tmp/home \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=0 \
    ROS_LOCALHOST_ONLY=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    NVIDIA_VISIBLE_DEVICES=void \
    NVIDIA_DRIVER_CAPABILITIES= \
    GO2_ENABLE_CAMERA=0

USER 10001:10001
WORKDIR /tmp

HEALTHCHECK --interval=10s --timeout=3s --start-period=20s --retries=3 \
  CMD ["/opt/robonix/profile/healthcheck.sh"]

ENTRYPOINT ["/opt/robonix/profile/entrypoint.sh"]
