# cuRobo + ROS2 Humble Docker
#
# 베이스: NVIDIA PyTorch (CUDA + PyTorch 포함)
# 추가: ROS2 Humble + cuRobo pip 설치
#
# 빌드:
#   cd curobo && docker build -t curobo-ros2:latest .
#
# 실행:
#   ./docker_start.sh

FROM pytorch/pytorch:2.4.0-cuda12.4-cudnn9-devel

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 미러 설정
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list

# ── ROS2 Humble 설치 ────────────────────────────────────
RUN apt-get update && apt-get install -y \
      software-properties-common curl gnupg lsb-release && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --batch --dearmor -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y \
      ros-humble-ros-base \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-sensor-msgs \
      ros-humble-geometry-msgs \
      ros-humble-std-srvs \
      ros-humble-trajectory-msgs \
      ros-humble-control-msgs \
      ros-humble-tf2-ros \
      ros-humble-visualization-msgs \
      python3-colcon-common-extensions \
      python3-rosdep && \
    rm -rf /var/lib/apt/lists/*

# rosdep 초기화
RUN rosdep init 2>/dev/null || true && rosdep update

# ── cuRobo 설치 ──────────────────────────────────────────
RUN pip install --no-cache-dir "nvidia-curobo[all]"

# ── 환경 설정 ────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/install/setup.bash 2>/dev/null" >> /root/.bashrc

WORKDIR /root/ws
