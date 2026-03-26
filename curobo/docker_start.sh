docker run -it \
  --rm \
  --net=host \
  --privileged \
  --shm-size=4gb \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_VISIBLE_DEVICES=all" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --env="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd)/ws:/root/ws:rw" \
  --runtime=nvidia \
  --gpus all \
  --name curobo_dev \
  curobo-ros2:latest
