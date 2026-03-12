docker run -it --rm \
    --runtime=nvidia \
    --gpus all \
    --shm-size 2gb \
    --network host \
    -e "ACCEPT_EULA=Y" \
    -e "OMNI_KIT_ALLOW_ROOT=1" \
    -e "ISAACSIM_PATH=/workspace/isaaclab/_isaac_sim" \
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    -e "LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib" \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/tmp/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/tmp/.Xauthority:ro \
    -v $(pwd):/workspace/isaaclab \
    --name isaac_lab_dev \
    isaac-lab-ros2:latest
