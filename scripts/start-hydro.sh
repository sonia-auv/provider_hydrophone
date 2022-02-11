#! /bin/sh
# Launch the container on the Jetson TX2 for testing

export ROS_IP=$(hostname -I | awk '{print $1}');
export AUV=LOCAL;

echo -n "Enter the ROS_MASTER_IP_ADRESS : ";
read;

export ROS_MASTER_URI=http://${REPLY}:11311;

docker rm $(docker ps -a -q);

echo "Starting Container"

docker run --name provider_hydro -it --privileged -e AUV -e ROS_MASTER_URI -e ROS_IP --network=host --mount type=bind,src=/dev,dst=/dev  --mount type=bind,src=/home/sonia/Documents/bags,dst=/home/sonia/ros_sonia_ws/src/bags ghcr.io/sonia-auv/provider_hydrophone/provider_hydrophone:arm64-perception-feature-noetic bash;
