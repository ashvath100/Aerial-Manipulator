#!/bin/bash

. $(echo $PX4_DIR)/Tools/setup_gazebo.bash $PX4_DIR $(echo $PX4_DIR)/build/px4_sitl_default

export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$PX4_DIR
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:($PX4_DIR)/Tools/sitl_gazebo

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_DIR
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(echo $PX4_DIR)/Tools/sitl_gazebo

echo AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH
echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH
