##
# Source this file from the VM's .bashrc
#
# Make sure your system is configured as described in INSTRUCTIONS_MAC.md.
## 

# This makes RViz and Gazebo work better in Parallels, for some reason
# https://forum.parallels.com/threads/ros-rviz-opengl-problem-on-ubuntu-20-04.351631/page-2#post-899436
export LIBGL_ALWAYS_SOFTWARE=1

# From https://unix.stackexchange.com/a/115431
export COMPROBO=${0:a:h}
CATKIN_WS=$(cd "$COMPROBO/../.." > /dev/null; pwd)

# Make sure Gazebo can find our stuff in its weird place
export GAZEBO_MODEL_PATH=$COMPROBO/*/model

source /opt/ros/noetic/setup.zsh
source $CATKIN_WS/devel/setup.zsh

# Make sure ROS can find our stuff in its weird place
# This must be after sourcing ROS's setup.bash
export ROS_PACKAGE_PATH=$COMPROBO:$ROS_PACKAGE_PATH

function comprobocd() {
	cd $COMPROBO
}
