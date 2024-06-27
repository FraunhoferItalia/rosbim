export WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

printf "Loading workspace environment in folder $(basename $WS_ROOT)\n"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/$ROS_DISTRO/setup.bash
source $ROS_WS_DIR/install/setup.bash
