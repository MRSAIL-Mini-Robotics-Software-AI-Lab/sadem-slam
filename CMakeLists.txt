cmake_minimum_required(VERSION 3.0.2)
project(sadem_slam)

find_package(catkin REQUIRED
        COMPONENTS roscpp
        rospy
        sensor_msgs
        cv_bridge
        std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sadem_slam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})


# catkin_python_setup()

catkin_install_python(PROGRAMS nodes/zed_pub.py nodes/zed_sdk_pub.py nodes/orb_slam.py DESTINATION
                      ${CATKIN_PACKAGE_BIN_DESTINATION})