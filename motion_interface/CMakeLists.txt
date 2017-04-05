cmake_minimum_required(VERSION 2.8.3)
project(motion_interface)

find_package(catkin REQUIRED COMPONENTS
    motion_msgs
    rospy)

# install python modules
catkin_python_setup()

# install scripts
install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
