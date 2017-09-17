cmake_minimum_required(VERSION 2.8.3)
project(anchorpoint)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   	roscpp
   	message_generation
	roscpp
)
catkin_package( CATKIN_DEPENDS 
	message_runtime
	roscpp 
)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable( anchor_compution src/anchor_compution.cpp )
target_link_libraries( anchor_compution ${catkin_LIBRARIES} )
