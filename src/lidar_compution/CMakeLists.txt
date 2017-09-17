cmake_minimum_required(VERSION 2.8.3)
project(lidar_compution)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
   	roscpp
   	message_generation
   	pcl_conversions
   	pcl_ros
	roscpp
)
catkin_package(
	CATKIN_DEPENDS message_runtime pcl_ros roscpp pcl_conversions
)

include_directories( 
	${PCL_INCLUDE_DIRS} 
	${catkin_INCLUDE_DIRS} 
)

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( filter src/filter.cpp)
target_link_libraries( filter 
	${PCL_LIBRARIES} 
	${catkin_LIBRARIES}
)
add_executable( visualize src/visualize.cpp)
target_link_libraries( visualize
	${PCL_LIBRARIES} 
	${catkin_LIBRARIES}
)
add_executable( antiCollision src/antiCollision.cpp)
target_link_libraries( antiCollision
	${PCL_LIBRARIES} 
	${catkin_LIBRARIES}
)
