cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

# find catkin packages
find_package(catkin REQUIRED COMPONENTS  
										pcl_conversions
                    					pcl_ros
                    					roscpp
                    					sensor_msgs
										geometry_msgs
										rospy
										std_msgs										
)
                    					
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

catkin_package()

add_executable(obstacle_detection src/obstacle_detection.cpp)
#add_executable(pcl_filter src/pcl_filter.cpp)
target_link_libraries(obstacle_detection ${PCL_LIBRARIES})
target_link_libraries(obstacle_detection ${catkin_LIBRARIES})
target_link_libraries(obstacle_detection ${pcl_segmentation})

