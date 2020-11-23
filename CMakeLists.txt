cmake_minimum_required(VERSION 2.8.3)
project(objectdetection)

set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



#set(SOURCE_FILES src/main.cpp src/gaus_blur.cpp src/gaus_blur.h src/ground_removal.cpp src/ground_removal.h src/component_clustering.cpp src/component_clustering.h src/box_fitting.cpp src/box_fitting.cpp src/ukf.cpp)

set(SOURCE_FILES src/cloudprocess.cpp include/cloudprocess.h src/road_removal.cpp include/road_removal.h src/rviz_visualizer.cpp include/rviz_visualizer.h src/gaus_blur.cpp include/gaus_blur.h src/ground_removal.cpp include/ground_removal.h src/component_clustering.cpp include/component_clustering.h src/box_fitting.cpp include/box_fitting.h)


add_executable(tracking_node src/tracking_node.cpp)
target_link_libraries(tracking_node ${MY_QT_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${CMAKE_THREAD_LIBS_INIT})


add_executable(main_detection src/main_detection.cpp ${SOURCE_FILES})
target_link_libraries(main_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
