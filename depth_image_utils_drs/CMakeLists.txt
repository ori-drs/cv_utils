cmake_minimum_required(VERSION 3.0.2)
project(depth_image_utils_drs)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  image_utils_drs
  image_transport
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################

add_library(${PROJECT_NAME} src/depth_image_utils.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)


######################################################
add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp)
target_link_libraries(depth_to_pointcloud_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(downsample_depth_image
  src/downsample_depth_image.cpp)
target_link_libraries(downsample_depth_image ${catkin_LIBRARIES} ${PROJECT_NAME})


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

