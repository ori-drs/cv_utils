cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()
#set(POD_NAME cv_utils)

project(multisense_utils)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros multisense_image_utils jpeg_utils
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pcl_ros multisense_image_utils jpeg_utils
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

######################################################

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(${PROJECT_NAME} src/multisense_utils.cpp)

target_link_libraries(${PROJECT_NAME} jpeg_utils multisense_image_utils ${OpenCV_LIBRARIES})# ${LCMTYPES_LIBS})

#add_dependencies(${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)

pods_use_pkg_config_packages(${PROJECT_NAME} lcmtypes_bot2-core)


add_executable(test-multisense-image-filtering src/test-multisense-image-filtering.cpp)
target_link_libraries(test-multisense-image-filtering ${catkin_LIBRARIES}
   multisense_image_utils lcm ${OpenCV_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
