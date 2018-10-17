cmake_minimum_required(VERSION 2.8.3)

project(multisense_glview)

find_package(catkin REQUIRED COMPONENTS
  jpeg_utils bot_core_lcmtypes
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stereo_view
  src/stereo_view/stereo_view.cpp
)

add_dependencies(stereo_view ${catkin_EXPORTED_TARGETS})
target_link_libraries(stereo_view ${catkin_LIBRARIES}   GL png z glut)


add_executable(stereo_view_lcm
  src/stereo_view/stereo_view_lcm.cpp )
target_link_libraries(stereo_view_lcm ${catkin_LIBRARIES} stereo_view)


add_executable(stereo_view_ros
  src/stereo_view/stereo_view_ros.cpp )
target_link_libraries(stereo_view_ros ${catkin_LIBRARIES} stereo_view)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

## Declare a cpp executable
add_executable(
  multisense_glview
  src/stereo_view/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  multisense_glview
  ${catkin_LIBRARIES}
  GL png z glut
)

#############
## Install ##
#############

#install(TARGETS ${TARGETS}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


# Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
