cmake_minimum_required(VERSION 2.8.3)

project(multisense_glview)

find_package(catkin REQUIRED COMPONENTS
  jpeg_utils bot_core_lcmtypes
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

## Declare a cpp executable
add_executable(
  multisense_glview
  src/main.cpp
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
