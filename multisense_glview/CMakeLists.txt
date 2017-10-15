cmake_minimum_required(VERSION 2.8.3)

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()
project(multisense_glview)

find_package(catkin REQUIRED COMPONENTS
  jpeg_utils
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


pods_use_pkg_config_packages(
  multisense_glview lcmtypes_bot2-core
  )

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
