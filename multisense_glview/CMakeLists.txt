cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

#pkg_search_module (GLIB2   glib-2.0)


project(multisense_glview)

find_package(catkin REQUIRED COMPONENTS
  jpeg_utils
)

catkin_package(
#  INCLUDE_DIRS
#    include
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(TARGETS
    multisense_glview
)


include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
)

## Declare a cpp executable
add_executable(
  multisense_glview
  src/main.c
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  multisense_glview
  ${catkin_LIBRARIES}
  glib-2.0 glut GL m jpeg_utils png z
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
