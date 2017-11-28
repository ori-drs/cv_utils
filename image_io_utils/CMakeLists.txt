cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

project(image_io_utils)

find_package(catkin REQUIRED COMPONENTS
  jpeg_utils bot_core_lcmtypes
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES image_io_utils
  CATKIN_DEPENDS jpeg_utils
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

add_library(${PROJECT_NAME} src/image_io_utils.cpp)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
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
