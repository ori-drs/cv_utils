cmake_minimum_required(VERSION 2.8.3)
project(jpeg_utils)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jpeg_utils
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS jpeg
)

###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

######################################################
add_library(${PROJECT_NAME} src/jpeg-utils.cpp  src/jpeg-utils-ijg.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} jpeg
)


