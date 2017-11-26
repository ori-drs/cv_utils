cmake_minimum_required(VERSION 2.8.3)
project(multisense_utils)

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  jpeg_utils
  multisense_image_utils
  pcl_ros
  bot_core_lcmtypes
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multisense_utils
  CATKIN_DEPENDS cv_bridge jpeg_utils multisense_image_utils pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/multisense_utils.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test-multisense-image-filtering src/test-multisense-image-filtering.cpp)

add_dependencies(test-multisense-image-filtering ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test-multisense-image-filtering
  ${catkin_LIBRARIES}
)
pods_use_pkg_config_packages(test-multisense-image-filtering path-util)




###########
# Install #
###########
install(TARGETS multisense_utils
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})



install(DIRECTORY include/multisense_utils/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)