cmake_minimum_required(VERSION 2.8.3)
project(locomotion_viewer)

find_package(catkin REQUIRED
  roscpp
  rviz_visual_tools
  )

find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz_visual_tools
)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} src/RvizPolygonsTools.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++0x -o3")     

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install xml plugin config
install(FILES package.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


