cmake_minimum_required(VERSION 2.8.3)
project(thormang3_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
  tf
  robotis_math
  eigen_conversions
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(thormang3_localization
  src/main.cpp
  src/thormang3_localization.cpp
)

add_dependencies(thormang3_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(thormang3_localization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

