cmake_minimum_required(VERSION 3.0.2)
project(obstacle_attack)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  graph_planner
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS graph_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/obstacle_attack.cpp 
# )

add_executable(${PROJECT_NAME} src/obstacle_attack.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)