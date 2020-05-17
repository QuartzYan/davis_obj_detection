cmake_minimum_required(VERSION 2.8.3)
project(davis_obj_detection)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


#########################
# make 
#########################

cs_add_executable(davis_obj_detection
  src/davis_obj_detection_node.cpp
  src/davis_obj_detection.cpp
)

# link the executable to the necesarry libs
target_link_libraries(davis_obj_detection
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

