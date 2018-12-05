cmake_minimum_required(VERSION 2.8.3)
project(the_inspector)
add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  map_server
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(the_inspector src/ImageProcessingModule.cpp src/main.cpp src/PathPlanningModule.cpp src/RobotModule.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(the_inspector ${catkin_LIBRARIES})
target_link_libraries(the_inspector ${OpenCV_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage")
  add_rostest_gtest(
   test_the_inspector
   test/inspect.launch 
   test/main.cpp
   test/RobotModuleTest.cpp 
   test/PathPlanningModuleTest.cpp 
   test/ImageProcessingModuleTest.cpp
   src/RobotModule.cpp 
   src/ImageProcessingModule.cpp
   src/PathPlanningModule.cpp )

  target_link_libraries(
   test_the_inspector
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS})
  
  add_dependencies(
    the_inspector
    test_the_inspector
    ${catkin_EXPORTED_TARGETS}
  )
endif()

