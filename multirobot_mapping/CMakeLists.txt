# 
# Adapted from mcl_ws/uml_mcl/CMakeLists.txt found at https://github.com/DeepBlue14/mcl_ws.git
#
cmake_minimum_required(VERSION 2.8.3)
project(multirobotmapping)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-sgd=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-sgd=c++0x" COMPILER_SUPPORTS_CXX0x)

if(COMPILER_SUPPORTS_CXX11)
    message(STATUS "Linking -std=c++11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    message(STATUS "Linking -std=c++0x")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11/0x support.")
endif(COMPILER_SUPPORTS_CXX11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  stage_ros
)

catkin_package()


include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)


set(HDR_DIR include/grid_fusion)
set(SRC_DIR src)

set(HDRS
    ${HDR_DIR}/Robot.hpp
    ${HDR_DIR}/Point.hpp
    ${HDR_DIR}/Helpers.hpp
)

set(SRCS
    ${SRC_DIR}/Robot.cpp
    ${SRC_DIR}/Robot_Runner.cpp
    ${SRC_DIR}/Point.cpp
    ${SRC_DIR}/Helpers.cpp
)

set(F_HDRS
    ${HDR_DIR}/Fuser.hpp
)

set(F_SRCS
    ${SRC_DIR}/Fuser.cpp
    ${SRC_DIR}/Fuser_Runner.cpp
)

add_executable(Robot_Runner ${HDRS} ${SRCS})
target_link_libraries(Robot_Runner ${catkin_LIBRARIES})
add_executable(Fuser_Runner ${F_HDRS} ${F_SRCS})
target_link_libraries(Fuser_Runner ${catkin_LIBRARIES})
