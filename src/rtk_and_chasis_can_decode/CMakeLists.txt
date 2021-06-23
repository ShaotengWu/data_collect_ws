cmake_minimum_required(VERSION 3.0.2)
project(rtk_and_chasis_can_decode)

add_compile_options(-std=c++14)

# 库文件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  location_msgs
  control_msgs
)
find_package(Threads  REQUIRED)
link_directories(${catkin_LIB_DIRS} dll) 

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS 
  std_msgs
  location_msgs
  control_msgs
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanCommonFcn.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanHandler.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CanMsgDefs.cc
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanCommonFcn.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanHandler.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/CanMsgDefs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ICANCmd.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/canstream.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rtknavigation.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/uwb.h
    )

include_directories(
    include 
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
)

# 创建库文件
add_library(canlib src/CanHandler.cc src/CanMsgDefs.cc src/CanCommonFcn.cc)

# 创建可执行文件
add_executable(rtk_and_chasis_can_decode src/main.cc ${HEADERS} ${SOURCES})
target_link_libraries(rtk_and_chasis_can_decode ${CMAKE_CURRENT_SOURCE_DIR}/dll/libCanCmd.so ${catkin_LIBRARIES} pthread glog)
add_dependencies(rtk_and_chasis_can_decode ${catkin_EXPORTED_TARGETS})







