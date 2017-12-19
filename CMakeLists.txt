cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_calibration)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CXX_COMPILE_FLAGS "-std=c++11 -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_COMPILE_FLAGS}")

###########
## Build ##
###########

include_directories(
  INCLUDE include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibrator_pcl 
  test/calibrator_pcl.cpp
  src/calibrator.cpp
  src/marker_detector.cpp
)
target_link_libraries(
  calibrator_pcl
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
