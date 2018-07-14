cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_calibration)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

set(CXX_COMPILE_FLAGS "-std=c++11 -w")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_COMPILE_FLAGS}")

###########
## Build ##
###########

include_directories(
  INCLUDE include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibration_test 
  test/calibration_test.cpp
  src/calibrator.cpp
  src/marker_detector.cpp
)
target_link_libraries(
  calibration_test
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

#add_executable(to_transform_matrix test/to_transform_matrix.cpp)
#target_link_libraries(to_transform_matrix ${PCL_LIBRARIES})
