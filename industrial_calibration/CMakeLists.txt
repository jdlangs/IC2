cmake_minimum_required(VERSION 2.8.3)
project(industrial_calibration)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  image_transport 
  industrial_calibration_libs
  roscpp
  tf
  sensor_msgs
)

# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  industrial_calibration_libs
  roscpp
  tf
  sensor_msgs

  DEPENDS 
  Boost
  OpenCV
)

###########
## Build ##
###########
include_directories(
  include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)

set(COMPILER_WARNINGS PROPERTIES COMPILE_FLAGS "-pedantic -Wall -Wextra -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef -Wno-unused")

# Other nodes
add_executable(data_collector src/data_collector.cpp)
add_dependencies(data_collector ${catkin_EXPORTED_TARGETS})
set_target_properties(data_collector ${COMPILER_WARNINGS})
target_link_libraries(data_collector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(extrinsic_example src/extrinsic_example.cpp)
add_dependencies(extrinsic_example ${catkin_EXPORTED_TARGETS})
set_target_properties(extrinsic_example ${COMPILER_WARNINGS})
target_link_libraries(extrinsic_example ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(extrinsic_example_debug src/extrinsic_example_debug.cpp)
add_dependencies(extrinsic_example_debug ${catkin_EXPORTED_TARGETS})
set_target_properties(extrinsic_example_debug ${COMPILER_WARNINGS})
target_link_libraries(extrinsic_example_debug ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(intrinsic_example_mh3 src/intrinsic_example_mh3.cpp)
add_dependencies(intrinsic_example_mh3 ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_example_mh3 ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_example_mh3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(intrinsic_example_abb src/intrinsic_example_abb.cpp)
add_dependencies(intrinsic_example_abb ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_example_abb ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_example_abb ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(extrinsic_simulate src/extrinsic_simulate.cpp)
add_dependencies(extrinsic_simulate ${catkin_EXPORTED_TARGETS})
set_target_properties(extrinsic_simulate ${COMPILER_WARNINGS})
target_link_libraries(extrinsic_simulate ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

# Research
add_executable(intrinsic_opencv src/research/intrinsic_opencv.cpp)
add_dependencies(intrinsic_opencv ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_opencv ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_opencv ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(intrinsic_opencv_single src/research/intrinsic_opencv_single.cpp)
add_dependencies(intrinsic_opencv_single ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_opencv_single ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_opencv_single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp ${OpenMP_LIBS})
target_compile_options(intrinsic_opencv_single PRIVATE ${OpenMP_FLAGS})

add_executable(intrinsic_ic2 src/research/intrinsic_ic2.cpp)
add_dependencies(intrinsic_ic2 ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_ic2 ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_ic2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(intrinsic_ic2_single src/research/intrinsic_ic2_single.cpp)
add_dependencies(intrinsic_ic2_single ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_ic2_single ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_ic2_single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp ${OpenMP_LIBS})
target_compile_options(intrinsic_ic2_single PRIVATE ${OpenMP_FLAGS})

add_executable(intrinsic_ic2_sim src/research/intrinsic_ic2_sim.cpp)
add_dependencies(intrinsic_ic2_sim ${catkin_EXPORTED_TARGETS})
set_target_properties(intrinsic_ic2_sim ${COMPILER_WARNINGS})
target_link_libraries(intrinsic_ic2_sim ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)

add_executable(data_cleaner src/research/data_cleaner.cpp)
add_dependencies(data_cleaner ${catkin_EXPORTED_TARGETS})
set_target_properties(data_cleaner ${COMPILER_WARNINGS})
target_link_libraries(data_cleaner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
  ${Boost_LIBRARIES} yaml-cpp)