cmake_minimum_required(VERSION 2.8.3)
project(industrial_calibration_gui)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  image_transport 
  industrial_calibration_libs
  roscpp
  tf
  sensor_msgs
  rqt_gui
  rqt_gui_cpp
  rviz
)

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  industrial_calibration_libs
  roscpp
  tf
  sensor_msgs
  rqt_gui 
  rqt_gui_cpp
  rviz  
  DEPENDS 
  Boost
  OpenCV 
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIR}
)

set(COMPILER_WARNINGS PROPERTIES COMPILE_FLAGS "-pedantic -Wall -Wextra -Wcast-align -Wcast-qual -Wctor-dtor-privacy -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-declarations -Wmissing-include-dirs -Wnoexcept -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-conversion -Wsign-promo -Wstrict-null-sentinel -Wstrict-overflow=5 -Wswitch-default -Wundef -Wno-unused")

set(${PROJECT_NAME}_SRCS
  src/calibration_panel.cpp
  src/calibration_widget.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/industrial_calibration_gui/calibration_panel.h
  include/industrial_calibration_gui/calibration_widget.h
)

set(${PROJECT_NAME}_UI
  src/calibration_widget.ui
)

qt5_wrap_cpp(industrial_calibration_gui_MOCS ${industrial_calibration_gui_HEADERS})
qt5_wrap_ui(industrial_calibration_gui_UIS_H ${industrial_calibration_gui_UI})

add_library(${PROJECT_NAME}
  ${industrial_calibration_gui_UIS_H}
  ${industrial_calibration_gui_MOCS}
  ${industrial_calibration_gui_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  Qt5::Widgets
  Qt5::Gui
  yaml-cpp
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(${PROJECT_NAME} ${COMPILER_WARNINGS})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
