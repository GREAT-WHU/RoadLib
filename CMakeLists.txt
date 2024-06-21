cmake_minimum_required(VERSION 2.8.3)
project(roadlib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wno-unused-result")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -Wno-unused-result -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Add OpenCV if available
# set(OpenCV_DIR "/home/zivlin/opt/opencv/build")
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
if(Boost_FOUND)
	INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
	LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
endif()

# find_package(Ceres REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${livox_ros_driver_INCLUDE_DIRS}
  ./
  ./gv_tools
  ./roadlib
)
file(GLOB_RECURSE SRC_FILE_LIST "roadlib/*.cpp" "gv_tools/*.cpp" "gv_tools/*.cc" "gv_tools/*.c" "camodocal/*.cpp" "camodocal/*.cc" "camodocal/*.c")

add_executable(demo_mapping demo/demo_mapping.cpp demo/main_phase_mapping.cpp ${SRC_FILE_LIST})
target_link_libraries(demo_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OPENGL_LIBRARIES} glfw)