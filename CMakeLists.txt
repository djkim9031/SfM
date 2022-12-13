cmake_minimum_required(VERSION 3.19 FATAL_ERROR)
project(SfM)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE  Debug)
endif()
    
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(COLMAP REQUIRED)
find_package(OpenCV 4.6 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(VTK REQUIRED)

file(GLOB SRC_DIR src/*.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/includes ${OpenCV_INCLUDE_DIRS} ${COLMAP_INCLUDE_DIRS})
link_directories(${COLMAP_LINK_DIRS})
add_executable(SfM main.cpp)
target_sources(SfM PRIVATE ${SRC_DIR})
target_link_libraries(SfM ${PCL_LIBRARIES} ${OpenCV_LIBS} ${COLMAP_LIBRARIES})

