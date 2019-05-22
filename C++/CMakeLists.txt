cmake_minimum_required(VERSION 3.13)
project(main.cpp)

set(CMAKE_CXX_STANDARD)

find_package(OpenCV REQUIRED)

add_executable(main main.cpp defs.h camera.cpp camera.h defs.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(main ${OpenCV_LIBS} ueye_api)