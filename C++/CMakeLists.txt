cmake_minimum_required(VERSION 3.13)

project(main.cpp)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find OpenCV
set(OpenCV_DIR /home/imarcher/installed/OpenCV4/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Find ueye
set(UEYE_LIBRARY_PATH /usr/lib/)
find_library(UEYE_LIBRARY ueye_api)
set(UEYE_INCLUDE_DIR /usr/include/)
message(STATUS "ueye library status:")
message(STATUS "    libraries: ${UEYE_LIBRARY}")

include_directories(${OpenCV_INCLUDE_DIRS} ${UEYE_INCLUDE_DIR})

add_executable(main main.cpp defs.h camera.cpp camera.h defs.cpp estimator.cpp estimator.h image.cpp image.h)
target_link_libraries(main ${OpenCV_LIBS} ${UEYE_LIBRARY})