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

# Find Ceres
find_package(Ceres REQUIRED)
message(STATUS "ceres library status:")
message(STATUS "    libraries: ${CERES_LIBRARIES}")

# Find LpSensor
find_library(LPSENSOR_LIBRARY LpSensor)

include_directories(${OpenCV_INCLUDE_DIRS} ${UEYE_INCLUDE_DIR} ${CERES_INCLUDE_DIRS})
add_executable(main main.cpp defs.h camera.cpp camera.h rotation.cpp rotation.h image.cpp image.h sensor.cpp sensor.h defs.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${UEYE_LIBRARY} ${CERES_LIBRARIES} ${LPSENSOR_LIBRARY} pthread dl rt)
