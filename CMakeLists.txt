cmake_minimum_required(VERSION 3.10)
project(ultimate_alpr)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(src/helper)
include_directories(src/platerecognizer)
include_directories(src/core)
include_directories(src/cardetection)

file(GLOB_RECURSE SRC_FILES "src/*.cpp" "src/*.h")

add_executable(ultimate_alpr ${SRC_FILES})

target_link_libraries(ultimate_alpr ${OpenCV_LIBS})
