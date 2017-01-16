cmake_minimum_required(VERSION 3.5)
project(fit_tracking)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp codebook.cpp)
add_executable(fit_tracking ${SOURCE_FILES})
target_link_libraries(fit_tracking ${OpenCV_LIBS})