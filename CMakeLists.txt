cmake_minimum_required(VERSION 3.5)
project(fit_tracking)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(east_gate main_east_west.cpp codebook.cpp)
target_compile_definitions(east_gate PRIVATE CC_SCALE=3600) # "scale" parameter for ConnectedComponents
target_link_libraries(east_gate ${OpenCV_LIBS})

add_executable(west_gate main_east_west.cpp codebook.cpp)
target_compile_definitions(west_gate PRIVATE CC_SCALE=200) # "scale" parameter for ConnectedComponents
target_link_libraries(west_gate ${OpenCV_LIBS})

add_executable(lift main_lift.cpp)
target_link_libraries(lift ${OpenCV_LIBS})