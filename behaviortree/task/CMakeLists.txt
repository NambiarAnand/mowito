cmake_minimum_required(VERSION 3.10)
project(task)

set(CMAKE_CXX_STANDARD 17)

find_package(behaviortree_cpp REQUIRED)

add_executable(task bt.cpp)
target_link_libraries(task behaviortree_cpp)
