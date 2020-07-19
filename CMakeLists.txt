cmake_minimum_required(VERSION 3.16)
project(ros2_experiments)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(spin_some)
add_subdirectory(publisher_without_spin)
add_subdirectory(publishing_overhead)

