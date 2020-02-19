cmake_minimum_required(VERSION 3.15)
project(ConstructTemplate)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
list (REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ConstructTemplate main.cpp)
target_link_libraries(ConstructTemplate ${PCL_LIBRARIES})