cmake_minimum_required(VERSION 3.16)
project(position_gimbal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(serial)

add_library(gimbal STATIC gimbal.cpp gimbal.hpp)
target_include_directories(gimbal PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gimbal PUBLIC serial)

if(UNIX)
  target_link_libraries(gimbal PUBLIC pthread)
endif()
