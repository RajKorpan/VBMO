cmake_minimum_required(VERSION 3.16)
project(VBEA)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

add_executable(evol VBEA.cpp)

add_subdirectory(inc)

target_link_libraries(evol PUBLIC inc)

target_include_directories(evol PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/inc"
                          )
