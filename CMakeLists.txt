cmake_minimum_required(VERSION 3.10)
project(dual-depth-mesh LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/.local")

find_package(realsense2 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(rs-multicam-mesh src/rs-multicam-mesh.cpp)
target_include_directories(rs-multicam-mesh PRIVATE src third_party)
target_link_libraries(rs-multicam-mesh PRIVATE realsense2::realsense2 glfw OpenGL::GL OpenGL::GLU)
if(NOT WIN32)
    target_link_libraries(rs-multicam-mesh PRIVATE dl)
endif()
