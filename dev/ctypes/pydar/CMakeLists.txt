cmake_minimum_required(VERSION 3.4)

# https://stackoverflow.com/questions/17511496/how-to-create-a-shared-library-with-cmake
# https://stackoverflow.com/questions/51451124/python-2-7-c-extensions-with-cmake-and-setuptools

project(pydarlib VERSION 1.0.0 DESCRIPTION "lidar driver")
set(CMAKE_CXX_STANDARD 14)
set(DEFAULT_BUILD_TYPE "Release")

# Threads
find_package(Threads REQUIRED)

# Find Python
find_package(PythonLibs 3.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIR})

# or STATIC
add_library(${PROJECT_NAME} SHARED pydar.cpp)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
# install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
