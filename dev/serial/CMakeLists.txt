cmake_minimum_required(VERSION 3.4)

# https://stackoverflow.com/questions/17511496/how-to-create-a-shared-library-with-cmake
# https://stackoverflow.com/questions/51451124/python-2-7-c-extensions-with-cmake-and-setuptools

project(test VERSION 1.0.0 DESCRIPTION "test")
set(CMAKE_CXX_STANDARD 14)
set(DEFAULT_BUILD_TYPE "Release")

# Threads
find_package(Threads REQUIRED)

# or STATIC
add_executable(${PROJECT_NAME} main.cpp imp.cpp lidar.cpp)
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
# set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
# install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
