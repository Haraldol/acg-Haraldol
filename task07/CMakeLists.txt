# specify the version of cmake (intentionally using old version for those who cannot update CMake)
cmake_minimum_required(VERSION 3.10)

#############################
# set C++ detail
enable_language(CXX)  # we are using C++
set(CMAKE_CXX_STANDARD 17)  # we are using C++ 17
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)  # we are using STL library



#############################
# set project name

project(task07)

#############################
# define macro
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

#############################
# specifying libraries to use

########################
# include, build, and link

include_directories(
    ${PROJECT_SOURCE_DIR}/../external
    ${PROJECT_SOURCE_DIR}/../external/eigen
)

add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
)

message(STATUS "Current build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ Flags: ${CMAKE_CXX_FLAGS}")