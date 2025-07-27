cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

add_definitions(-std=c++20)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(playback)

# # FetchContent module
# include(FetchContent)

# # Declare range-v3
# FetchContent_Declare(
#   range-v3
#   GIT_REPOSITORY https://github.com/ericniebler/range-v3.git
#   GIT_TAG        0.12.0 
# )

# # Make available
# FetchContent_MakeAvailable(range-v3)


find_package(PCL 1.15 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
target_include_directories(environment PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries (environment ${PCL_LIBRARIES})

# add_subdirectory(src/quiz/ransac)
# add_subdirectory(src/quiz/cluster)



