project(PathfinderTest)
cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_STANDARD 11)

# Make sure conan FindXYZ.cmake files can be found
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_MODULE_PATH})

# Find libraries
find_package(SDL2 REQUIRED)
# Conan names use a different case
set(SDL2_LIBRARIES ${SDL2_LIBRARIES} ${sdl2_LIBRARIES})
set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS} ${sdl2_INCLUDE_DIRS})

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/pathfinder/include)

file(GLOB_RECURSE PF_SOURCES "pathfinder/src/*.c" "pathfinder/include/*.h")
add_library(PathfinderCustom STATIC ${PF_SOURCES})

file(GLOB_RECURSE SOURCES "src/*.cpp" "include/*.h" "include/*.hpp")
add_executable(PathfinderTest ${SOURCES})
target_link_libraries(PathfinderTest PathfinderCustom ${SDL2_LIBRARIES})
