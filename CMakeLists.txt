project(PathfinderTest)
cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_STANDARD 11)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/pathfinder/include)

file(GLOB_RECURSE PF_SOURCES "pathfinder/src/*.c" "pathfinder/include/*.h")
add_library(PathfinderCustom STATIC ${PF_SOURCES})

file(GLOB_RECURSE SOURCES "src/*.cpp" "include/*.h" "include/*.hpp")
add_executable(PathfinderTest ${SOURCES})
target_link_libraries(PathfinderTest PathfinderCustom CONAN_PKG::sdl2)
