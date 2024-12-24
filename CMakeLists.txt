cmake_minimum_required(VERSION 3.20)
project(PSXEmulator VERSION 1.0.0)

add_subdirectory(deps/SDL2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCE_FILES 
    "src/*.cpp"
    "src/*.hpp"
    "src/*.h"
)

add_executable(PSXEmulator ${SOURCE_FILES})

target_include_directories(PSXEmulator PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(PSXEmulator PRIVATE SDL2)
