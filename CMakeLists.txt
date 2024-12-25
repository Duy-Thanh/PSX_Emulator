# Force MinGW generator
set(CMAKE_GENERATOR "MinGW Makefiles" CACHE INTERNAL "" FORCE)

cmake_minimum_required(VERSION 3.20)

# Set policy for timestamp extraction
cmake_policy(SET CMP0135 NEW)

include(FetchContent)

# Function to download and setup MinGW
function(setup_mingw)
    set(MINGW_URL "https://github.com/brechtsanders/winlibs_mingw/releases/download/14.2.0posix-19.1.1-12.0.0-ucrt-r2/winlibs-x86_64-posix-seh-gcc-14.2.0-mingw-w64ucrt-12.0.0-r2.zip")
    set(MINGW_DIR "${CMAKE_BINARY_DIR}/mingw")
    set(MINGW_ZIP "${CMAKE_BINARY_DIR}/mingw.zip")

    if(NOT EXISTS ${MINGW_DIR})
        message(STATUS "Downloading MinGW...")
        file(DOWNLOAD ${MINGW_URL} ${MINGW_ZIP})
        
        message(STATUS "Extracting MinGW...")
        file(ARCHIVE_EXTRACT INPUT ${MINGW_ZIP} DESTINATION ${MINGW_DIR})
        
        # Debug output to check the extracted structure
        message(STATUS "Checking MinGW directory content:")
        file(GLOB_RECURSE MINGW_FILES "${MINGW_DIR}/*g++.exe")
        foreach(F IN LISTS MINGW_FILES)
            message(STATUS "Found g++.exe at: ${F}")
        endforeach()
        
        file(REMOVE ${MINGW_ZIP})
    endif()

    # Set make program first
    set(CMAKE_MAKE_PROGRAM "${MINGW_DIR}/mingw64/bin/mingw32-make.exe" CACHE STRING "" FORCE)
    
    # Then set the compilers
    set(CMAKE_C_COMPILER "${MINGW_DIR}/mingw64/bin/gcc.exe" CACHE STRING "" FORCE)
    set(CMAKE_CXX_COMPILER "${MINGW_DIR}/mingw64/bin/g++.exe" CACHE STRING "" FORCE)
    set(CMAKE_RC_COMPILER "${MINGW_DIR}/mingw64/bin/windres.exe" CACHE STRING "" FORCE)

    # Add MinGW bin directory to PATH
    set(ENV{PATH} "${MINGW_DIR}/mingw64/bin;$ENV{PATH}")
endfunction()

# Setup MinGW before project declaration
setup_mingw()

project(PSXEmulator VERSION 1.0.0)

# Set make program first
set(CMAKE_MAKE_PROGRAM "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/mingw32-make.exe" CACHE STRING "" FORCE)

# Then set the compilers
set(CMAKE_C_COMPILER "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/gcc.exe" CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/g++.exe" CACHE STRING "" FORCE)
set(CMAKE_RC_COMPILER "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/windres.exe" CACHE STRING "" FORCE)

# Make sure RC compiler is set after project() as well
set(CMAKE_RC_COMPILER "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/windres.exe" CACHE STRING "" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCE_FILES 
    "src/*.cpp"
    "src/*.hpp"
    "src/*.h"
)

add_executable(PSXEmulator ${SOURCE_FILES})

target_include_directories(PSXEmulator PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Function to copy DLLs - Move this BEFORE we call it
function(copy_runtime_dlls)
    # MinGW DLLs
    set(MINGW_DLLS
        "libgcc_s_seh-1.dll"
        "libstdc++-6.dll"
        "libwinpthread-1.dll"
    )
    
    # Check and copy MinGW DLLs
    foreach(DLL ${MINGW_DLLS})
        set(DLL_PATH "${CMAKE_BINARY_DIR}/mingw/mingw64/bin/${DLL}")
        if(NOT EXISTS "${DLL_PATH}")
            message(WARNING "MinGW DLL not found: ${DLL_PATH}")
            continue()
        endif()
        file(COPY "${DLL_PATH}" DESTINATION "${CMAKE_BINARY_DIR}")
    endforeach()

    # Check and copy SDL2.dll
    set(SDL2_DLL "${CMAKE_BINARY_DIR}/deps/SDL2/SDL2.dll")
    if(NOT EXISTS "${SDL2_DLL}")
        message(WARNING "SDL2.dll not found: ${SDL2_DLL}")
    else()
        file(COPY "${SDL2_DLL}" DESTINATION "${CMAKE_BINARY_DIR}")
    endif()
         
    message(STATUS "DLL copy operation completed")
endfunction()

# Then use it after SDL2 setup
add_subdirectory(deps/SDL2)
target_link_libraries(PSXEmulator PRIVATE SDL2)
copy_runtime_dlls()
