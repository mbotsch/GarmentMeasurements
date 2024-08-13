cmake_minimum_required(VERSION 3.16.3)
project(ShapesForGarments)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set output paths
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/")

# pmp-library
set(PMP_BUILD_EXAMPLES     OFF CACHE BOOL "")
set(PMP_BUILD_TESTS        OFF CACHE BOOL "")
set(PMP_BUILD_DOCS         OFF CACHE BOOL "")
set(PMP_BUILD_VIS          ON  CACHE BOOL "")
set(PMP_INSTALL            OFF CACHE BOOL "")
set(PMP_STRICT_COMPILATION ON  CACHE BOOL "")
set(BUILD_SHARED_LIBS      OFF CACHE BOOL "")

add_subdirectory(${PROJECT_SOURCE_DIR}/external/pmp-library)

include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/src)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/eigen-3.4.0)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/glfw-3.3.8/include)
include_directories(${PROJECT_SOURCE_DIR}/external/pmp-library/external/imgui-1.89.8)

# cmake modules
include(FetchContent)

# fbx sdk
find_package(FBX)

if (NOT FBX_FOUND)
    message("FBX SDK not found")
else()
    include_directories(${FBX_INCLUDE_DIRS})
endif()

# cgal
# set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL REQUIRED)
#target_link_libraries(<Target> CGAL::CGAL)

# nlohmann json
find_package(nlohmann_json 3.11.0)

# argparse
include(cmake/modules/Argparse.cmake)

if (NOT nlohmann_json_FOUND)
    include(FetchContent)

    FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
    FetchContent_MakeAvailable(json)
endif ()

# set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COMMON_CXX_FLAGS "-pedantic -Wall -Wextra -Wshadow")
set(CMAKE_CXX_FLAGS ${COMMON_CXX_FLAGS})

include_directories(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(src)
