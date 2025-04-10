cmake_minimum_required(VERSION 3.15)
project(flatness_probe_simulation)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json (for IntelliSense, auto-complete, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Path to your custom OpenCascade build
set(CMAKE_PREFIX_PATH "$ENV{HOME}/occt-install")

# Include + link OpenCascade
include_directories("${OCCT_DIR}/include/opencascade")
link_directories("${OCCT_DIR}/lib")

find_package(OpenCASCADE REQUIRED)
include_directories(${OpenCASCADE_INCLUDE_DIR})

add_executable(flatness_probe_simulation main.cpp)
target_link_libraries(flatness_probe_simulation ${OpenCASCADE_LIBRARIES})
