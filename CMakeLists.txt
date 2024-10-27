# Require a minimum CMake version
cmake_minimum_required(VERSION 3.16)

# Project name
project(vl53l8cx)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")

# Create the library
add_library(libs_vl53l8cx STATIC
    src/vl53l8cx.cpp
    src/platform.c
    src/vl53l8cx_api.c
    src/vl53l8cx_plugin_detection_thresholds.c
    src/vl53l8cx_plugin_motion_indicator.c
    src/vl53l8cx_plugin_xtalk.c
)

# Add include directories
target_include_directories(libs_${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against the base libraries
target_link_libraries(libs_${PROJECT_NAME}
    PUBLIC
        libs_base-m7_freertos
)

# Bundle the static library if the function exists
if(COMMAND bundle_static_library)
    bundle_static_library(libs_${PROJECT_NAME} libs_${PROJECT_NAME}_bundled)
endif()