set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os")


add_library(libs_vl53l8cx STATIC
    src/vl53l8cx.cpp
    src/platform.c
    src/vl53l8cx_buffers.h
    src/vl53l8cx_api.c
    src/vl53l8cx_plugin_detection_thresholds.c
    src/vl53l8cx_plugin_motion_indicator.c
    src/vl53l8cx_plugin_xtalk.c
)

target_include_directories(libs_vl53l8cx PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(libs_vl53l8cx PUBLIC
    libs_base-m7_freertos
)

# Ensure that the VL53L8CX library inherits include directories from libs_base-m7_freertos
get_target_property(base_includes libs_base-m7_freertos INTERFACE_INCLUDE_DIRECTORIES)
if(base_includes)
    target_include_directories(libs_vl53l8cx PUBLIC ${base_includes})
endif()

# If needed, you can add additional include directories specific to VL53L8CX
target_include_directories(libs_vl53l8cx PUBLIC
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176
    ${PROJECT_SOURCE_DIR}/third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers
)

bundle_static_library(libs_vl53l8cx libs_vl53l8cx_bundled)