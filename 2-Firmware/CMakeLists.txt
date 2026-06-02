cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(kia_arm_ctrl
    VERSION 1.0.0
    LANGUAGES C CXX
)

idf_build_set_property(COMPILE_OPTIONS "-Wall;-Wextra;-Werror;-Wshadow;-Wundef;-Wpointer-arith;-Wcast-align" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-std=gnu11" APPEND)
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-fno-exceptions;-fno-rtti" APPEND)
