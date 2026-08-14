cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(attitude_validation)

target_include_directories(app PRIVATE ../../include)
target_sources(app PRIVATE
    src/main.c
    ../../src/euler.c
    ../../src/dcm.c
    ../../src/quaternion.c
    ../../src/vector3.c
    ../../src/attitude_utils.c
    ../../src/validation.c
)
