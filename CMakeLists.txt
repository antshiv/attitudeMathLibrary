cmake_minimum_required(VERSION 3.10)
project(attitude_library C)

set(CMAKE_C_STANDARD 99)

# Add include path
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library sources
set(SOURCES
    src/euler.c
    src/dcm.c
    src/quaternion.c
    src/vector3.c
    src/attitude_utils.c
)

add_library(attitude ${SOURCES})
target_include_directories(attitude PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(attitude PRIVATE m)

add_executable(test_attitude tests/test_attitude.c)
target_link_libraries(test_attitude PRIVATE attitude m)

add_executable(test_attitude_degrees tests/test_attitude_degrees.c)
target_link_libraries(test_attitude_degrees PRIVATE attitude m)
add_test(NAME test_attitude_degrees COMMAND test_attitude_degrees)

add_executable(test_euler tests/test_euler.c)
target_link_libraries(test_euler PRIVATE attitude m)

add_executable(test_dcm_orthogonal tests/test_dcm_orthogonal.c)
target_link_libraries(test_dcm_orthogonal PRIVATE attitude m)

add_executable(test_quaternion_inverse_axis tests/test_quaternion_inverse_axis.c)
target_link_libraries(test_quaternion_inverse_axis PRIVATE attitude m)

add_executable(test_euler_random tests/test_euler_random.c)
target_link_libraries(test_euler_random PRIVATE attitude m)

add_executable(test_rotation tests/test_rotation.c)
target_link_libraries(test_rotation PRIVATE attitude m)

add_executable(test_quaternion tests/test_quaternion.c)
target_link_libraries(test_quaternion PRIVATE attitude m)

# Example test (optional at this stage)
enable_testing()
target_link_libraries(test_attitude PRIVATE attitude)
add_test(NAME test_attitude COMMAND test_attitude)

