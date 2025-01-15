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

# Create the library
add_library(attitude ${SOURCES})
target_include_directories(attitude PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(attitude PRIVATE m)

# Enable testing
enable_testing()

# Automatically find and build all test files
file(GLOB TEST_SOURCES "tests/test_*.c")
foreach(TEST_SOURCE ${TEST_SOURCES})
    # Get the filename without extension
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    
    # Create executable
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    
    # Link with attitude library and math library
    target_link_libraries(${TEST_NAME} PRIVATE attitude m)
    
    # Add as test
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()