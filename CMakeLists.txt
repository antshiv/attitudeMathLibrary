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

# Shared ABI used by independent reference implementations such as SciPy.
add_library(attitude_shared SHARED ${SOURCES})
set_target_properties(attitude_shared PROPERTIES OUTPUT_NAME attitude)
target_include_directories(attitude_shared PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(attitude_shared PRIVATE m)

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

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    set(
        ATTITUDE_REFERENCE_PYTHON
        "${Python3_EXECUTABLE}"
        CACHE FILEPATH
        "Python interpreter containing compatible NumPy and SciPy packages"
    )
    add_test(
        NAME test_scipy_quaternion_parity
        COMMAND ${ATTITUDE_REFERENCE_PYTHON}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/reference/test_scipy_quaternion_parity.py
                --library $<TARGET_FILE:attitude_shared>
    )
    set_tests_properties(test_scipy_quaternion_parity PROPERTIES SKIP_RETURN_CODE 77)
endif()
