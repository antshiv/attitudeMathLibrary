#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

/* Helper function to check if two values are approximately equal */
int compare_doubles(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

/* Helper function to check if two quaternions are approximately equal */
void assert_quaternion_equal(const double q1[4], const double q2[4], double tolerance) {
    for (int i = 0; i < 4; i++) {
        assert(compare_doubles(q1[i], q2[i], tolerance));
    }
}

/* Helper function to verify quaternion normalization */
void assert_quaternion_normalized(const double q[4], double tolerance) {
    double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    assert(compare_doubles(1.0, norm, tolerance));
}

void test_slerp_boundary_conditions() {
    printf("\nTest 1: SLERP Boundary Conditions\n");
    const double TOLERANCE = 1e-6;
    
    // Test quaternions
    const double q1[4] = {1.0, 0.0, 0.0, 0.0};  // Identity quaternion
    const double q2[4] = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};  // 90-degree X rotation
    double result[4];
    
    // Test t = 0 (should return q1)
    quaternion_slerp(q1, q2, 0.0, result);
    printf("t=0 result: [%.6f, %.6f, %.6f, %.6f]\n", 
           result[0], result[1], result[2], result[3]);
    assert_quaternion_equal(q1, result, TOLERANCE);
    
    // Test t = 1 (should return q2)
    quaternion_slerp(q1, q2, 1.0, result);
    printf("t=1 result: [%.6f, %.6f, %.6f, %.6f]\n", 
           result[0], result[1], result[2], result[3]);
    assert_quaternion_equal(q2, result, TOLERANCE);
}

void test_slerp_midpoint() {
    printf("\nTest 2: SLERP Midpoint\n");
    const double TOLERANCE = 1e-6;
    
    // Test 180-degree rotation around X axis
    const double q1[4] = {1.0, 0.0, 0.0, 0.0};
    const double q2[4] = {0.0, 1.0, 0.0, 0.0};
    const double expected[4] = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};  // 90-degree rotation
    double result[4];
    
    quaternion_slerp(q1, q2, 0.5, result);
    printf("Midpoint result: [%.6f, %.6f, %.6f, %.6f]\n", 
           result[0], result[1], result[2], result[3]);
    assert_quaternion_equal(expected, result, TOLERANCE);
}

void test_slerp_normalization() {
    printf("\nTest 3: SLERP Maintains Normalization\n");
    const double TOLERANCE = 1e-6;
    
    const double q1[4] = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};
    const double q2[4] = {0.0, 0.0, 0.7071067811865476, 0.7071067811865476};
    double result[4];
    
    // Test several interpolation points
    double t_values[] = {0.0, 0.25, 0.5, 0.75, 1.0};
    for (int i = 0; i < 5; i++) {
        quaternion_slerp(q1, q2, t_values[i], result);
        printf("t=%.2f result: [%.6f, %.6f, %.6f, %.6f]\n", 
               t_values[i], result[0], result[1], result[2], result[3]);
        assert_quaternion_normalized(result, TOLERANCE);
    }
}

void test_slerp_shortest_path() {
    printf("\nTest 4: SLERP Shortest Path\n");
    const double TOLERANCE = 1e-6;
    
    // Test that SLERP takes shortest path when quaternions represent same rotation
    const double q1[4] = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};
    const double q2[4] = {-0.7071067811865476, -0.7071067811865476, 0.0, 0.0};  // Same rotation as q1
    double result[4];
    
    quaternion_slerp(q1, q2, 0.5, result);
    printf("Shortest path result: [%.6f, %.6f, %.6f, %.6f]\n", 
           result[0], result[1], result[2], result[3]);
    
    // Should interpolate along shortest path, resulting in approximately q1
    double dot = fabs(result[0]*q1[0] + result[1]*q1[1] + result[2]*q1[2] + result[3]*q1[3]);
    assert(compare_doubles(1.0, dot, TOLERANCE));
}

void test_slerp_identical_quaternions() {
    printf("\nTest 5: SLERP with Identical Quaternions\n");
    const double TOLERANCE = 1e-6;
    
    const double q1[4] = {0.7071067811865476, 0.7071067811865476, 0.0, 0.0};
    double result[4];
    
    quaternion_slerp(q1, q1, 0.5, result);
    printf("Same quaternion result: [%.6f, %.6f, %.6f, %.6f]\n", 
           result[0], result[1], result[2], result[3]);
    assert_quaternion_equal(q1, result, TOLERANCE);
}

int main() {
    printf("Starting Quaternion SLERP Tests...\n");
    
    test_slerp_boundary_conditions();
    test_slerp_midpoint();
    test_slerp_normalization();
    test_slerp_shortest_path();
    test_slerp_identical_quaternions();
    
    printf("\nAll tests completed successfully!\n");
    return 0;
}