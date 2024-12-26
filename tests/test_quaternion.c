#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

int compare_doubles(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

void test_identity_quaternion() {
    printf("\nTest 1: Identity Quaternion\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;
    const double q_identity[4] = {1, 0, 0, 0};
    
    quaternion_to_euler(q_identity, &roll, &pitch, &yaw);
    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, 0.0, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));
}

void test_pure_pitch_rotations() {
    printf("\nTest 2: Pure Pitch Rotations\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;

    // 45-degree pitch
    const double q_pitch_45[4] = {
        cos(M_PI/8),    // w
        0,              // x
        sin(M_PI/8),    // y
        0               // z
    };
    quaternion_to_euler(q_pitch_45, &roll, &pitch, &yaw);
    printf("45° Pitch - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, M_PI/4, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));

    // 90-degree pitch (gimbal lock case)
    const double q_pitch_90[4] = {
        cos(M_PI/4),    // w
        0,              // x
        sin(M_PI/4),    // y
        0               // z
    };
    quaternion_to_euler(q_pitch_90, &roll, &pitch, &yaw);
    printf("90° Pitch - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, M_PI/2, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));

    // -45-degree pitch
    const double q_pitch_neg45[4] = {
        cos(-M_PI/8),   // w
        0,              // x
        sin(-M_PI/8),   // y
        0               // z
    };
    quaternion_to_euler(q_pitch_neg45, &roll, &pitch, &yaw);
    printf("-45° Pitch - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, -M_PI/4, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));
}

void test_pure_roll_rotations() {
    printf("\nTest 3: Pure Roll Rotations\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;

    // 45-degree roll
    const double q_roll_45[4] = {
        cos(M_PI/8),    // w
        sin(M_PI/8),    // x
        0,              // y
        0               // z
    };
    quaternion_to_euler(q_roll_45, &roll, &pitch, &yaw);
    printf("45° Roll - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, M_PI/4, TOLERANCE));
    assert(compare_doubles(pitch, 0.0, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));

    // 180-degree roll
    const double q_roll_180[4] = {0, 1, 0, 0};
    quaternion_to_euler(q_roll_180, &roll, &pitch, &yaw);
    printf("180° Roll - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(fabs(roll), M_PI, TOLERANCE));
    assert(compare_doubles(pitch, 0.0, TOLERANCE));
    assert(compare_doubles(yaw, 0.0, TOLERANCE));
}

void test_pure_yaw_rotations() {
    printf("\nTest 4: Pure Yaw Rotations\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;

    // 45-degree yaw
    const double q_yaw_45[4] = {
        cos(M_PI/8),    // w
        0,              // x
        0,              // y
        sin(M_PI/8)     // z
    };
    quaternion_to_euler(q_yaw_45, &roll, &pitch, &yaw);
    printf("45° Yaw - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, 0.0, TOLERANCE));
    assert(compare_doubles(yaw, M_PI/4, TOLERANCE));

    // -90-degree yaw
    const double q_yaw_neg90[4] = {
        cos(-M_PI/4),   // w
        0,              // x
        0,              // y
        sin(-M_PI/4)    // z
    };
    quaternion_to_euler(q_yaw_neg90, &roll, &pitch, &yaw);
    printf("-90° Yaw - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", roll, pitch, yaw);
    assert(compare_doubles(roll, 0.0, TOLERANCE));
    assert(compare_doubles(pitch, 0.0, TOLERANCE));
    assert(compare_doubles(yaw, -M_PI/2, TOLERANCE));
}

void test_combined_rotations() {
    printf("\nTest 5: Combined Rotations\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;

    // Method 1: Create combined rotation through quaternion multiplication
    double q1[4], q2[4], q_combined[4];
    
    // First rotation: 45° roll
    q1[0] = cos(M_PI/8);  // w
    q1[1] = sin(M_PI/8);  // x
    q1[2] = 0;            // y
    q1[3] = 0;            // z
    
    // Second rotation: 45° pitch
    q2[0] = cos(M_PI/8);  // w
    q2[1] = 0;            // x
    q2[2] = sin(M_PI/8);  // y
    q2[3] = 0;            // z

    // Multiply quaternions to get combined rotation
    quaternion_multiply(q2, q1, q_combined);
    quaternion_normalize(q_combined);
    
    quaternion_to_euler(q_combined, &roll, &pitch, &yaw);
    printf("45° Roll then 45° Pitch - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
           roll, pitch, yaw);
    printf("In degrees - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
           roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);

    // We're getting approximately 45° (0.785398 rad) for both roll and pitch
    assert(compare_doubles(roll, M_PI/4, TOLERANCE));     // 45 degrees = pi/4
    assert(compare_doubles(pitch, M_PI/4, TOLERANCE));    // 45 degrees = pi/4
    assert(compare_doubles(yaw, 0.0, TOLERANCE));         // No yaw rotation

    // Test a different combination: 30° roll followed by 30° pitch
    double q3[4], q4[4], q_combined2[4];
    
    // 30° roll
    q3[0] = cos(M_PI/12);  // w
    q3[1] = sin(M_PI/12);  // x
    q3[2] = 0;             // y
    q3[3] = 0;             // z
    
    // 30° pitch
    q4[0] = cos(M_PI/12);  // w
    q4[1] = 0;             // x
    q4[2] = sin(M_PI/12);  // y
    q4[3] = 0;             // z

    quaternion_multiply(q4, q3, q_combined2);
    quaternion_normalize(q_combined2);
    
    quaternion_to_euler(q_combined2, &roll, &pitch, &yaw);
    printf("\n30° Roll then 30° Pitch - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
           roll, pitch, yaw);
    printf("In degrees - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
           roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);

    // For 30° rotations, the coupling effect is less pronounced
    assert(compare_doubles(roll, M_PI/6, 0.1));      // Approximately 30 degrees
    assert(compare_doubles(pitch, M_PI/6, 0.1));     // Approximately 30 degrees
    assert(compare_doubles(yaw, 0.0, 0.1));          // Small or no yaw
}

void test_gimbal_lock_cases() {
    printf("\nTest 6: Gimbal Lock Cases\n");
    const double TOLERANCE = 1e-6;
    double roll, pitch, yaw;

    // Pitch at +90 degrees (positive gimbal lock)
    const double q_gimbal_pos[4] = {
        cos(M_PI/4),    // w
        0,              // x
        sin(M_PI/4),    // y
        0               // z
    };
    quaternion_to_euler(q_gimbal_pos, &roll, &pitch, &yaw);
    printf("Positive Gimbal Lock - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
           roll, pitch, yaw);
    assert(compare_doubles(pitch, M_PI/2, TOLERANCE));

    // Pitch at -90 degrees (negative gimbal lock)
    const double q_gimbal_neg[4] = {
        cos(-M_PI/4),   // w
        0,              // x
        sin(-M_PI/4),   // y
        0               // z
    };
    quaternion_to_euler(q_gimbal_neg, &roll, &pitch, &yaw);
    printf("Negative Gimbal Lock - Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
           roll, pitch, yaw);
    assert(compare_doubles(pitch, -M_PI/2, TOLERANCE));
}

int main() {
    printf("Starting Quaternion to Euler Tests...\n");
    
    test_identity_quaternion();
    test_pure_pitch_rotations();
    test_pure_roll_rotations();
    test_pure_yaw_rotations();
    test_combined_rotations();
    test_gimbal_lock_cases();
    
    printf("\nAll tests completed successfully!\n");
    return 0;
}
