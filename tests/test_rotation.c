#include <stdio.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/attitude_utils.h"

void test_attitude_rotation() {
    // Initial and target Euler angles
    EulerAngles start = {deg2rad(30), deg2rad(20), deg2rad(0), EULER_ZYX};
    EulerAngles target = {deg2rad(0), deg2rad(0), deg2rad(0), EULER_ZYX};

    // Convert to quaternions
    double q_start[4], q_target[4];
    euler_to_quaternion(&start, q_start);
    euler_to_quaternion(&target, q_target);

    // Interpolation parameters
    double q_current[4];
    double step = 0.1; // Step size (fraction of the total transition per iteration)
    int steps = (int)(1.0 / step);

    printf("Time(s) | Roll(deg) | Pitch(deg) | Yaw(deg) | Quaternion (w, x, y, z)\n");
    printf("-------------------------------------------------------------------\n");

    for (int i = 0; i <= steps; ++i) {
        double t = i * step; // Fractional time
        quaternion_slerp(q_start, q_target, t, q_current);

        // Convert current quaternion back to Euler angles
        EulerAngles current_euler;
        quaternion_to_euler(q_current, &current_euler.roll, &current_euler.pitch, &current_euler.yaw);

        printf("%6.2f | %9.2f | %10.2f | %7.2f | (%5.3f, %5.3f, %5.3f, %5.3f)\n",
               t,
               rad2deg(current_euler.roll),
               rad2deg(current_euler.pitch),
               rad2deg(current_euler.yaw),
               q_current[0], q_current[1], q_current[2], q_current[3]);
    }
}

int main() {
    test_attitude_rotation();
    return 0;
}

