#include <stdio.h>
#include <math.h>
#include "attitude/quaternion.h"
#include "attitude/vector3.h" // if needed for checking axis length

int main() {
    double q[4] = {0.707, 0.707, 0.0, 0.0}; // 90° rotation about X-axis
    quaternion_normalize(q);

    double q_inv[4];
    if (!quaternion_inverse(q, q_inv)) {
        printf("FAIL: Could not invert quaternion.\n");
        return 1;
    }

    // q * q_inv should be close to [1,0,0,0]
    double q_id[4];
    quaternion_multiply(q, q_inv, q_id);
    if (fabs(q_id[0]-1.0) > 1e-9 || fabs(q_id[1]) > 1e-9 || fabs(q_id[2]) > 1e-9 || fabs(q_id[3]) > 1e-9) {
        printf("FAIL: Quaternion inverse test failed.\n");
        return 1;
    }

    double axis[3], angle;
    quaternion_to_axis_angle(q, axis, &angle);
    double axis_mag = sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (fabs(angle - M_PI/2) > 1e-6 || fabs(axis_mag - 1.0) > 1e-9) {
        printf("FAIL: Axis-angle conversion not as expected.\n");
        return 1;
    }

    printf("PASS: Quaternion inverse and axis-angle tests.\n");
    return 0;
}

