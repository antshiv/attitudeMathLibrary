#include <math.h>
#include <stdio.h>

#include "attitude/quaternion.h"

int main(void) {
    const double half_angle = M_PI / 4.0;
    const double q[4] = {cos(half_angle), 0.0, 0.0, sin(half_angle)};
    const double v_in[3] = {1.0, 0.0, 0.0};
    double v_out[3];

    printf("Demonstrating quaternion_rotate_vector_explicit with debug tracing:\n");
    quaternion_set_explicit_debug(1);
    quaternion_rotate_vector_explicit(q, v_in, v_out);
    quaternion_set_explicit_debug(0);

    printf("Final rotated vector (should align with y-axis): [%.6f, %.6f, %.6f]\n",
           v_out[0], v_out[1], v_out[2]);

    const double tol = 1e-9;
    if (fabs(v_out[0]) > tol || fabs(v_out[1] - 1.0) > tol || fabs(v_out[2]) > tol) {
        printf("Result deviates from expectation.\n");
        return 1;
    }

    return 0;
}
