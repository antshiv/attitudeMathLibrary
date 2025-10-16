#include <math.h>
#include <stdio.h>

#include "attitude/quaternion.h"

static int nearly_equal(double a, double b, double tol) {
    return fabs(a - b) < tol;
}

int main(void) {
    const double tol = 1e-9;
    /* 90 degree rotation about Z axis */
    const double angle_half = M_PI / 4.0;
    const double q[4] = {
        cos(angle_half), /* w */
        0.0,
        0.0,
        sin(angle_half)  /* z */
    };

    const double v_in[3] = {1.0, 0.0, 0.0};
    double v_out[3];
    quaternion_rotate_vector(q, v_in, v_out);

    if (!nearly_equal(v_out[0], 0.0, tol) ||
        !nearly_equal(v_out[1], 1.0, tol) ||
        !nearly_equal(v_out[2], 0.0, tol)) {
        printf("FAIL: quaternion_rotate_vector produced [%f, %f, %f]\n",
               v_out[0], v_out[1], v_out[2]);
        return 1;
    }

    /* Check explicit form matches optimized form */
    double v_explicit[3];
    quaternion_rotate_vector_explicit(q, v_in, v_explicit);

    if (!nearly_equal(v_out[0], v_explicit[0], tol) ||
        !nearly_equal(v_out[1], v_explicit[1], tol) ||
        !nearly_equal(v_out[2], v_explicit[2], tol)) {
        printf("FAIL: optimized and explicit rotations differ\n");
        return 1;
    }

    printf("PASS: quaternion_rotate_vector tests\n");
    return 0;
}
