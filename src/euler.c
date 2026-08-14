#include "attitude/euler.h"
#include <math.h>
#include <stddef.h>

static void set_nan_matrix(double dcm[3][3]) {
    if (dcm == NULL) {
        return;
    }
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            dcm[row][column] = NAN;
        }
    }
}

int euler_to_dcm_checked(const EulerAngles *e, double dcm[3][3]) {
    if (e == NULL || dcm == NULL || e->order != EULER_ZYX ||
        !isfinite(e->roll) || !isfinite(e->pitch) || !isfinite(e->yaw)) {
        return 0;
    }

    double cr = cos(e->roll);
    double sr = sin(e->roll);
    double cp = cos(e->pitch);
    double sp = sin(e->pitch);
    double cy = cos(e->yaw);
    double sy = sin(e->yaw);

    dcm[0][0] = cy*cp;
    dcm[0][1] = cy*sp*sr - sy*cr;
    dcm[0][2] = cy*sp*cr + sy*sr;
    dcm[1][0] = sy*cp;
    dcm[1][1] = sy*sp*sr + cy*cr;
    dcm[1][2] = sy*sp*cr - cy*sr;
    dcm[2][0] = -sp;
    dcm[2][1] = cp*sr;
    dcm[2][2] = cp*cr;
    return 1;
}

void euler_to_dcm(const EulerAngles *e, double dcm[3][3]) {
    if (!euler_to_dcm_checked(e, dcm)) {
        set_nan_matrix(dcm);
    }
}

int euler_to_quaternion_checked(const EulerAngles *e, double q[4]) {
    if (e == NULL || q == NULL || e->order != EULER_ZYX ||
        !isfinite(e->roll) || !isfinite(e->pitch) || !isfinite(e->yaw)) {
        return 0;
    }

    double cr = cos(e->roll/2.0);
    double sr = sin(e->roll/2.0);
    double cp = cos(e->pitch/2.0);
    double sp = sin(e->pitch/2.0);
    double cy = cos(e->yaw/2.0);
    double sy = sin(e->yaw/2.0);

    q[0] = cr*cp*cy + sr*sp*sy; // w
    q[1] = sr*cp*cy - cr*sp*sy; // x
    q[2] = cr*sp*cy + sr*cp*sy; // y
    q[3] = cr*cp*sy - sr*sp*cy; // z
    return 1;
}

void euler_to_quaternion(const EulerAngles *e, double q[4]) {
    if (!euler_to_quaternion_checked(e, q) && q != NULL) {
        for (int index = 0; index < 4; ++index) {
            q[index] = NAN;
        }
    }
}
