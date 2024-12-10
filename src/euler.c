#include "attitude/euler.h"
#include <math.h>

void euler_to_dcm(const EulerAngles *e, double dcm[3][3]) {
    double cr = cos(e->roll);
    double sr = sin(e->roll);
    double cp = cos(e->pitch);
    double sp = sin(e->pitch);
    double cy = cos(e->yaw);
    double sy = sin(e->yaw);

    switch (e->order) {
        case EULER_ZYX:
            // Z-Y-X convention
            dcm[0][0] = cy*cp;
            dcm[0][1] = cy*sp*sr - sy*cr;
            dcm[0][2] = cy*sp*cr + sy*sr;
            dcm[1][0] = sy*cp;
            dcm[1][1] = sy*sp*sr + cy*cr;
            dcm[1][2] = sy*sp*cr - cy*sr;
            dcm[2][0] = -sp;
            dcm[2][1] = cp*sr;
            dcm[2][2] = cp*cr;
            break;

        // Add other conventions similarly
        default:
            // If not implemented, print warning or use a default
            break;
    }
}

void euler_to_quaternion(const EulerAngles *e, double q[4]) {
    // Placeholder: Proper quaternion from Euler angles
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
}

