#include "attitude/quaternion.h"
#include <math.h>

void quaternion_to_dcm(const double q[4], double dcm[3][3]) {
    // q = [w, x, y, z]
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double xx = x*x; double yy = y*y; double zz = z*z;
    double xy = x*y; double xz = x*z; double yz = y*z;
    double wx = w*x; double wy = w*y; double wz = w*z;

    dcm[0][0] = 1.0 - 2.0*(yy + zz);
    dcm[0][1] = 2.0*(xy - wz);
    dcm[0][2] = 2.0*(xz + wy);
    dcm[1][0] = 2.0*(xy + wz);
    dcm[1][1] = 1.0 - 2.0*(xx + zz);
    dcm[1][2] = 2.0*(yz - wx);
    dcm[2][0] = 2.0*(xz - wy);
    dcm[2][1] = 2.0*(yz + wx);
    dcm[2][2] = 1.0 - 2.0*(xx + yy);
}

void quaternion_to_euler(const double q[4], double *roll, double *pitch, double *yaw) {
    // Placeholder: Implement proper math
    double w = q[0], x = q[1], y = q[2], z = q[3];
    // Yaw (z), Pitch (y), Roll (x)
    double sinr_cosp = 2.0 * (w*x + y*z);
    double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w*y - z*x);
    *pitch = fabs(sinp) >= 1 ? copysign(M_PI/2, sinp) : asin(sinp);

    double siny_cosp = 2.0*(w*z + x*y);
    double cosy_cosp = 1.0 - 2.0*(y*y + z*z);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

void quaternion_normalize(double q[4]) {
    double norm = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (norm > 0) {
        q[0]/=norm; q[1]/=norm; q[2]/=norm; q[3]/=norm;
    }
}

void quaternion_multiply(const double q1[4], const double q2[4], double q_out[4]) {
    double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

    q_out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q_out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q_out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q_out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

int quaternion_inverse(const double q[4], double q_inv[4]) {
    double norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_sq < 1e-14) {
        // Norm too close to zero
        return 0; // Indicate failure
    }
    q_inv[0] = q[0] / norm_sq;
    q_inv[1] = -q[1] / norm_sq;
    q_inv[2] = -q[2] / norm_sq;
    q_inv[3] = -q[3] / norm_sq;
    return 1; // Success
}

int quaternion_to_axis_angle(const double q[4], double axis[3], double *angle) {
    double w = q[0];
    double norm_vec = sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm_vec < 1e-14) {
        // Pure scalar quaternion (angle=0)
        *angle = 0.0;
        axis[0] = axis[1] = axis[2] = 0.0; // no rotation axis
        return 1; 
    }
    *angle = 2.0 * acos(w);
    double s = 1.0 / norm_vec;
    axis[0] = q[1]*s;
    axis[1] = q[2]*s;
    axis[2] = q[3]*s;
    return 1;
}

