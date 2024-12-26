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
    double w = q[0], x = q[1], y = q[2], z = q[3];

    // Normalize quaternion
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (norm == 0.0) {
        *roll = 0.0;
        *pitch = 0.0;
        *yaw = 0.0;
        printf("Quaternion norm is zero, setting roll, pitch, and yaw to 0.\n");
        return;
    }
    w /= norm; x /= norm; y /= norm; z /= norm;

    // Compute intermediate values
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    *roll = atan2(sinr_cosp, cosr_cosp);
    printf("sinr_cosp: %f, cosr_cosp: %f, roll: %f\n", sinr_cosp, cosr_cosp, *roll);

    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1.0) {
        *pitch = copysign(M_PI / 2, sinp);
        printf("Gimbal lock detected at pitch. sinp: %f, pitch: %f\n", sinp, *pitch);
    } else {
        *pitch = asin(sinp);
        printf("sinp: %f, pitch: %f\n", sinp, *pitch);
    }

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    *yaw = atan2(siny_cosp, cosy_cosp);
    printf("siny_cosp: %f, cosy_cosp: %f, yaw: %f\n", siny_cosp, cosy_cosp, *yaw);

    // Print final results
    printf("Final computed angles - Roll: %f, Pitch: %f, Yaw: %f\n", *roll, *pitch, *yaw);
}

void quaternion_normalize(double q[4]) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    
    if (fabs(norm - 1.0) > 1e-6) {
        printf("Quaternion is not normalized: norm = %f\n", norm);
    }
    
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

void quaternion_slerp(const double q1[4], const double q2[4], double t, double q_out[4]) {
    double dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

    // Ensure shortest path
    double q2_adjusted[4];
    if (dot < 0.0) {
        dot = -dot;
        q2_adjusted[0] = -q2[0];
        q2_adjusted[1] = -q2[1];
        q2_adjusted[2] = -q2[2];
        q2_adjusted[3] = -q2[3];
    } else {
        q2_adjusted[0] = q2[0];
        q2_adjusted[1] = q2[1];
        q2_adjusted[2] = q2[2];
        q2_adjusted[3] = q2[3];
    }

    // If quaternions are nearly identical, use linear interpolation
    if (dot > 0.9995) {
        for (int i = 0; i < 4; ++i) {
            q_out[i] = q1[i] + t * (q2_adjusted[i] - q1[i]);
        }
        quaternion_normalize(q_out);
        return;
    }

    // Calculate SLERP
    double theta = acos(dot);
    double sin_theta = sin(theta);
    double weight1 = sin((1 - t) * theta) / sin_theta;
    double weight2 = sin(t * theta) / sin_theta;

    for (int i = 0; i < 4; ++i) {
        q_out[i] = weight1 * q1[i] + weight2 * q2_adjusted[i];
    }
}

void axis_angle_rotate(const double axis[3], double angle, const double v_in[3], double v_out[3]) {
    double cos_theta = cos(angle);
    double sin_theta = sin(angle);

    double dot = axis[0] * v_in[0] + axis[1] * v_in[1] + axis[2] * v_in[2];
    double cross[3];
    cross[0] = axis[1] * v_in[2] - axis[2] * v_in[1];
    cross[1] = axis[2] * v_in[0] - axis[0] * v_in[2];
    cross[2] = axis[0] * v_in[1] - axis[1] * v_in[0];

    for (int i = 0; i < 3; ++i) {
        v_out[i] = v_in[i] * cos_theta + cross[i] * sin_theta + axis[i] * dot * (1 - cos_theta);
    }
}

