#ifndef ATTITUDE_QUATERNION_H
#define ATTITUDE_QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Convert a quaternion to a DCM
 */
void quaternion_to_dcm(const double q[4], double dcm[3][3]);

/**
 * Convert a quaternion to Euler angles
 */
void quaternion_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

/**
 * Normalize a quaternion
 */
void quaternion_normalize(double q[4]);

/**
 * Multiply two quaternions: q_out = q1 * q2
 */
void quaternion_multiply(const double q1[4], const double q2[4], double q_out[4]);

void quaternion_slerp(const double q1[4], const double q2[4], double t, double q_out[4]);

void axis_angle_rotate(const double axis[3], double angle, const double v_in[3], double v_out[3]);


#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_QUATERNION_H

