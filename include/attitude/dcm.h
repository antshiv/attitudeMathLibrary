#ifndef ATTITUDE_DCM_H
#define ATTITUDE_DCM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Convert a DCM to Euler angles
 */
void dcm_to_euler(const double dcm[3][3], double *roll, double *pitch, double *yaw);

/**
 * Convert a DCM to a quaternion
 */
void dcm_to_quaternion(const double dcm[3][3], double q[4]);

/**
 * Apply a DCM to a vector: v_out = DCM * v_in
 */
void dcm_apply(const double dcm[3][3], const double vin[3], double vout[3]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_DCM_H

