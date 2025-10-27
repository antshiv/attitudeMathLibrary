#ifndef ATTITUDE_DCM_H
#define ATTITUDE_DCM_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert a direction cosine matrix to intrinsic ZYX Euler angles.
 *
 * Output angles are in radians using the aerospace yaw→pitch→roll convention.
 *
 * @param dcm   Input @f$3\times3@f$ rotation matrix (row-major).
 * @param roll  Output rotation about the x-axis (rad).
 * @param pitch Output rotation about the y-axis (rad).
 * @param yaw   Output rotation about the z-axis (rad).
 */
void dcm_to_euler(const double dcm[3][3], double *roll, double *pitch, double *yaw);

/**
 * @brief Check whether a DCM is orthonormal within a tolerance.
 *
 * @param dcm Input rotation matrix.
 * @param tol Acceptable deviation from orthonormality.
 * @return 1 if the matrix is valid, 0 otherwise.
 */
int dcm_is_orthonormal(const double dcm[3][3], double tol);

/**
 * @brief Convert a DCM to a quaternion.
 *
 * @param dcm Input rotation matrix.
 * @param q   Output quaternion in @f$[w, x, y, z]@f$ order.
 */
void dcm_to_quaternion(const double dcm[3][3], double q[4]);

/**
 * @brief Apply a DCM to a vector.
 *
 * Computes @f$ v_{\text{out}} = \mathbf{C} \, v_{\text{in}} @f$.
 *
 * @param dcm   Rotation matrix.
 * @param vin   Input vector.
 * @param vout  Output rotated vector.
 */
void dcm_apply(const double dcm[3][3], const double vin[3], double vout[3]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_DCM_H
