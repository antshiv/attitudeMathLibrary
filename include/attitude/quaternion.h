#ifndef ATTITUDE_QUATERNION_H
#define ATTITUDE_QUATERNION_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Convert a quaternion to a direction cosine matrix (DCM).
 *
 * The resulting matrix rotates a vector from the body frame into the inertial frame:
 * @f$ v_{\text{world}} = \mathbf{C}_{b}^{w} \, v_{\text{body}} @f$.
 *
 * @param q     Quaternion in @f$[w, x, y, z]@f$ order.
 * @param dcm   Output @f$3\times3@f$ rotation matrix stored in row-major order.
 */
void quaternion_to_dcm(const double q[4], double dcm[3][3]);

/**
 * @brief Convert a quaternion to intrinsic ZYX Euler angles.
 *
 * Euler angles follow the aerospace convention (yaw→pitch→roll). Returned angles are in radians.
 *
 * @param q      Quaternion in @f$[w, x, y, z]@f$ order.
 * @param roll   Output rotation about the @f$x@f$ axis (rad).
 * @param pitch  Output rotation about the @f$y@f$ axis (rad).
 * @param yaw    Output rotation about the @f$z@f$ axis (rad).
 */
void quaternion_to_euler(const double q[4], double *roll, double *pitch, double *yaw);

/**
 * @brief Normalise a quaternion in-place.
 *
 * If the quaternion norm is very small the vector is left unchanged.
 *
 * @param q Quaternion to normalise.
 */
void quaternion_normalize(double q[4]);

/**
 * @brief Multiply two quaternions using Hamilton product.
 *
 * Performs @f$ q_{\text{out}} = q_1 \otimes q_2 @f$ which corresponds to applying @p q2 first,
 * followed by @p q1.
 *
 * @param q1     Left operand in @f$[w, x, y, z]@f$ order.
 * @param q2     Right operand in @f$[w, x, y, z]@f$ order.
 * @param q_out  Output quaternion product.
 */
void quaternion_multiply(const double q1[4], const double q2[4], double q_out[4]);

/**
 * @brief Compute the inverse of a quaternion.
 *
 * The inverse is defined as @f$ q^{-1} = \frac{\bar{q}}{\|q\|^2} @f$, where @f$\bar{q}@f$ is the conjugate.
 *
 * @param q      Input quaternion.
 * @param q_inv  Output inverse quaternion.
 * @return 1 if the inverse exists, 0 when the input norm is too small.
 */
int quaternion_inverse(const double q[4], double q_inv[4]);

/**
 * @brief Convert a quaternion to axis-angle representation.
 *
 * Produces a unit rotation axis and right-handed rotation angle in radians.
 *
 * @param q      Input quaternion.
 * @param axis   Output unit axis vector.
 * @param angle  Output rotation magnitude (rad).
 * @return 1 on success, 0 when the axis is undefined (quaternion too close to identity).
 */
int quaternion_to_axis_angle(const double q[4], double axis[3], double *angle);

/**
 * @brief Interpolate two quaternions using spherical linear interpolation (SLERP).
 *
 * @param q1     Start quaternion.
 * @param q2     End quaternion.
 * @param t      Interpolation parameter in @f$[0, 1]@f$.
 * @param q_out  Resulting interpolated quaternion.
 */
void quaternion_slerp(const double q1[4], const double q2[4], double t, double q_out[4]);

/**
 * @brief Rotate a vector about an axis by a given angle.
 *
 * @param axis   Unit rotation axis.
 * @param angle  Rotation angle in radians.
 * @param v_in   Input vector to rotate.
 * @param v_out  Output rotated vector.
 */
void axis_angle_rotate(const double axis[3], double angle, const double v_in[3], double v_out[3]);

/**
 * @brief Rotate a vector using a quaternion.
 *
 * Applies the efficient formulation described by @f$ v' = q v q^{-1} @f$ without explicitly forming the matrix.
 *
 * @param q      Rotation quaternion.
 * @param v_in   Input vector.
 * @param v_out  Output rotated vector.
 */
void quaternion_rotate_vector(const double q[4], const double v_in[3], double v_out[3]);

/**
 * @brief Rotate a vector using the explicit @f$q v q^\* @f$ formulation.
 *
 * Useful for educational tooling and verification of the optimised helper.
 *
 * @param q      Rotation quaternion.
 * @param v_in   Input vector.
 * @param v_out  Output rotated vector.
 */
void quaternion_rotate_vector_explicit(const double q[4], const double v_in[3], double v_out[3]);

/**
 * @brief Enable verbose logging for the explicit quaternion rotation implementation.
 *
 * When enabled, the helper emits intermediate products which is useful for tutorials and debugging.
 *
 * @param enabled Non-zero value enables logging, zero disables it.
 */
void quaternion_set_explicit_debug(int enabled);


#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_QUATERNION_H
