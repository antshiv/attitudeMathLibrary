#ifndef ATTITUDE_VECTOR3_H
#define ATTITUDE_VECTOR3_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Add two 3D vectors component-wise.
 *
 * @param a   First operand.
 * @param b   Second operand.
 * @param out Output @f$a + b@f$.
 */
void vector3_add(const double a[3], const double b[3], double out[3]);

/**
 * @brief Subtract two 3D vectors component-wise.
 *
 * @param a   Minuend.
 * @param b   Subtrahend.
 * @param out Output @f$a - b@f$.
 */
void vector3_sub(const double a[3], const double b[3], double out[3]);

/**
 * @brief Compute the dot product of two vectors.
 *
 * @param a First vector.
 * @param b Second vector.
 * @return Dot product @f$a \cdot b@f$.
 */
double vector3_dot(const double a[3], const double b[3]);

/**
 * @brief Compute the cross product of two vectors.
 *
 * @param a   First vector.
 * @param b   Second vector.
 * @param out Output @f$a \times b@f$.
 */
void vector3_cross(const double a[3], const double b[3], double out[3]);

/**
 * @brief Normalise a 3D vector in-place.
 *
 * @param v Vector to normalise.
 */
void vector3_normalize(double v[3]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_VECTOR3_H
