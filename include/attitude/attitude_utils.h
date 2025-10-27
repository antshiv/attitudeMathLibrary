#ifndef ATTITUDE_UTILS_H
#define ATTITUDE_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Macro helper converting degrees to radians.
 */
#define DEG2RAD(x) ((x) * M_PI / 180.0)
/**
 * @brief Macro helper converting radians to degrees.
 */
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

/**
 * @brief Convert degrees to radians.
 *
 * @param degrees Angle in degrees.
 * @return Angle in radians.
 */
double deg2rad(double degrees);

/**
 * @brief Convert radians to degrees.
 *
 * @param radians Angle in radians.
 * @return Angle in degrees.
 */
double rad2deg(double radians);

/**
 * @brief Wrap an angle to the @f$(-\pi, \pi]@f$ interval.
 *
 * @param angle Angle in radians.
 * @return Wrapped angle.
 */
double wrap_angle(double angle);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_UTILS_H
