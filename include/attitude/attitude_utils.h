#ifndef ATTITUDE_UTILS_H
#define ATTITUDE_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Utilities and helper functions, e.g.:
 * - safe angle wrapping
 * - small-angle approximations
 * - unit conversions (degrees <-> radians)
 */

// Macros for degree-radian conversion
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

double deg2rad(double degrees);
double rad2deg(double radians);
double wrap_angle(double angle);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_UTILS_H

