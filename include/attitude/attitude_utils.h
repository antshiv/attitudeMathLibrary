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

double deg2rad(double degrees);
double rad2deg(double radians);
double wrap_angle(double angle);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_UTILS_H

