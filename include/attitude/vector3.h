#ifndef ATTITUDE_VECTOR3_H
#define ATTITUDE_VECTOR3_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Basic vector operations
 */

void vector3_add(const double a[3], const double b[3], double out[3]);
void vector3_sub(const double a[3], const double b[3], double out[3]);
double vector3_dot(const double a[3], const double b[3]);
void vector3_cross(const double a[3], const double b[3], double out[3]);
void vector3_normalize(double v[3]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_VECTOR3_H

