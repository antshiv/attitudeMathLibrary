#include "attitude/attitude_utils.h"
#include <math.h>

double deg2rad(double degrees) {
    return degrees * ATTITUDE_PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / ATTITUDE_PI;
}

double wrap_angle(double angle) {
    // Wrap angle to [-pi, pi)
    while (angle >= ATTITUDE_PI) angle -= 2.0*ATTITUDE_PI;
    while (angle < -ATTITUDE_PI) angle += 2.0*ATTITUDE_PI;
    return angle;
}
