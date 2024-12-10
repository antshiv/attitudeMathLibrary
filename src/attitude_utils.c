#include "attitude/attitude_utils.h"
#include <math.h>

double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

double wrap_angle(double angle) {
    // Wrap angle to [-pi, pi)
    while (angle >= M_PI) angle -= 2.0*M_PI;
    while (angle < -M_PI) angle += 2.0*M_PI;
    return angle;
}

