#include "attitude/vector3.h"
#include <math.h>

void vector3_add(const double a[3], const double b[3], double out[3]) {
    out[0] = a[0] + b[0];
    out[1] = a[1] + b[1];
    out[2] = a[2] + b[2];
}

void vector3_sub(const double a[3], const double b[3], double out[3]) {
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

double vector3_dot(const double a[3], const double b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void vector3_cross(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

void vector3_normalize(double v[3]) {
    double mag = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if (mag > 0) {
        v[0]/=mag; v[1]/=mag; v[2]/=mag;
    }
}

double vector3_mag(const double v[3]) {
    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

int vector3_normalize_safe(double v[3]) {
    double mag = vector3_mag(v);
    if (mag < 1e-14) {
        // Too small to normalize, return failure
        return 0;
    }
    v[0] /= mag; v[1] /= mag; v[2] /= mag;
    return 1;
}

