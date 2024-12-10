#ifndef ATTITUDE_EULER_H
#define ATTITUDE_EULER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EULER_ZYX, // Yaw-Pitch-Roll
    EULER_ZYZ, // e.g. for certain aerospace applications
    EULER_XYZ, // etc.
    // Add others as needed
} EulerOrder;

typedef struct {
    double roll;   // rotation about x-axis
    double pitch;  // rotation about y-axis
    double yaw;    // rotation about z-axis
    EulerOrder order;
} EulerAngles;


/**
 * Convert Euler angles to a 3x3 DCM (row-major order)
 */
void euler_to_dcm(const EulerAngles *e, double dcm[3][3]);

/**
 * Convert Euler angles to a quaternion (w, x, y, z)
 */
void euler_to_quaternion(const EulerAngles *e, double q[4]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_EULER_H

