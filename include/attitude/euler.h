#ifndef ATTITUDE_EULER_H
#define ATTITUDE_EULER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enumeration of supported Euler rotation orders.
 *
 * The library currently implements intrinsic ZYX rotations. The other values are
 * reserved so callers can reject unsupported inputs through the checked API.
 */
typedef enum {
    EULER_ZYX, ///< Standard aerospace yaw → pitch → roll sequence.
    EULER_ZYZ, ///< Useful for certain satellite pointing laws and robotics arms.
    EULER_XYZ, ///< Roll → pitch → yaw intrinsic sequence.
    // Extend with other orders as needed.
} EulerOrder;

/**
 * @brief Storage for Euler angles with an associated rotation order.
 *
 * Angles are expressed in radians.
 */
typedef struct {
    double roll;   ///< Rotation about x-axis.
    double pitch;  ///< Rotation about y-axis.
    double yaw;    ///< Rotation about z-axis.
    EulerOrder order; ///< Intrinsic rotation order.
} EulerAngles;


/**
 * @brief Convert Euler angles to a direction cosine matrix.
 *
 * @param e    Euler angle container in radians.
 * @param dcm  Output @f$3\times3@f$ matrix (row-major).
 */
void euler_to_dcm(const EulerAngles *e, double dcm[3][3]);

/**
 * @brief Checked Euler-to-DCM conversion.
 *
 * @return 1 on success, 0 for null pointers, non-finite angles, or an unsupported order.
 */
int euler_to_dcm_checked(const EulerAngles *e, double dcm[3][3]);

/**
 * @brief Convert Euler angles to a quaternion.
 *
 * @param e  Euler angle container in radians.
 * @param q  Output quaternion in @f$[w, x, y, z]@f$ order.
 */
void euler_to_quaternion(const EulerAngles *e, double q[4]);

/**
 * @brief Checked Euler-to-quaternion conversion.
 *
 * @return 1 on success, 0 for null pointers, non-finite angles, or an unsupported order.
 */
int euler_to_quaternion_checked(const EulerAngles *e, double q[4]);

#ifdef __cplusplus
}
#endif

#endif // ATTITUDE_EULER_H
