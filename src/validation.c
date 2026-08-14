#include "attitude/validation.h"

#include <math.h>

#include "attitude/dcm.h"
#include "attitude/euler.h"
#include "attitude/quaternion.h"

static double matrix_error(const double left[3][3], const double right[3][3]) {
    double sum = 0.0;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const double difference = left[row][column] - right[row][column];
            sum += difference * difference;
        }
    }
    return sqrt(sum);
}

static uint32_t check_round_trip(const EulerAngles *input) {
    const double tolerance = 1e-12;
    double expected[3][3];
    double quaternion[4];
    double reconstructed[3][3];
    double roll;
    double pitch;
    double yaw;
    uint32_t failures = 0;

    if (!euler_to_dcm_checked(input, expected)) {
        return ATTITUDE_VALIDATION_EULER_TO_DCM;
    }
    if (!dcm_is_orthonormal((const double (*)[3])expected, tolerance)) {
        failures |= ATTITUDE_VALIDATION_ORTHONORMAL;
    }

    if (!euler_to_quaternion_checked(input, quaternion)) {
        failures |= ATTITUDE_VALIDATION_EULER_TO_QUATERNION;
    } else {
        quaternion_to_dcm(quaternion, reconstructed);
        if (matrix_error((const double (*)[3])expected,
                         (const double (*)[3])reconstructed) >= tolerance) {
            failures |= ATTITUDE_VALIDATION_QUATERNION_DCM;
        }
    }

    if (!dcm_to_quaternion_checked((const double (*)[3])expected, quaternion)) {
        failures |= ATTITUDE_VALIDATION_DCM_TO_QUATERNION;
    } else {
        quaternion_to_dcm(quaternion, reconstructed);
        if (matrix_error((const double (*)[3])expected,
                         (const double (*)[3])reconstructed) >= tolerance) {
            failures |= ATTITUDE_VALIDATION_DCM_TO_QUATERNION;
        }
    }

    if (!dcm_to_euler_checked((const double (*)[3])expected, &roll, &pitch, &yaw)) {
        failures |= ATTITUDE_VALIDATION_DCM_TO_EULER;
    } else {
        const EulerAngles recovered = {roll, pitch, yaw, EULER_ZYX};
        if (!euler_to_dcm_checked(&recovered, reconstructed) ||
            matrix_error((const double (*)[3])expected,
                         (const double (*)[3])reconstructed) >= tolerance) {
            failures |= ATTITUDE_VALIDATION_DCM_TO_EULER;
        }
    }

    return failures;
}

static uint32_t check_rejections(void) {
    const EulerAngles unsupported = {0.1, 0.2, 0.3, EULER_XYZ};
    const EulerAngles non_finite = {NAN, 0.2, 0.3, EULER_ZYX};
    const double reflection[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, -1.0}
    };
    double dcm[3][3];
    double quaternion[4];
    double roll;
    double pitch;
    double yaw;

    if (euler_to_dcm_checked(&unsupported, dcm) ||
        euler_to_quaternion_checked(&unsupported, quaternion) ||
        euler_to_dcm_checked(&non_finite, dcm) ||
        dcm_is_orthonormal(reflection, 1e-12) ||
        dcm_to_quaternion_checked(reflection, quaternion) ||
        dcm_to_euler_checked(reflection, &roll, &pitch, &yaw)) {
        return ATTITUDE_VALIDATION_INVALID_INPUT;
    }
    return 0;
}

uint32_t attitude_validation_run(void) {
    const double pi = 3.14159265358979323846;
    const EulerAngles cases[] = {
        {0.0, 0.0, 0.0, EULER_ZYX},
        {0.3, -0.4, 1.2, EULER_ZYX},
        {pi, 0.0, 0.0, EULER_ZYX},
        {0.0, pi, 0.0, EULER_ZYX},
        {0.0, 0.0, pi, EULER_ZYX},
        {0.7, pi / 2.0, -0.2, EULER_ZYX},
        {-0.8, -pi / 2.0, 1.1, EULER_ZYX}
    };
    uint32_t failures = 0;

    for (unsigned int index = 0; index < sizeof(cases) / sizeof(cases[0]); ++index) {
        failures |= check_round_trip(&cases[index]);
    }
    failures |= check_rejections();
    return failures;
}
