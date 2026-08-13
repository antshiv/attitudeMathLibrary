#ifndef ATTITUDE_VALIDATION_H
#define ATTITUDE_VALIDATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum AttitudeValidationFailure {
    ATTITUDE_VALIDATION_EULER_TO_DCM = 1u << 0,
    ATTITUDE_VALIDATION_ORTHONORMAL = 1u << 1,
    ATTITUDE_VALIDATION_EULER_TO_QUATERNION = 1u << 2,
    ATTITUDE_VALIDATION_QUATERNION_DCM = 1u << 3,
    ATTITUDE_VALIDATION_DCM_TO_QUATERNION = 1u << 4,
    ATTITUDE_VALIDATION_DCM_TO_EULER = 1u << 5,
    ATTITUDE_VALIDATION_INVALID_INPUT = 1u << 6
};

/**
 * @brief Run deterministic attitude conversion fixtures.
 *
 * The fixture has no heap, file, clock, or operating-system dependencies, so the
 * same code can run in host CI and on embedded targets.
 *
 * @return Zero on success or a bitwise OR of AttitudeValidationFailure values.
 */
uint32_t attitude_validation_run(void);

#ifdef __cplusplus
}
#endif

#endif
