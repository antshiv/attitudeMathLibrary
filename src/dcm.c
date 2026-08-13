#include "attitude/dcm.h"
#include <math.h>
#include <stddef.h>

int dcm_is_orthonormal(const double dcm[3][3], double tol) {
    if (dcm == NULL || !isfinite(tol) || tol < 0.0) {
        return 0;
    }

    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (!isfinite(dcm[row][column])) {
                return 0;
            }
        }
    }

    for (int left = 0; left < 3; ++left) {
        for (int right = 0; right < 3; ++right) {
            double row_dot = 0.0;
            double column_dot = 0.0;
            for (int index = 0; index < 3; ++index) {
                row_dot += dcm[left][index] * dcm[right][index];
                column_dot += dcm[index][left] * dcm[index][right];
            }
            const double expected = left == right ? 1.0 : 0.0;
            if (fabs(row_dot - expected) > tol || fabs(column_dot - expected) > tol) {
                return 0;
            }
        }
    }

    const double determinant =
        dcm[0][0] * (dcm[1][1] * dcm[2][2] - dcm[1][2] * dcm[2][1]) -
        dcm[0][1] * (dcm[1][0] * dcm[2][2] - dcm[1][2] * dcm[2][0]) +
        dcm[0][2] * (dcm[1][0] * dcm[2][1] - dcm[1][1] * dcm[2][0]);
    if (fabs(determinant - 1.0) > tol) {
        return 0;
    }

    return 1;
}

int dcm_to_euler_checked(const double dcm[3][3],
                         double *roll,
                         double *pitch,
                         double *yaw) {
    if (roll == NULL || pitch == NULL || yaw == NULL ||
        !dcm_is_orthonormal(dcm, 1e-9)) {
        return 0;
    }

    const double horizontal = hypot(dcm[0][0], dcm[1][0]);
    *pitch = atan2(-dcm[2][0], horizontal);
    if (horizontal > 1e-12) {
        *roll = atan2(dcm[2][1], dcm[2][2]);
        *yaw = atan2(dcm[1][0], dcm[0][0]);
    } else {
        *roll = 0.0;
        *yaw = atan2(-dcm[0][1], dcm[1][1]);
    }
    return 1;
}

void dcm_to_euler(const double dcm[3][3], double *roll, double *pitch, double *yaw) {
    if (!dcm_to_euler_checked(dcm, roll, pitch, yaw)) {
        if (roll != NULL) *roll = NAN;
        if (pitch != NULL) *pitch = NAN;
        if (yaw != NULL) *yaw = NAN;
    }
}

int dcm_to_quaternion_checked(const double dcm[3][3], double q[4]) {
    if (q == NULL || !dcm_is_orthonormal(dcm, 1e-9)) {
        return 0;
    }

    const double trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
    if (trace > 0.0) {
        const double scale = 2.0 * sqrt(trace + 1.0);
        q[0] = 0.25 * scale;
        q[1] = (dcm[2][1] - dcm[1][2]) / scale;
        q[2] = (dcm[0][2] - dcm[2][0]) / scale;
        q[3] = (dcm[1][0] - dcm[0][1]) / scale;
    } else if (dcm[0][0] > dcm[1][1] && dcm[0][0] > dcm[2][2]) {
        const double scale = 2.0 * sqrt(1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]);
        q[0] = (dcm[2][1] - dcm[1][2]) / scale;
        q[1] = 0.25 * scale;
        q[2] = (dcm[0][1] + dcm[1][0]) / scale;
        q[3] = (dcm[0][2] + dcm[2][0]) / scale;
    } else if (dcm[1][1] > dcm[2][2]) {
        const double scale = 2.0 * sqrt(1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]);
        q[0] = (dcm[0][2] - dcm[2][0]) / scale;
        q[1] = (dcm[0][1] + dcm[1][0]) / scale;
        q[2] = 0.25 * scale;
        q[3] = (dcm[1][2] + dcm[2][1]) / scale;
    } else {
        const double scale = 2.0 * sqrt(1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]);
        q[0] = (dcm[1][0] - dcm[0][1]) / scale;
        q[1] = (dcm[0][2] + dcm[2][0]) / scale;
        q[2] = (dcm[1][2] + dcm[2][1]) / scale;
        q[3] = 0.25 * scale;
    }

    const double norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int index = 0; index < 4; ++index) {
        q[index] /= norm;
    }
    if (q[0] < 0.0) {
        for (int index = 0; index < 4; ++index) {
            q[index] = -q[index];
        }
    }
    return 1;
}

void dcm_to_quaternion(const double dcm[3][3], double q[4]) {
    if (!dcm_to_quaternion_checked(dcm, q) && q != NULL) {
        for (int index = 0; index < 4; ++index) {
            q[index] = NAN;
        }
    }
}

void dcm_apply(const double dcm[3][3], const double vin[3], double vout[3]) {
    vout[0] = dcm[0][0]*vin[0] + dcm[0][1]*vin[1] + dcm[0][2]*vin[2];
    vout[1] = dcm[1][0]*vin[0] + dcm[1][1]*vin[1] + dcm[1][2]*vin[2];
    vout[2] = dcm[2][0]*vin[0] + dcm[2][1]*vin[1] + dcm[2][2]*vin[2];
}
