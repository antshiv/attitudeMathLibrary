#include "attitude/dcm.h"
#include <math.h>

int dcm_is_orthonormal(const double dcm[3][3], double tol) {
    // Check dot products of rows/cols
    // For a perfect DCM, rows are orthogonal and normalized.
    double r0_r1 = dcm[0][0]*dcm[1][0] + dcm[0][1]*dcm[1][1] + dcm[0][2]*dcm[1][2];
    double r0_norm = dcm[0][0]*dcm[0][0] + dcm[0][1]*dcm[0][1] + dcm[0][2]*dcm[0][2];
    // Check these and similarly for r1, r2, and cross-check columns.
    // If all checks pass within tolerance 'tol', return 1; else 0.
    
    if (fabs(r0_r1) > tol) return 0;
    if (fabs(r0_norm - 1.0) > tol) return 0;
    // ... repeat for other rows/columns ...

    return 1;
}


void dcm_to_euler(const double dcm[3][3], double *roll, double *pitch, double *yaw) {
    // Placeholder: Implement proper extraction
    *pitch = -asin(dcm[2][0]);
    *roll = atan2(dcm[2][1], dcm[2][2]);
    *yaw = atan2(dcm[1][0], dcm[0][0]);
}

void dcm_to_quaternion(const double dcm[3][3], double q[4]) {
    // Placeholder: Implement DCM to quaternion conversion
    double trace = dcm[0][0] + dcm[1][1] + dcm[2][2];
    if (trace > 0) {
        double s = 0.5 / sqrt(trace+1.0);
        q[0] = 0.25 / s;
        q[1] = (dcm[2][1] - dcm[1][2]) * s;
        q[2] = (dcm[0][2] - dcm[2][0]) * s;
        q[3] = (dcm[1][0] - dcm[0][1]) * s;
    } else {
        // Handle other cases...
        // This is a placeholder for brevity.
        q[0] = 1; q[1] = q[2] = q[3] = 0;
    }
}

void dcm_apply(const double dcm[3][3], const double vin[3], double vout[3]) {
    vout[0] = dcm[0][0]*vin[0] + dcm[0][1]*vin[1] + dcm[0][2]*vin[2];
    vout[1] = dcm[1][0]*vin[0] + dcm[1][1]*vin[1] + dcm[1][2]*vin[2];
    vout[2] = dcm[2][0]*vin[0] + dcm[2][1]*vin[1] + dcm[2][2]*vin[2];
}

