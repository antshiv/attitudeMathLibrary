#include <stdio.h>
#include <math.h>

#include "attitude/euler.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

int main() {
    // Original Euler angles in degrees
    double roll_deg = 30.0;
    double pitch_deg = 45.0;
    double yaw_deg = 90.0;

    // Convert to radians
    EulerAngles e;
    e.roll = deg2rad(roll_deg);
    e.pitch = deg2rad(pitch_deg);
    e.yaw = deg2rad(yaw_deg);

    // Convert Euler angles (radians) to DCM
    double dcm[3][3];
    euler_to_dcm(&e, dcm);

    // Convert DCM back to Euler angles (in radians)
    double roll_rad_out, pitch_rad_out, yaw_rad_out;
    dcm_to_euler(dcm, &roll_rad_out, &pitch_rad_out, &yaw_rad_out);

    // Convert back to degrees
    double roll_deg_out = rad2deg(roll_rad_out);
    double pitch_deg_out = rad2deg(pitch_rad_out);
    double yaw_deg_out = rad2deg(yaw_rad_out);

    // Print results
    printf("Original Euler (deg):  roll=%.2f, pitch=%.2f, yaw=%.2f\n", roll_deg, pitch_deg, yaw_deg);
    printf("Recovered Euler (deg): roll=%.2f, pitch=%.2f, yaw=%.2f\n", roll_deg_out, pitch_deg_out, yaw_deg_out);

    // Check if close to original (within some small tolerance)
    double tolerance = 1e-6;
    int success = (fabs(roll_deg - roll_deg_out) < tolerance) &&
                  (fabs(pitch_deg - pitch_deg_out) < tolerance) &&
                  (fabs(yaw_deg - yaw_deg_out) < tolerance);

    if (success) {
        printf("Test passed: Recovered angles match the original (within tolerance).\n");
    } else {
        printf("Test failed: Recovered angles deviate from the original.\n");
    }

    return success ? 0 : 1;
}

