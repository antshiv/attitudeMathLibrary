#include <stdio.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

int main() {
    // Test a few Euler orders
    EulerAngles e = {deg2rad(30.0), deg2rad(45.0), deg2rad(90.0), EULER_ZYX};

    double dcm[3][3];
    euler_to_dcm(&e, dcm);

    // Convert back
    double roll, pitch, yaw;
    dcm_to_euler(dcm, &roll, &pitch, &yaw);

    double roll_deg_out = rad2deg(roll);
    double pitch_deg_out = rad2deg(pitch);
    double yaw_deg_out = rad2deg(yaw);

    printf("Original: (30°,45°,90°)  Recovered: (%.2f°, %.2f°, %.2f°)\n", roll_deg_out, pitch_deg_out, yaw_deg_out);

    // Add checks for deviation and print test result
    return 0;
}

