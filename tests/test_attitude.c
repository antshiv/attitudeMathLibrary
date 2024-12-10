#include <stdio.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/quaternion.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

int main() {
    EulerAngles e = {0.0, M_PI/4, M_PI/2}; // roll=0, pitch=45°, yaw=90°
    double dcm[3][3];

    printf("The original angles in (deg): roll=%.2f, pitch=%.2f, yaw=%.2f\n", rad2deg(e.roll), rad2deg(e.pitch), rad2deg(e.yaw));
     
    euler_to_dcm(&e, dcm);

    double q[4];
    euler_to_quaternion(&e, q);

    printf("Quaternion: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", q[0], q[1], q[2], q[3]);

    double roll, pitch, yaw;
    dcm_to_euler(dcm, &roll, &pitch, &yaw);


    // Convert back to degrees
    double roll_deg_out = rad2deg(roll);
    double pitch_deg_out = rad2deg(pitch);
    double yaw_deg_out = rad2deg(yaw);
    
    printf("Back from DCM: roll=%.3f, pitch=%.3f, yaw=%.3f\n", roll, pitch, yaw);
    printf("Recovered Euler (deg): roll=%.2f, pitch=%.2f, yaw=%.2f\n", roll_deg_out, pitch_deg_out, yaw_deg_out);

    return 0;
}

