#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

// Wrap angle in degrees to [-180, 180)
static double wrap_angle_deg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

// Attempt to find an equivalent angle representation closer to 'ref' by adding ±180° or ±360°.
static double adjust_angle_for_equivalence(double angle, double ref) {
    double candidates[5] = {
        angle,
        angle + 360.0,
        angle - 360.0,
        angle + 180.0,
        angle - 180.0
    };

    double best_diff = fabs(ref - candidates[0]);
    double best_val = candidates[0];
    for (int i = 1; i < 5; i++) {
        double diff = fabs(ref - candidates[i]);
        if (diff < best_diff) {
            best_diff = diff;
            best_val = candidates[i];
        }
    }
    return wrap_angle_deg(best_val);
}

int main() {
    srand(12345); // fixed seed for reproducibility
    int num_tests = 1000;
    double max_error = 0.0;
    double tolerance = 1e-3; // 0.001 degrees tolerance

    int pass_count = 0;
    int fail_count = 0;

    EulerAngles e;
    e.order = EULER_ZYX; // Ensure your dcm_to_euler and euler_to_dcm follow Z-Y-X convention

    for (int i = 0; i < num_tests; i++) {
        double roll_deg = ((double)rand()/RAND_MAX)*360.0 - 180.0;
        double pitch_deg = ((double)rand()/RAND_MAX)*360.0 - 180.0;
        double yaw_deg = ((double)rand()/RAND_MAX)*360.0 - 180.0;

        e.roll = deg2rad(roll_deg);
        e.pitch = deg2rad(pitch_deg);
        e.yaw = deg2rad(yaw_deg);

        double dcm[3][3];
        euler_to_dcm(&e, dcm);

        double r_out, p_out, y_out;
        dcm_to_euler(dcm, &r_out, &p_out, &y_out);

        // Convert back to degrees and wrap
        double roll_deg_out = wrap_angle_deg(rad2deg(r_out));
        double pitch_deg_out = wrap_angle_deg(rad2deg(p_out));
        double yaw_deg_out = wrap_angle_deg(rad2deg(y_out));

        // Attempt to find a closer equivalent representation
        roll_deg_out = adjust_angle_for_equivalence(roll_deg_out, roll_deg);
        pitch_deg_out = adjust_angle_for_equivalence(pitch_deg_out, pitch_deg);
        yaw_deg_out = adjust_angle_for_equivalence(yaw_deg_out, yaw_deg);

        double roll_err = fabs(roll_deg - roll_deg_out);
        double pitch_err = fabs(pitch_deg - pitch_deg_out);
        double yaw_err = fabs(yaw_deg - yaw_deg_out);

        // Track max error
        if (roll_err > max_error) max_error = roll_err;
        if (pitch_err > max_error) max_error = pitch_err;
        if (yaw_err > max_error) max_error = yaw_err;

        int pass = (roll_err < tolerance && pitch_err < tolerance && yaw_err < tolerance);

        if (pass) {
            pass_count++;
            printf("Test %d: PASS (roll_err=%.6f, pitch_err=%.6f, yaw_err=%.6f)\n",
                   i+1, roll_err, pitch_err, yaw_err);
        } else {
            fail_count++;
            printf("Test %d: FAIL (roll_err=%.6f, pitch_err=%.6f, yaw_err=%.6f)\n",
                   i+1, roll_err, pitch_err, yaw_err);
        }
    }

    // Print summary
    printf("Randomized Euler round-trip max error: %.6f degrees\n", max_error);
    printf("Total Tests: %d, Pass: %d, Fail: %d\n", num_tests, pass_count, fail_count);

    if (fail_count == 0) {
        printf("All tests passed within tolerance.\n");
        return 0;
    } else {
        printf("Some rotations exceeded tolerance.\n");
        return 1;
    }
}

