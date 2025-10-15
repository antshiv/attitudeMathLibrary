#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

/*
 * This regression used to compare Euler angles directly.  Example: start with
 * 30°, round-trip through the DCM, and the library might hand back -330°
 * because it always reports angles in [-180°, 180°).  Those two numbers differ
 * by 360° but describe the exact same orientation.  The old test looked only at
 * the degrees and falsely declared “360° error”, so random samples spanning the
 * branch cuts failed even though the rotation matrix was correct.
 *
 * Fix: keep printing the angles for debugging, but judge correctness by the
 * Frobenius norm of the DCM mismatch.  Identical rotations—regardless of how
 * the angles wrap—now pass (‖ΔR‖_F < 1e-9), while true discrepancies still trip
 * the test.
 */

/* Wrap angle in degrees to [-180, 180).  These values are printed to help spot
 * branch-cut behaviour in the logs, but they no longer drive the pass/fail
 * decision. */
static double wrap_angle_deg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

/* Frobenius norm of the difference between two rotation matrices.  If the
 * Euler round-trip is correct, the matrices should agree to within floating
 * point noise regardless of how the individual angles are wrapped. */
static double dcm_difference_norm(const double a[3][3], const double b[3][3]) {
    double sum_sq = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double diff = a[i][j] - b[i][j];
            sum_sq += diff * diff;
        }
    }
    return sqrt(sum_sq);
}

int main() {
    srand(12345); // fixed seed for reproducibility
    int num_tests = 1000;
    double max_angle_error = 0.0;
    double max_dcm_error = 0.0;
    const double angle_tolerance = 1e-3;  // purely informational
    const double dcm_tolerance = 1e-9;    // Frobenius norm threshold

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
        dcm_to_euler((const double (*)[3])dcm, &r_out, &p_out, &y_out);

        EulerAngles recovered = {.roll = r_out,
                                 .pitch = p_out,
                                 .yaw = y_out,
                                 .order = EULER_ZYX};

        double dcm_recovered[3][3];
        euler_to_dcm(&recovered, dcm_recovered);

        // Convert back to degrees and wrap
        double roll_deg_out = wrap_angle_deg(rad2deg(r_out));
        double pitch_deg_out = wrap_angle_deg(rad2deg(p_out));
        double yaw_deg_out = wrap_angle_deg(rad2deg(y_out));

        double roll_err = fabs(roll_deg - roll_deg_out);
        double pitch_err = fabs(pitch_deg - pitch_deg_out);
        double yaw_err = fabs(yaw_deg - yaw_deg_out);

        // Track max angle deviations (informational only)
        if (roll_err > max_angle_error) max_angle_error = roll_err;
        if (pitch_err > max_angle_error) max_angle_error = pitch_err;
        if (yaw_err > max_angle_error) max_angle_error = yaw_err;

        const double (*dcm_const)[3] = (const double (*)[3])dcm;
        const double (*dcm_recovered_const)[3] = (const double (*)[3])dcm_recovered;
        double dcm_err = dcm_difference_norm(dcm_const, dcm_recovered_const);
        if (dcm_err > max_dcm_error) {
            max_dcm_error = dcm_err;
        }

        int pass = (dcm_err < dcm_tolerance);

        if (pass) {
            ++pass_count;
        } else {
            ++fail_count;
            printf("Test %d: FAIL (roll_err=%.6f, pitch_err=%.6f, yaw_err=%.6f, dcm_err=%.12e)\n",
                   i + 1, roll_err, pitch_err, yaw_err, dcm_err);
        }
    }

    // Print summary
    printf("Randomized Euler round-trip max angle deviation: %.6f degrees\n", max_angle_error);
    printf("Randomized Euler round-trip max DCM Frobenius error: %.12e\n", max_dcm_error);
    printf("Angle tolerance (informational): %.3e deg, DCM tolerance (pass criterion): %.3e\n",
           angle_tolerance, dcm_tolerance);
    printf("Total Tests: %d, Pass: %d, Fail: %d\n", num_tests, pass_count, fail_count);

    if (fail_count == 0) {
        printf("All tests passed within tolerance.\n");
        return 0;
    } else {
        printf("Some rotations exceeded tolerance.\n");
        return 1;
    }
}
