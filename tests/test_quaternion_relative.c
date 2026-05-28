#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "attitude/quaternion.h"

static int close_to(double a, double b, double tolerance) {
    return fabs(a - b) <= tolerance;
}

static double quat_abs_dot(const double q1[4], const double q2[4]) {
    return fabs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]);
}

static void assert_same_orientation(const double q1[4], const double q2[4], double tolerance) {
    assert(close_to(quat_abs_dot(q1, q2), 1.0, tolerance));
}

static double rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
}

static void print_quaternion(const char *label, const double q[4]) {
    double roll;
    double pitch;
    double yaw;

    quaternion_to_euler(q, &roll, &pitch, &yaw);

    printf("  %s quaternion [w, x, y, z] = [%.9f, %.9f, %.9f, %.9f]\n",
           label, q[0], q[1], q[2], q[3]);
    printf("  %s Euler ZYX deg          = roll %.3f, pitch %.3f, yaw %.3f\n",
           label, rad_to_deg(roll), rad_to_deg(pitch), rad_to_deg(yaw));
}

static void print_axis_angle(const char *label, const double axis[3], double angle) {
    printf("  %s axis                  = [%.9f, %.9f, %.9f]\n",
           label, axis[0], axis[1], axis[2]);
    printf("  %s angle                 = %.9f rad / %.3f deg\n",
           label, angle, rad_to_deg(angle));
}

static void make_axis_angle_quaternion(const double axis[3], double angle, double q[4]) {
    const double half = 0.5 * angle;
    const double s = sin(half);

    q[0] = cos(half);
    q[1] = axis[0] * s;
    q[2] = axis[1] * s;
    q[3] = axis[2] * s;
    quaternion_normalize(q);
}

static void test_identity_to_target_relative() {
    printf("\nTest 1: identity current to target orientation\n");

    const double q_current[4] = {1.0, 0.0, 0.0, 0.0};
    const double z_axis[3] = {0.0, 0.0, 1.0};
    double q_target[4];
    double q_error[4];

    make_axis_angle_quaternion(z_axis, M_PI / 2.0, q_target);

    assert(quaternion_relative(q_current, q_target, q_error));

    print_quaternion("current", q_current);
    print_quaternion("target ", q_target);
    print_quaternion("error  ", q_error);
    printf("  Expected: identity -> 90 deg yaw, so error equals target orientation.\n");

    assert_same_orientation(q_error, q_target, 1e-9);
}

static void test_relative_reconstructs_target() {
    printf("\nTest 2: relative rotation reconstructs target\n");

    const double x_axis[3] = {1.0, 0.0, 0.0};
    const double y_axis[3] = {0.0, 1.0, 0.0};
    double q_current[4];
    double q_target[4];
    double q_error[4];
    double q_reconstructed[4];

    make_axis_angle_quaternion(x_axis, M_PI / 6.0, q_current);
    make_axis_angle_quaternion(y_axis, M_PI / 3.0, q_target);

    assert(quaternion_relative(q_current, q_target, q_error));

    // By convention q_out = q1 ⊗ q2, applying q2 first and q1 second.
    quaternion_multiply(q_error, q_current, q_reconstructed);
    quaternion_normalize(q_reconstructed);

    print_quaternion("current      ", q_current);
    print_quaternion("target       ", q_target);
    print_quaternion("error        ", q_error);
    print_quaternion("reconstructed", q_reconstructed);
    printf("  Convention check: q_error applied after q_current reconstructs q_target.\n");

    assert_same_orientation(q_reconstructed, q_target, 1e-9);
}

static void test_camera_tracking_axis_angle_command() {
    printf("\nTest 3: camera tracking yaw correction as axis-angle\n");

    const double q_current[4] = {1.0, 0.0, 0.0, 0.0};
    const double z_axis[3] = {0.0, 0.0, 1.0};
    double q_target[4];
    double q_error[4];
    double axis[3];
    double angle;

    make_axis_angle_quaternion(z_axis, M_PI / 4.0, q_target);

    assert(quaternion_relative(q_current, q_target, q_error));
    assert(quaternion_orientation_error_axis_angle(q_current, q_target, axis, &angle));

    print_quaternion("camera current", q_current);
    print_quaternion("camera target ", q_target);
    print_quaternion("camera error  ", q_error);
    print_axis_angle("camera command", axis, angle);
    printf("  Tracking example: turn 45 deg around +Z to keep the subject centered.\n");

    assert(close_to(axis[0], 0.0, 1e-9));
    assert(close_to(axis[1], 0.0, 1e-9));
    assert(close_to(axis[2], 1.0, 1e-9));
    assert(close_to(angle, M_PI / 4.0, 1e-9));
}

int main() {
    printf("Starting quaternion relative orientation tests...\n");

    test_identity_to_target_relative();
    test_relative_reconstructs_target();
    test_camera_tracking_axis_angle_command();

    printf("\nPASS: quaternion relative orientation tests.\n");
    return 0;
}
