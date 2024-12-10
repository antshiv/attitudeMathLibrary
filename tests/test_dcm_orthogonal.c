#include <stdio.h>
#include <math.h>
#include "attitude/euler.h"
#include "attitude/dcm.h"
#include "attitude/attitude_utils.h"

int main() {
    EulerAngles e = {deg2rad(0), deg2rad(90), deg2rad(45), EULER_ZYX}; // A known angle set
    double dcm[3][3];

    euler_to_dcm(&e, dcm);

    int is_ortho = dcm_is_orthonormal(dcm, 1e-9);
    if (!is_ortho) {
        printf("FAIL: DCM is not orthonormal.\n");
        return 1;
    } else {
        printf("PASS: DCM is orthonormal.\n");
    }

    return 0;
}

