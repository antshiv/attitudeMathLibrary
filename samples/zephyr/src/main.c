#include <stdint.h>
#include <stdio.h>

#include <zephyr/kernel.h>

#include "attitude/validation.h"

int main(void) {
    const uint32_t failures = attitude_validation_run();
    if (failures == 0u) {
        printf("ATTITUDE_VALIDATION PASS\n");
        return 0;
    }

    printf("ATTITUDE_VALIDATION FAIL mask=0x%08x\n", (unsigned int)failures);
    return 1;
}
