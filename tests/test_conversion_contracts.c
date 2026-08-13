#include <stdint.h>
#include <stdio.h>

#include "attitude/validation.h"

int main(void) {
    const uint32_t failures = attitude_validation_run();
    if (failures != 0u) {
        printf("FAIL: attitude conversion contract mask=0x%08x\n", (unsigned int)failures);
        return 1;
    }

    printf("PASS: checked attitude conversion contracts\n");
    return 0;
}
