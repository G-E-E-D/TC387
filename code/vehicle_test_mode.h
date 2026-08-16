#ifndef VEHICLE_TEST_MODE_H
#define VEHICLE_TEST_MODE_H

#include <stdbool.h>

/* Temporary bench-test firmware switch. Set to 0 to return to vehicle_app. */
#define VEHICLE_TEMP_TEST_MODE (1U)

bool vehicle_test_mode_init(void);
void vehicle_test_mode_process(void);

#endif
