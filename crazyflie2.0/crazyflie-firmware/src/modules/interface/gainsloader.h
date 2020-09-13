/**
 * Author : Antonio Marino
 * date: 2020
 */

#ifndef GAINSLOADER_H_
#define GAINSLOADER_H_
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "stabilizer_types.h"

#define LOADER_PRIORITY_DISABLE 0
#define LOADER_PRIORITY_CRTP    1
#define LOADER_PRIORITY_EXTRX   2

void loaderInit(void);
bool loaderTest(void);
uint32_t loaderGetInactivityTime(void);

void loaderSetGains(gain_t *gain_setpoint, int priority);
int  loaderGetActivePriority(void);

void loaderGetGains(gain_t *gain_setpoint);

#endif /* GAINSLOAD_H_ */
