/**
 * Author : Antonio Marino
 * date: 2020
 *
 - Aggressive Controller Interface
 */
#ifndef __CONTROLLER_AGGRESSIVE_H__
#define __CONTROLLER_AGGRESSIVE_H__

#include "stabilizer_types.h"

void controllerAggressiveInit(void);
bool controllerAggressiveTest(void);
void controllerAggressive(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_AGGRESSIVE_H__
