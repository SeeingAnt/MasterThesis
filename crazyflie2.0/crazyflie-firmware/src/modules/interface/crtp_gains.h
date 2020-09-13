/**
 * Author : Antonio Marino
 * date: 2020
 */

#ifndef CRTP_GAINS_H_
#define CRTP_GAINS_H_

#include <stdint.h>
#include "stabilizer_types.h"
#include "crtp.h"

void crtpGainsInit(void);
void crtpGainsGenericDecode(float *gain_setpoint, CRTPPacket *pk);

#endif /* CRTP_GAINS_H_ */
