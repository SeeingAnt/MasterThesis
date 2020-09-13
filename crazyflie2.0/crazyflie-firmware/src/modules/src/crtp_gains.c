/**
 * Author : Antonio Marino
 * date: 2020
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "log.h"

#include "param.h"
#include "crtp.h"
#include "num.h"
#include "quatcompress.h"
#include "FreeRTOS.h"

#include "crtp_gains.h"

#include "gainsloader.h"

static float got=0;
static bool isInit;

static void gainCrtpCB(CRTPPacket* pk);

void crtpGainsInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_GAIN1, gainCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_GAIN2, gainCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_GAIN3, gainCrtpCB);
  crtpRegisterPortCB(CRTP_PORT_GAIN4, gainCrtpCB);
  isInit = true;
}


static void gainCrtpCB(CRTPPacket* pk)
{
  static gain_t gain_setpoint;

  if(pk->port == CRTP_PORT_GAIN1 && pk->channel == 0){
	  loaderGetGains(&gain_setpoint);
	  crtpGainsGenericDecode(gain_setpoint.Gain1, pk);
  	  loaderSetGains(&gain_setpoint, LOADER_PRIORITY_CRTP );
  }else if(pk->port == CRTP_PORT_GAIN2 && pk->channel == 0){
	  loaderGetGains(&gain_setpoint);
	  crtpGainsGenericDecode(gain_setpoint.Gain2, pk);
	  loaderSetGains(&gain_setpoint, LOADER_PRIORITY_CRTP );

  }else if(pk->port == CRTP_PORT_GAIN3 && pk->channel == 0){
	  loaderGetGains(&gain_setpoint);
	  crtpGainsGenericDecode(gain_setpoint.Gain3, pk);
	  loaderSetGains(&gain_setpoint, LOADER_PRIORITY_CRTP );

  }else if(pk->port == CRTP_PORT_GAIN4 && pk->channel == 0){
	  loaderGetGains(&gain_setpoint);
	  crtpGainsGenericDecode(gain_setpoint.Gain4, pk);
	  loaderSetGains(&gain_setpoint, LOADER_PRIORITY_CRTP );

  }

}

struct gainStruct_s {
  int16_t gain1;         // x10*3
  int16_t gain2;
  int16_t gain3;
  int16_t gain4;
  int16_t gain5;
  int16_t gain6;
  int16_t gain7;
  int16_t gain8;
  int16_t gain9;
  int16_t gain10;
  int16_t gain11;
  int16_t gain12;
} __attribute__((packed));
static void gainsDecoder(float *gain_setpoint, const void *data, size_t datalen)
{
  const struct gainStruct_s *values = data;

  ASSERT(datalen == sizeof(struct gainStruct_s));

  #define UNPACK(i,j) \
  gain_setpoint[i] = (values->gain ## j)/1000.0f; \

  UNPACK(0,1)
  UNPACK(1,2)
  UNPACK(2,3)
  UNPACK(3,4)
  UNPACK(4,5)
  UNPACK(5,6)
  UNPACK(6,7)
  UNPACK(7,8)
  UNPACK(8,9)
  UNPACK(9,10)
  UNPACK(10,11)
  UNPACK(11,12)
  #undef UNPACK

}


/* Decoder switch */
void crtpGainsGenericDecode(float *gain_setpoint, CRTPPacket *pk)
{
    gainsDecoder(gain_setpoint, ((char*)pk->data)+1, pk->size-1);

}

LOG_GROUP_START(ctrpgainLog)
LOG_ADD(LOG_FLOAT, init, &got)
LOG_GROUP_STOP(ctrpgainLog)

