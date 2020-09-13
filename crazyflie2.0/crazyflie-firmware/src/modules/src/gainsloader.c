/**
 * Author : Antonio Marino
 * date: 2020
 */

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "queue.h"

#include "gainsloader.h"
#include "crtp_gains.h"

#include "param.h"

static float l = 0;
static bool isInit;
const static gain_t nullGainSetPoint;
const static int priorityDisable = LOADER_PRIORITY_DISABLE;

static uint32_t lastUpdate;

QueueHandle_t setGainQueue;
QueueHandle_t priorityQueue;

/* Public functions */
void loaderInit(void)
{
  setGainQueue = xQueueCreate(1, sizeof(gain_t));
  ASSERT(setGainQueue);
  xQueueSend(setGainQueue, &nullGainSetPoint, 0);

  priorityQueue = xQueueCreate(1, sizeof(int));
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpGainsInit();
  lastUpdate = xTaskGetTickCount();

  isInit = true;
}

void loaderSetGains(gain_t *gain_setpoint, int priority)
{
  int currentPriority;
  xQueuePeek(priorityQueue, &currentPriority, 0);

  if (priority >= currentPriority) {
    gain_setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setGainQueue, gain_setpoint);
    xQueueOverwrite(priorityQueue, &priority);
  }
}

void loaderGetGains(gain_t *gain_setpoint)
{
  xQueuePeek(setGainQueue, gain_setpoint, 0);
  lastUpdate = gain_setpoint->timestamp;
  l = gain_setpoint->Gain1[0];
}

bool loaderTest(void)
{
  return isInit;
}

uint32_t loaderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int loaderGetActivePriority(void)
{
  int priority;
  xQueuePeek(priorityQueue, &priority, 0);
  return priority;
}

LOG_GROUP_START(gainLog)
LOG_ADD(LOG_FLOAT, init, &l)
LOG_GROUP_STOP(gainLog)
