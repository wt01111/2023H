#ifndef SERIAL_SCREEN_H
#define SERIAL_SCREEN_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void SerialScreen_Init(void);
void SerialScreen_Task(void);
uint8_t SerialScreen_TakeSeparateRequest(void);
void SerialScreen_SendFrequencies(uint32_t freq0_hz, uint32_t freq1_hz);

#ifdef __cplusplus
}
#endif

#endif
