#ifndef SIGNAL_SEPARATION_H
#define SIGNAL_SEPARATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void SignalSeparation_Start(void);
void SignalSeparation_Task(void);
void SignalSeparation_RestartIdentify(void);
uint8_t SignalSeparation_GetFrequencies(uint32_t *freq0_hz, uint32_t *freq1_hz);

#ifdef __cplusplus
}
#endif

#endif
