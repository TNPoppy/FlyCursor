#ifndef ADCBAT_H
#define ADCBAT_H

#include <stdint.h>

void ADCBAT_Init(void);
uint16_t ADCBAT_ReadVoltageMV(void);
uint8_t  ADCBAT_ReadPercentage(void);

#endif
