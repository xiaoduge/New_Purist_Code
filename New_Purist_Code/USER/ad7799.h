#ifndef _AD7799_H_
#define _AD7799_H_

void AD7799_Init(void);

uint32_t AD7799_ReadAdc(int iAdc);
uint32_t GetAdcData(uint8_t ucChl);

#endif

