#ifndef _DICA_H_
#define _DICA_H_


#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   STM32F4_WATER_LEVEL_N       = 0,  // INPUT FOR TANK WATER LEVEL CHECK
   INPUT_NUMBER,
}INPUT_ENUM;

#define INPUT_MASK ((1<<INPUT_NUMBER)-1)

void Ioctrl_input_init( void );
UINT8 GetInputLogicStatus(UINT8 ucChannel);
UINT16 GetAllInputLogicState(void);
void InputStateReport(void);
void InputReset(void);

#endif
