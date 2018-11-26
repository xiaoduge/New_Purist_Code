#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   CONDUCTIVE_CH1       = 0,  // 紫外灯开启信号，高有效，默认关闭
   CONDUCTIVE_CH2,
   CONDUCTIVE_CH3,
   STM32F4_VALVE1_24V_EN, //RELAY_VALVE_CIRCULATE_E5
   STM32F4_VALVE2_24V_EN, //RELAY_VALVE_WATER_INTAKING_E6
   STM32F4_VALVE3_24V_EN, //RELAY_VALVE_WATER_INFLOWING_E7
   STM32F4_VALVE4_24V_EN, //RELAY_VALVE_FLUSH_E8
   STM32F4_LAMP_24V_EN,   //RELAY_ULTRA_VIOLET_LAMP
   STM32F4_PUMP_24V_EN,   //RELAY_PUMP_CIRCULATE_C3
   STM32F4_WATER_LED_N,   //RELAY_LED_WATER_ANYIN
   //STM32F4_UART6_TXEN,    
   RELAY_NUMBER,
}RELAY_ENUM;

/*ALIAS */

#define ch1_sw PEout(0)
#define ch2_sw PEout(1)
#define ch3_sw PEout(2)


#define MAX_OUTPUT_PORT_NUMBER RELAY_NUMBER

#define OUTPUT_MASK ((1<<MAX_OUTPUT_PORT_NUMBER)-1)

#define MAX_RELAY_NUMBER  (RELAY_NUMBER)
typedef void (*RelayPulse_Cb)(void);

void InitRelays(void);
void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable);
UINT8 GetRelayLogicStatus(UINT8 ucChannel);
UINT16 GetAllRelayLogicStatus(void);
void RelayToggle(UINT8 ucChannel);
void RelayLogicTurnoffAll(void);
void RelayLogicTurnonAll(void);
void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb);

#endif
