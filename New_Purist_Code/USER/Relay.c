#include "relay.h"

#include "Config.h"

#include <string.h>

#include "system_time.h"

typedef struct 
{
    uint8_t ucChl;
    system_timeo to;

    RelayPulse_Cb cb;
    
}RELAY_PULSE_STRU;


const uint8_t relays[RELAY_NUMBER] = {
    STM32F103_GPE(0),     
    STM32F103_GPE(1),    
    STM32F103_GPE(2),     
    STM32F103_GPE(7),     
    STM32F103_GPE(8),     
    STM32F103_GPE(9),     
    STM32F103_GPE(10),     
    STM32F103_GPE(11),     
    STM32F103_GPE(12),     
    STM32F103_GPB(9),
    //STM32F103_GPG(1),
};

const uint8_t aRelayDefaults[RELAY_NUMBER] = 
{
    1,1,1,1,1,1,1,1,1,1,
};

const uint16_t RelayInvertMask = 0x1F8;

RELAY_PULSE_STRU aPulseCtrl[RELAY_NUMBER];


void InitRelays(void)
{
    int iLoop;
    for (iLoop = 0; iLoop < RELAY_NUMBER; iLoop++)
    {
        stm32_gpio_cfgpin(relays[iLoop],MAKE_PIN_CFG(GPIO_PuPd_NOPULL,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_Mode_OUT)); 
        
        stm32_gpio_set_value(relays[iLoop],aRelayDefaults[iLoop]); // ylf:  inactive low
        
        memset(&aPulseCtrl[iLoop],0,sizeof(RELAY_PULSE_STRU));

        aPulseCtrl[iLoop].ucChl = iLoop;
    }

}

void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable)
{
    if (RelayInvertMask & (1 << ucChannel))
    {
        stm32_gpio_set_value(relays[ucChannel],!ucEnable); // ylf:  inactive low
    }
    else
    {
        stm32_gpio_set_value(relays[ucChannel],ucEnable); // ylf:  inactive low
    }

}

UINT8 GetRelayLogicStatus(UINT8 ucChannel)
{

    if (RelayInvertMask & (1 << ucChannel))
    {
        return !stm32_gpio_get_value(relays[ucChannel]);
    }
    else
    {
        return stm32_gpio_get_value(relays[ucChannel]);
    }

}


void RelayToggle(UINT8 ucChannel)
{
    stm32_gpio_toggle_value(relays[ucChannel]); // ylf:  inactive low
}

void RelayPulse_cb(void *para)
{
    RELAY_PULSE_STRU *pulse = (RELAY_PULSE_STRU *)para;
    
    RelayLogicCtrl(pulse->ucChl,FALSE);
    
    if (pulse->cb)  (pulse->cb)();
}


void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb)
{
    aPulseCtrl[ucChannel].cb = cb;

    RelayLogicCtrl(ucChannel,TRUE);

    system_timeout(duration,SYS_TIMER_ONE_SHOT,duration,RelayPulse_cb,&aPulseCtrl[ucChannel],&aPulseCtrl[ucChannel].to);
}
UINT16 GetAllRelayLogicStatus(void)
{
    UINT8 ucLoop;
    UINT16 usMode = 0;

    for (ucLoop = 0; ucLoop < RELAY_NUMBER; ucLoop++)
    {
        if (GetRelayLogicStatus (ucLoop ))
        {
            usMode |= (1 << ucLoop);
        }
    }
    return usMode;
}



/***********************************
    Put all device to its default state.
************************************/
void RelayLogicTurnoffAll(void)
{
    UINT8 ucLoop;

    for (ucLoop = 0; ucLoop < RELAY_NUMBER; ucLoop++)
    {
        RelayLogicCtrl(ucLoop,FALSE);
    }
        
}

/***********************************
    Put all device to its default state.
************************************/
void RelayLogicTurnonAll(void)
{
    UINT8 ucLoop;

    for (ucLoop = 0; ucLoop < RELAY_NUMBER; ucLoop++)
    {
        RelayLogicCtrl(ucLoop,TRUE);
    }
        
}

