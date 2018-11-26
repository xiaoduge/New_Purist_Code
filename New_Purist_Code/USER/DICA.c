#include  <ucos_ii.h>

#include  <string.h>

#include "stm32_eval.h"

#include "app_cfg.h"

#include "Dica.h"

#include "Beep.h"


const uint8_t inputs[INPUT_NUMBER] = {
    STM32F103_GPG(0),        
};


//--------------------------------------------------------------------------
// buttons_init
//
/// This function initializes the port, state and interrupt handler for the 
/// buttons component.
//--------------------------------------------------------------------------
void Ioctrl_input_init( void )
{
    int iLoop;

    // begin user add code here to initialize input configuration

    // end user add code here
#if (RTC_SUPPORT == 0)   
    /* Enable PWR and BKP clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR , ENABLE);
    
    /* Allow access to BKP Domain */
    PWR_BackupAccessCmd(ENABLE);
    
    RCC_LSEConfig(RCC_LSE_OFF);// turn off LSE to enable  PC14 PC15 as gpio
    
    //BKP_TamperPinCmd(DISABLE);
    PWR_BackupAccessCmd(DISABLE);
#endif    

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    for (iLoop = 0; iLoop < INPUT_NUMBER; iLoop++)
    {
        stm32_gpio_cfgpin(inputs[iLoop],MAKE_PIN_CFG(GPIO_PuPd_UP,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_Mode_IN)); 
    }


}

UINT8 GetInputLogicStatus(UINT8 ucChannel)
{
    return !!stm32_gpio_get_value(inputs[ucChannel]);
}

UINT16 GetAllInputLogicState(void)
{
    UINT8 ucLoop;
    UINT16 usMask = 0;

    for (ucLoop = 0; ucLoop < INPUT_NUMBER; ucLoop++)
    {
        if (GetInputLogicStatus(ucLoop))
        {
            usMask |=  1 << ucLoop;
        }
    }

    return usMask;
}

