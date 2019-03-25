#include    <stdio.h>
#include    <string.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <stdarg.h>

#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>

#include    <stdint.h>

#include    <app_cfg.h>
#include    <bsp.h>

#include "stm32_eval.h"

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "Beep.h"

#include "system_time.h"

#include "OSAL_Memory.h"

#include "keyboard.h"

#include "UartCmd.h"

#include "sapp.h"

#if (ADC_SUPPORT > 0)
#include "adc.h"
#endif

#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif

#if (SERIAL_SUPPORT > 0)
#include "serial.h"
#endif

#if (RTC_SUPPORT > 0)
#include "RTC_Driver.h"
#endif

#include "adc.h"

#include "config.h"

#include "relay.h"

#include "display.h"

#include "ff.h"	

#include "LCD.h"

#include "ad7799.h"

#include "storage.h"

#include "ble.h"

#include "modbus.h"

#include "mfddriver.h"

uint32_t gulSecond;



STACK_ALIGN static OS_STK AppTaskMainStk[APP_TASK_MAIN_STK_SIZE];
STACK_ALIGN static OS_STK AppTaskRootStk[APP_TASK_ROOT_STK_SIZE];


void App_TaskIdleHook      (void)
{
    #if (IWDG_SUPPORT > 0)
    IWDG_Feed();
    #endif
    
}


void App_TimeTickHook(void)
{

    TimerProc();

    system_time_proc();

#if (SERIAL_SUPPORT > 0)
    {
        UINT8 ucLoop;

        for (ucLoop = 0; ucLoop < SERIAL_MAX_NUMBER; ucLoop++)
        {
            Serial_CheckRcvIdleTime(ucLoop);
        }
    }
#endif    
}

void InitKeys(void)
{
    GPIO_KEY key;

    key.gpio = STM32F103_GPG(2);
    key.key = KEY_CODE_NORMAL_K1;
    RegisterKey(&key);

    key.gpio = STM32F103_GPG(3);
    key.key = KEY_CODE_RECURSIVE_K2;
    RegisterKey(&key);
    
    key.gpio = STM32F103_GPG(4);
    key.key = KEY_CODE_QUANTITY_K3;
    RegisterKey(&key);

    key.gpio = STM32F103_GPG(5);
    key.key = KEY_CODE_SET_K4;
    RegisterKey(&key);
}


void  MainInit (void)
{
   //int ret;
   
   RCC_ClearFlag();

   VOS_SetLogLevel(VOS_LOG_DEBUG);

   stm32_gpio_init();

   UartCmdInit();

   osal_snv_init();
   
   Config_Init();

   Storage_Init();

#if (OS_TASK_STAT_EN > 0)
   OSStatInit();   /* Determine CPU capacity  */
#endif

   InitRelays();  

   Ioctrl_input_init();

   BeepInit();
   
   AlarmInit();
   
   RunInit();


#if (RTC_SUPPORT > 0)
   RTC_Config();
   RTC_Get();
#endif

   KeyboardInit();

   InitKeys();

#if (ADC_SUPPORT > 0)
   ADC_Meas_Init(1000000);
#endif
    LCD_Init();

    bleInit();
    
    ModbusInit();

    AD7799_Init();

    Disp_Init();

#if (IWDG_SUPPORT > 0)
    IWDG_Init();
#endif

    //mfd_EnableDebug(true);

    mfd_Init(STM32F103_GPB(11),0);

    // ylf: start timers
    AddTimer(TIMER_CHECK_SECOND,OS_TMR_OPT_PERIODIC,OS_TICKS_PER_SEC,0xffff);

    MainBeepWithDuration(1);    

    //AddTimer(TIMER_CHECK_KEY,OS_TMR_OPT_PERIODIC,(OS_TICKS_PER_SEC*10)/1000,0xffff); // 10ms

}


UINT8 PidSelfProcess(Message *pMsg)
{
    switch(pMsg->msgHead.nMsgType)
    {
    case SELF_MSG_CODE_INIT:
        MainInit();
        break;
    case SELF_MSG_CODE_DELAY:
        break;
    case SELF_MSG_CODE_USER_DISP:
        Disp_msg_Handler(pMsg);
        break;
    }
    return 0;
}

UINT8 PidKeyboardProcess(Message *pMsg)
{
    //(pMsg->msgHead.AddData) // key value
    
    //VOS_LOG(VOS_LOG_DEBUG,"key %d,%d",pMsg->msgHead.AddData,pMsg->msgHead.nMsgType);
    Disp_KeyHandler(pMsg->msgHead.AddData,pMsg->msgHead.nMsgType);    
    return 0;
}


void SecondTimer(void)
{
    gulSecond++;
    RunToggle();

    Disp_SecondTask();

    bleSecondTask();


}


UINT8 PidTimerProcess(Message *pMsg)
{
    UINT8 ucTimer = pMsg->msgHead.nMsgType;

    switch(ucTimer) /*MsgLen is userid */
    {
    case TIMER_CHECK_SECOND:
        SecondTimer();
        break;
    case TIMER_CHECK_ALARM:
        Alarm(0);
        break;
    case TIMER_CHECK_BEEP:
        Beep(0);
        break;
    case TIMER_CHECK_KEY:
        break;
    case TIMER_CHECK_HBT:
        break;
    default:
        break;
    }
    
    return 0;
}



UINT8 MainSappProc(Message *pMsg)
{
    UINT8 ucRet = TRUE;

    MainAlarmWithDuration(1);               

    sappItfType = Interface_RS232;
    sappItfPort = pMsg->msgHead.MsgSeq;
    
    switch(sbRxBuf[RPC_POS_CMD0])
    {
    case RPC_SYS_APP:
        ucRet = SHZNAPP_SerialAppProc();
        break;
    case RPC_SYS_BOOT:
        ucRet = SHZNAPP_SerialBootProc();
        break;
    default:
        ucRet = SHZNAPP_SerialUnknowProc();
        break;
    }
    if (ucRet )
    {
        (void)SHZNAPP_SerialResp(sappItfPort);  // Send the SB response setup in the sbRxBuf passed to sblProc().
    }

    return 0;
}


UINT8 Msg_proc(Message *pMsg)
{
    switch(pMsg->msgHead.nSendPid)
    {
    case PID_TIMER:
        return PidTimerProcess(pMsg);
    case PID_SELF:
        return PidSelfProcess(pMsg);
    case PID_KB:
        return PidKeyboardProcess(pMsg);
    case PID_SAPP:
        return MainSappProc(pMsg);   
#if (ADC_SUPPORT > 0)
    case PID_ADC:
        return PidAdcProcess(pMsg);
#endif 
#if (SERIAL_SUPPORT > 0)
    case PID_RS485:
        return PidSerialProcess(pMsg);
#endif        
#if (CAN_SUPPORT > 0)
    case PID_CAN:
        return PidCanProcess(pMsg);
#endif    

    default:
        break;
    }
      
    return 0;
}


void Main_Entry(Message *msg)
{
    Msg_proc(msg);

}

void MainKickoff(void)
{
    Message *Msg;
    
    Msg = MessageAlloc(PID_SELF,0);
    
    if (Msg)
    {
        Msg->msgHead.nMsgType = SELF_MSG_CODE_INIT;
        MessageSend(Msg);
    }
}


static  void  AppTaskRoot (void *p_arg)
{
 
 (void)p_arg;


  VOS_CreateTask(APP_TASK_MAIN_PRIO,(OS_STK *)&AppTaskMainStk[0],APP_TASK_MAIN_STK_SIZE,Main_Entry);

#if (OS_TASK_NAME_SIZE > 13)
  {
     INT8U err;
     OSTaskNameSet(APP_TASK_MAIN_PRIO, "Main-Task", &err);
  }
#endif


  MainKickoff();
  
  while(DEF_TRUE) 
  {
      OSTimeDlyHMSM(0,0,0,100);
  }
}



int main(void)
{ 
 
    INT8U  err;

    CPU_IntDis();

    BSP_Init(); 
    
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // ylf: 2bits for priority group 

    OSInit();                /* Initialize "uC/OS-II, The Real-Time Kernel"              */
   /* Create the start task */
   OSTaskCreateExt(AppTaskRoot,(void *)0,(OS_STK *)&AppTaskRootStk[APP_TASK_ROOT_STK_SIZE-1],APP_TASK_ROOT_PRIO,APP_TASK_ROOT_PRIO,(OS_STK *)&AppTaskRootStk[0],
                     APP_TASK_ROOT_STK_SIZE,
                     (void *)0,
                     OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);

#if (OS_TASK_NAME_SIZE > 13)
   OSTaskNameSet(APP_TASK_ROOT_PRIO, "Root-Task", &err);
#endif

    OSStart(); 
}



