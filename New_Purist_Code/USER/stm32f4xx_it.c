/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include <ucos_ii.h>

#include "stm32_eval.h"

#include "Timer_Driver.h"

#include <ucos_ii.h>

#include "memory.h"

#include "task.h"


#include "adc.h"

#if (SERIAL_SUPPORT > 0)
#include "serial.h"
#endif

#include "UartCmd.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

 

 

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

  //static uint32_t TimingDelay = 0;
  /* Decrement the TimingDelay variable */
  //Decrement_TimingDelay();
  //TimingDelay++;
  OS_CPU_SR  cpu_sr;
 
  OS_ENTER_CRITICAL();  //����ȫ���жϱ�־,�����ж�/* Tell uC/OS-II that we are starting an ISR*/
  if (OSRunning)
      OSIntNesting++;
  OS_EXIT_CRITICAL();   //�ָ�ȫ���жϱ�־
  
  OSTimeTick();     /* Call uC/OS-II's OSTimeTick(),��os_core.c�ļ��ﶨ��,��Ҫ�ж���ʱ�������Ƿ��ʱ��*/
  
  OSIntExit();  //��os_core.c�ļ��ﶨ��,����и������ȼ������������,��ִ��һ�������л�              

}


static void USART_IRQCommHandler(int iPort)
{
 
    if(USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_RXNE) != RESET)
    {
      UINT8 ucRcvData;
      /* Read one byte from the receive data register */
      ucRcvData = USART_ReceiveData(Serial[iPort].UsartDef);
#if (SERIAL_SUPPORT > 0)
      Serial_FillRcvBuf(iPort,&ucRcvData,1);
#else
      ucRcvData = ucRcvData;
#endif
  
    }

#if (SERIAL_SUPPORT > 0)
    if(USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_TXE) != RESET)
    {   

      if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
      {
        /* Disable the USART Transmit interrupt */
        USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TXE, DISABLE);

      }
      else
      {
          USART_SendData(Serial[iPort].UsartDef, Serial[iPort].SndBuff[Serial[iPort].usSndRear]);
          Serial[iPort].usSndRear = (Serial[iPort].usSndRear + 1)%SERIAL_MAX_SEND_BUFF_LENGTH;

          if (RS485 == Serial[iPort].ucPortType)
          {
              if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
              {
                 USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TC, ENABLE);
              }
          }
      }
    }

    
    if (RS485 == Serial[iPort].ucPortType)
    {
        if (USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_TC) != RESET)
        {
            if(SERIAL_EMPTY(Serial[iPort].usSndFront, Serial[iPort].usSndRear))
            {
               SerialEnableTx(iPort,FALSE); 
               
               USART_ITConfig(Serial[iPort].UsartDef, USART_IT_TC, DISABLE);
            }
        }
    }
    
#endif      

}

static void USART_StdioCommHandler(int iPort)
{
    if(USART_GetITStatus(Serial[iPort].UsartDef, USART_IT_RXNE) != RESET)
    {
      UINT8 ucRcvData;
      /* Read one byte from the receive data register */
      ucRcvData = USART_ReceiveData(Serial[iPort].UsartDef);
#if (SERIAL_SUPPORT > 0)
      Serial_FillRcvBuf(iPort,&ucRcvData,1);
#else
      ucRcvData = ucRcvData;
#endif
  
    }

}

void USART1_IRQHandler(void)
{
#ifdef USE_USART1_STDIO
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      UINT8 ucRcvData;
      /* Read one byte from the receive data register */
      ucRcvData = USART_ReceiveData(USART1);
      
      {
          UartCmdPutData(ucRcvData);        
      }
    }
#else

    if (UART_STDIO == Serial[SERIAL_PORT0].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT0);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT0);
    }
#endif
}

/**
* @brief  This function handles USART2 global interrupt request.
* @param  None
* @retval None
*/

void USART2_IRQHandler(void)
{
    if (UART_STDIO == Serial[SERIAL_PORT1].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT1);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT1);
    }
}

/**
  * @brief  This function handles USART3 global interrupt request.
  * @param  None
  * @retval None
  */
  
void USART3_IRQHandler(void)
{
    if (UART_STDIO == Serial[SERIAL_PORT2].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT2);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT2);
    }
}

void  UART4_IRQHandler(void)
{
    if (UART_STDIO == Serial[SERIAL_PORT3].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT3);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT3);
    }
}

void  UART5_IRQHandler(void)
{
    if (UART_STDIO == Serial[SERIAL_PORT4].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT4);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT4);
    }
}

void  USART6_IRQHandler(void)
{
    if (UART_STDIO == Serial[SERIAL_PORT5].ucPortMode)
    {
        USART_StdioCommHandler(SERIAL_PORT5);
    }
    else
    {
        USART_IRQCommHandler(SERIAL_PORT5);
    }
}


void TIM1_CC_IRQHandler()
{
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)  
    {  
        uint16_t capture;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);  
          
        capture = TIM_GetCapture1(TIM1);  
        TIM_SetCompare1(TIM1, capture + 1000);      

        TIM_event_handler(TIMER4,TIM_IT_CC1);      

     }        

}


void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

      // User Add callback functions here
      TIM_event_handler(TIMER1,TIM_IT_Update);      
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);  
      TIM_event_handler(TIMER1,TIM_IT_CC1);      
  }  
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);  
      TIM_event_handler(TIMER1,TIM_IT_CC2);      
   } 
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);  
      TIM_event_handler(TIMER1,TIM_IT_CC3);      
  } 

}

void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    TIM_event_handler(TIMER2,TIM_IT_Update);      
  }

   if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);  
       TIM_event_handler(TIMER2,TIM_IT_CC1);      
   }  
   
   if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);  
       TIM_event_handler(TIMER2,TIM_IT_CC2);      
   } 
   
   if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)  
   {  
       TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);  
       TIM_event_handler(TIMER2,TIM_IT_CC3);      
   } 
  
}

void TIM4_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    
    TIM_event_handler(TIMER3,TIM_IT_Update);      
  }
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);  
      TIM_event_handler(TIMER3,TIM_IT_CC1);      
  }  
  
  if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);  
      
      TIM_event_handler(TIMER3,TIM_IT_CC2);      
  } 
  
  if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)  
  {  
      TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);  
      
      TIM_event_handler(TIMER3,TIM_IT_CC3);      
  } 
  
}


void CAN1_RX0_IRQHandler(void)
{
#if  CAN_SUPPORT
    {
        Message *msg;
        // do someting here
        msg = MessageAlloc(PID_CAN,sizeof(CanRxMsg));
        //  UART_PutChar('T');
        
        if (msg)
        {
        
          msg->msgHead.nRcvPid = PID_MAIN_TASK;
          msg->msgHead.nMsgType = 0;
          CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)msg->data);
          
          MessageSend(msg);
        }
         
    }
#endif
}

void CAN1_RX1_IRQHandler(void)
{
#if  CAN_SUPPORT
    {
        Message *msg;
        
        // do someting here
        msg = MessageAlloc(PID_CAN,sizeof(CanRxMsg));
        //  UART_PutChar('T');
        
        if (msg)
        {
        
          msg->msgHead.nRcvPid = PID_MAIN_TASK;
          msg->msgHead.nMsgType = 0;
          CAN_Receive(CAN1, CAN_FIFO1, (CanRxMsg *)msg->data);
          
          MessageSend(msg);
        }
         
    }
#endif
}


#if (CAN_SUPPORT > 0)
void SndCanData(void);

void CAN1_TX_IRQHandler(void)
{
    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP0))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP0);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
        
        // do someting
        SndCanData();
    }

    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP1))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP1);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);
        
        // do someting
        SndCanData();
    }

    if(SET == CAN_GetITStatus(CAN1,CAN_IT_RQCP2))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP2);
        //CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);

        // do someting
        SndCanData();
    }

}
#endif
  
/**
* @brief  This function handles external interrupts generated by UserButton.
* @param  None
* @retval None
*/


void EXTI0_IRQHandler(void)
{
  /* set UserButton Flag */
  
  EXTI_ClearITPendingBit(EXTI_Line0);

  sensor_event_handler(DI_SENSOR1);
  
  //MainAlarmWithDuration(1);
}

    
  /**
  * @brief  This function handles external interrupts generated by UserButton.
  * @param  None
  * @retval None
  */
  
  void EXTI1_IRQHandler(void)
  {
    /* set UserButton Flag */
    
    EXTI_ClearITPendingBit(EXTI_Line1);

    sensor_event_handler(DI_SENSOR2);
    
    //MainAlarmWithDuration(1);
  }

    
  /**
  * @brief  This function handles external interrupts generated by UserButton.
  * @param  None
  * @retval None
  */
  
  void EXTI2_IRQHandler(void)
  {
    /* set UserButton Flag */
    
    EXTI_ClearITPendingBit(EXTI_Line2);

    
    sensor_event_handler(DI_SENSOR3);
    //MainAlarmWithDuration(1);
  }

    
  /**
  * @brief  This function handles external interrupts generated by UserButton.
  * @param  None
  * @retval None
  */
  
  void EXTI3_IRQHandler(void)
  {
    /* set UserButton Flag */
    
    EXTI_ClearITPendingBit(EXTI_Line3);

    sensor_event_handler(DI_SENSOR4);
    
    //MainAlarmWithDuration(1);
  }

    
  /**
  * @brief  This function handles external interrupts generated by UserButton.
  * @param  None
  * @retval None
  */
  
  void EXTI4_IRQHandler(void)
  {
    /* set UserButton Flag */
    
    EXTI_ClearITPendingBit(EXTI_Line4);

    
    sensor_event_handler(DI_SENSOR5);
    //MainAlarmWithDuration(1);
  }

/**
* @brief  This function handles External lines 9 to 5 interrupt request.
* @param  None
* @retval None
*/
void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) != RESET) // ylf: pending inter
	{
	  /* Clear the EXTI Line 5 */
	  EXTI_ClearITPendingBit(EXTI_Line5);
      sensor_event_handler(DI_SENSOR6);
      
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{

	  /* Clear the EXTI Line 6 */
	  EXTI_ClearITPendingBit(EXTI_Line6);
      sensor_event_handler(DI_SENSOR7);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 7 */
	  EXTI_ClearITPendingBit(EXTI_Line7);
      sensor_event_handler(DI_SENSOR8);
	}
	
	if (EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
	  /* UP function */
	  /* Clear the EXTI Line 8 */
	  EXTI_ClearITPendingBit(EXTI_Line8);
      sensor_event_handler(DI_SENSOR9);
	}

    if (EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
      /* UP function */
      /* Clear the EXTI Line 9 */
      EXTI_ClearITPendingBit(EXTI_Line9);
      sensor_event_handler(DI_SENSOR10);
    }

	//MainAlarmWithDuration(1);

}

/**
* @brief  This function handles External lines 9 to 5 interrupt request.
* @param  None
* @retval None
*/
void EXTI15_10_IRQHandler(void)
{

    if (EXTI_GetITStatus(EXTI_Line10) != RESET)
    {

      /* Clear the EXTI Line 10 */
      EXTI_ClearITPendingBit(EXTI_Line10);
      
      sensor_event_handler(10);
      
    }

    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
      /* UP function */
      /* Clear the EXTI Line 11 */
      EXTI_ClearITPendingBit(EXTI_Line11);
      sensor_event_handler(11);
      
    }
    
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
      /* UP function */
      /* Clear the EXTI Line 13 */
      EXTI_ClearITPendingBit(EXTI_Line12);
      
      sensor_event_handler(12);
    }
    
    if (EXTI_GetITStatus(EXTI_Line13) != RESET)
    {
      
      /* Clear the EXTI Line 14*/
      EXTI_ClearITPendingBit(EXTI_Line13);
      /* UP function */
      sensor_event_handler(13);
    }
    
    if (EXTI_GetITStatus(EXTI_Line14) != RESET)
    {
      /* Clear the EXTI Line 15 */
      EXTI_ClearITPendingBit(EXTI_Line14);
      /* UP function */
      sensor_event_handler(14);
    }
    if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
      /* Clear the EXTI Line 15 */
      EXTI_ClearITPendingBit(EXTI_Line15);
      /* UP function */
      sensor_event_handler(15);
    }

    //MainAlarmWithDuration(1);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
