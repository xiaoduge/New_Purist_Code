#include    <ucos_ii.h>

#include    <cpu.h>

#include    <string.h>

#include "stm32f4xx.h"

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "stm32_eval.h"


#include "app_cfg.h"

#include "Config.h"

#include "Beep.h"

#include "Errorcode.h"

#include "common.h"

#include "sapp.h"

#include "cminterface.h"

#include "osal_snv.h"

#include "RTC_Driver.h"

#include "Display.h"

#include "Task.h"

#include "Relay.h"

#include "blecmitf.h"

#include "serial.h"

#include "ble.h"


#define SBL_BUF_SIZE                 196
#define SBL_MAX_SIZE                (SBL_BUF_SIZE - RPC_FRAME_HDR_SZ - RPC_UART_FRAME_OVHD)

#define MAX_SERAIL_MESSAGE_LEN       SBL_BUF_SIZE

typedef struct
{
    unsigned char ucStage;
    unsigned char ucIndex;
    unsigned char ucMsgLen;
    unsigned char ucChecksum;
    unsigned char Data[MAX_SERAIL_MESSAGE_LEN];
}SERIAL_BUS_PARSER;

typedef void (*halBLECBack_t) (uint8 port, uint8 event);

typedef struct
{
    halBLECBack_t cb;
}BLE_STUR;

static SERIAL_BUS_PARSER  SerBusParser;

//static BLE_STUR sBle;


void ble_Serial_InitParser(void)
{
    SerBusParser.ucStage = rpcSteSOF;
    SerBusParser.ucIndex = 0;
}


int ble_ParseMessage(uint8_t *pucData,int len)
{
    unsigned char ucData;

    int bgn = 0;
    
    while(bgn < len)
    {
        ucData = pucData[bgn++];
        
        switch(SerBusParser.ucStage)
        {
        case rpcSteSOF:
            if (RPC_UART_SOF == ucData)
            {
                SerBusParser.ucStage = rpcSteLen;
            }
            break;
        case rpcSteLen:
           if (ucData > SBL_MAX_SIZE)
           {
             SerBusParser.ucStage = rpcSteSOF;
             break;
           }
           else
           {
             SerBusParser.ucStage = rpcSteData;
             
             SerBusParser.ucChecksum = 0;
             SerBusParser.ucIndex = 0;
             SerBusParser.ucMsgLen = ucData + (RPC_FRAME_HDR_SZ );  // Combine the parsing of Len, Cmd0 & Cmd1 with the data for BUS TYPE NONE ,OTHER busitf.
             // no break;
           }            

        case rpcSteData:
            SerBusParser.ucChecksum ^= ucData;
            SerBusParser.Data[SerBusParser.ucIndex] = ucData;
            
            if (++SerBusParser.ucIndex == SerBusParser.ucMsgLen)
            {
              SerBusParser.ucStage = rpcSteFcs;
            }
            break;
        case rpcSteFcs:
          SerBusParser.ucStage = rpcSteSOF;
        
          if ((SerBusParser.ucChecksum == ucData)) //  && ((SerKeyParser.Data[RPC_POS_CMD0] & RPC_SUBSYSTEM_MASK) == RPC_SYS_BOOT
          {
            return 1; 
          }
          break;
        default:
          SerBusParser.ucStage = rpcSteSOF;
          SerBusParser.ucIndex = 0;
          break;
        }
    }
    return 0;
}



/**************************************************************************************************
 * @fn          ble_SerialResp
 *
 * @brief       Make the SB response.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the downloaded code has been enabled; FALSE otherwise.
 */
void ble_SerialResp(void)
{
  uint8 fcs = 0, len = sbTxBuf[RPC_POS_LEN] + RPC_FRAME_HDR_SZ;
  uint8 idx;
    
  for ( idx = RPC_POS_LEN; idx < len; idx++)
  {
    fcs ^= sbTxBuf[idx];
  }
  sbTxBuf[len] = fcs;

  sbTxBuf[-1] = sappTgtType; // 
  
  sappTgtType = RPC_UART_SOF4TA;

  Serial_FillSndBuf(SERIAL_PORT2,sbTxBuf-1, len + RPC_UART_FRAME_OVHD);

}

void bleReset(void)
{
    sbTxBuf[RPC_POS_LEN]  = 0; // no data
    sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
    sbTxBuf[RPC_POS_CMD1] = SAPP_CMD_RESET;

    sappTgtType = RPC_UART_SOF;
    
    ble_SerialResp();

    //Disp_zbResetInd();
}

void ble_PrepareReset()
{

}

void ble_config_cb(uint8_t ucPort)
{
    memset(&Serial[ucPort],0,offsetof(SERIAL_STRU,ucDriverType));

    Serial[ucPort].ucDriverType = MSG_DRIVER;
    Serial[ucPort].ucPortType   = RS232;
    Serial[ucPort].ucPortCtrl   = 0; // DONT CARE FOR RS232

    switch(ucPort)
    {
    case SERIAL_PORT2:
        Serial[ucPort].UsartDef = USART3;
        Serial[ucPort].iIrq     = USART3_IRQn;
        Serial[ucPort].iComIdx  = SERIAL_PORT2;
        break;
    }

    Serial_RetriveConfig(ucPort);

    Serial[ucPort].SerialConfig.BaundRate  = 115200;
    Serial[ucPort].SerialConfig.ucDataBits = BAUD_DATA_8BITS;
    Serial[ucPort].SerialConfig.ucStopBits = BAUD_STOP_1BITS;
    Serial[ucPort].SerialConfig.ucParity   = BAUD_PARITY_NO;

}

void ble_SappProc(uint8_t *pucData,uint8_t len)
{
    uint8_t ucRet;

    switch(pucData[RPC_POS_CMD1])
    {
    case SAPP_CMD_DATA:
        {
            uint8_t ucRspLen = 1;
            
            /* skip protol */
            sbRxBuf[RPC_POS_LEN]  = pucData[RPC_POS_LEN];
            sbRxBuf[RPC_POS_CMD0] = pucData[RPC_POS_CMD0];
            sbRxBuf[RPC_POS_CMD1] = pucData[RPC_POS_CMD1];

            memcpy(&sbRxBuf[RPC_POS_DAT0],pucData + RPC_POS_DAT0,len - RPC_FRAME_HDR_SZ);
            
            ucRet = Disp_DataHandler(&sbRxBuf[RPC_POS_DAT0],&sbTxBuf[SAPP_RSP_STATUS + 1],SAPP_PAYLOAD_LEN(sbLen),&ucRspLen);

            sbTxBuf[RPC_POS_LEN] = ucRspLen; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
            sbTxBuf[RPC_POS_CMD0] = sbRxBuf[RPC_POS_CMD0];
            sbTxBuf[RPC_POS_CMD1] = sbRxBuf[RPC_POS_CMD1]|SAPP_RSP_MASK;
            sbTxBuf[RPC_POS_DAT0] = ucRet; // state

        }
        break;
    case (SAPP_CMD_AT | 0X80):
        {
            /* 2017/06 ADD for changing BLE Name */
            DispBleProc(pucData,len);
            return ; 
        }
    default:
        break;
    }

    ble_SerialResp();		
    
    (void )ucRet;
}

void bleSecondTask(void)
{
}

void bleCmdSerialWrite(uint8 ucCmd,uint8 *pBuffer, uint16 length)
{
    if (length <= SAPP_MAX_SIZE)
    {
        sbTxBuf[RPC_POS_LEN]  = length;
        sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
        sbTxBuf[RPC_POS_CMD1] = ucCmd;
    
        memcpy(&sbTxBuf[RPC_POS_DAT0],pBuffer, length);
    
        (void)ble_SerialResp();
    }
}

void ble_ItfProcess(Message *pMsg)
{
    if (ble_ParseMessage((uint8_t *)pMsg->data,pMsg->msgHead.nMsgLen))
    {
       /* process frame */
       switch(SerBusParser.Data[RPC_POS_CMD0])
       {
       case RPC_SYS_BOOT:
          break;
       case RPC_SYS_APP:
          ble_SappProc(SerBusParser.Data,SerBusParser.ucMsgLen);
          break;
       }
    }

}


void bleInit(void)
{
   uint8_t ucPortIdx = SERIAL_PORT2;

   sappTgtType = RPC_UART_SOF4TA;

   Serial[ucPortIdx].sccb = ble_config_cb;

   Serial[ucPortIdx].mcb  = ble_ItfProcess;

   SerialInitPort(ucPortIdx);   
   
   //sys_timeout(1000,SYS_TIMER_PERIOD,1000,Modbus_Scheduler_cb,NULL,&ModbusSheduler.to4Schedule); 
   ble_Serial_InitParser();

   ble_PrepareReset();
}

