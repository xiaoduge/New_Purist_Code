#include    <ucos_ii.h>

#include    <cpu.h>

#include    <string.h>

#include "stm32f4xx.h"


#include "Config.h"

#include "UartCmd.h"

#include "serial.h"

#include "Relay.h"

#include "DICA.h"

#include "Beep.h"
#include "Rtc_Driver.h"

#include "mfd.h"
#include "mfddriver.h"

 
#include "sapp.h"
#if (IWDG_SUPPORT > 0)
#include "IWDG_Driver.h"
#endif


const char DataStr[]=__DATE__; 
const char TimeStr[]=__TIME__; 

//const char version[] = {"STM32F407_WDMB_V1.0"};
const char version[] = {"0.1.2.190422_WRW960"};

uint16_t gusDeviceId;

const char dtype[]   = {"SHLF0702"};
UINT8 Config_buff[1024];

SERIALNO_CONFIG_STRU gSerialNoCfg;

LOCAL_CONFIG_STRU    gLocalCfg;


uint8_t aDevType[NV_DTYPE_SIZE];

ADMIN_STRU gAdmin;

uint16_t gusOperMode;

#if CAN_SUPPORT == 0
UINT16 CanAddress = 0;
#endif

const uint8_t LocalCfgSize[LOCAL_CONFIG_NUM] = {PARAMETER_LOCAL1_SIZE,PARAMETER_LOCAL2_SIZE,PARAMETER_LOCAL3_SIZE};


const char *Config_GetVersion(void)
{
    return &version[0];
}

uint8 Config_SetItem(uint16 id, uint16 len, void *buf)
{
    uint8 ucStatus = osal_snv_write(id,len,buf);
    return ucStatus;
}

uint8 Config_GetItem(uint16 id, uint16 len, void *buf)
{
    uint8 ucStatus = osal_snv_read(id,len,buf);
    
    return ucStatus;
}

void Config_Sapp_Cmd(uint8_t cmd,uint8_t *data, uint8_t len)
{
    {
        sbTxBuf[RPC_POS_LEN]  = len; // len for data area (NOT INCLUDE CMD0&CMD1&LEN itself)
        sbTxBuf[RPC_POS_CMD0] = RPC_SYS_APP;
        sbTxBuf[RPC_POS_CMD1] = cmd;
        
        memcpy(&sbTxBuf[RPC_POS_DAT0],data,len);

        SHZNAPP_SerialResp(sappItfPort);
    }
}

#define CONTENT_POS 2
void Config_SaveSerialNo(UINT8 *pData)
{
    memcpy(gSerialNoCfg.aucSerialNo,pData,DEVICE_SERIAL_NO_SIZE);

    Config_SetItem(APP_NV_SERIALNO_ID,DEVICE_SERIAL_NO_SIZE,&gSerialNoCfg);

}

int Config_GetSerialNo(uint8_t *pucSerial)
{

    memcpy(pucSerial,gSerialNoCfg.aucSerialNo,DEVICE_SERIAL_NO_SIZE);
    return DEVICE_SERIAL_NO_SIZE;
}

int Config_SaveMmInfo(uint8_t *pInData)
{
    return write_mfd(pInData);

}

int Config_GetMmInfo(uint8_t *pInData,uint8_t *pOutData)
{
	return read_mfd(pInData,pOutData);
}



void ConfigReset(void)
{
    SCB->AIRCR = 0X05FA0004; 
}


/* |BLOCKLEN(2BYTES)|PATTERN LEN(1BYTE)|PATTERN|PORT(1BYTE)|TRIGTYPE(1BYTES)|RSPLEN(1BYTE)|RSPCONT|*/
uint8 ConfigRs485Ctrl(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
 
    return 0;
}

UINT8 ConfigGetParam4Local(uint8* pucCont,int *offset,int sub)
{

    *offset = 0;

    switch(sub)
    {
    case 0:
        memcpy(pucCont,&gLocalCfg.cfg1,LocalCfgSize[sub]);
        break;
    case 1:
        memcpy(pucCont,&gLocalCfg.cfg2,LocalCfgSize[sub]);
        break;
    case 2:
        memcpy(pucCont,&gLocalCfg.cfg3,LocalCfgSize[sub]);
        break;
    }
    

    return (LocalCfgSize[sub] + 1);
}



uint8 ConfigGetParam(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    int offset;


    switch(pCmd[2])
    {
    case DEVICE_TYPE_LOCAL:
    case DEVICE_TYPE_LOCAL+1:
    case DEVICE_TYPE_LOCAL+2:
        ucPayLoadLen = ConfigGetParam4Local(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH+1],&offset,pCmd[2]-DEVICE_TYPE_LOCAL);
        break;
     default:
        return 0xff;
    }
    pRsp[2] = pCmd[2];
    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_PARAMETER_SET_REPORT;
    
    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(pRsp[0]);

    return 0;

}

UINT8 ConfigSetParam4Local(uint8 *pCmd,uint8 CmdLen,int sub)
{
    switch(sub)
    {
    case 0:
        memcpy(&gLocalCfg.cfg1,&pCmd[SERIAL_MESSAGE_HEADER_LENGTH+1],LocalCfgSize[sub]);
        Config_SetItem(APP_NV_LOCAL_CFG_1,LocalCfgSize[sub],&gLocalCfg.cfg1);
        break;
    case 1:
        memcpy(&gLocalCfg.cfg2,&pCmd[SERIAL_MESSAGE_HEADER_LENGTH+1],LocalCfgSize[sub]);
        Config_SetItem(APP_NV_LOCAL_CFG_2,LocalCfgSize[sub],&gLocalCfg.cfg2);
        break;
    case 2:
        memcpy(&gLocalCfg.cfg3,&pCmd[SERIAL_MESSAGE_HEADER_LENGTH+1],LocalCfgSize[sub]);
        Config_SetItem(APP_NV_LOCAL_CFG_3,LocalCfgSize[sub],&gLocalCfg.cfg3);
        break;
    }

    return (LocalCfgSize[sub] + 1);
}


uint8 ConfigSetParam(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;
    int offset;
    
    IWDG_Feed();
    
    switch(pCmd[2])
    {
    case DEVICE_TYPE_LOCAL:
    case DEVICE_TYPE_LOCAL+1:
    case DEVICE_TYPE_LOCAL+2:
        ucPayLoadLen = ConfigSetParam4Local(pCmd,CmdLen,pCmd[2]-DEVICE_TYPE_LOCAL);
        ConfigGetParam4Local(&pRsp[SERIAL_MESSAGE_HEADER_LENGTH+1],&offset,pCmd[2]-DEVICE_TYPE_LOCAL);
        break;
    default:
        return 0xff;
    }
    
    // backup parameter area
#ifdef CHECK_CFG    
    pRsp[2] = pCmd[2];
    pRsp[0] = SERIAL_MSG_SIZE(ucPayLoadLen);
    pRsp[1] = CMD_CLIENT2HOST_PARAMETER_SET_REPORT;
#else
    pRsp[0] = 0;
    pRsp[1] = CMD_HOST2CLIENT_PARAMETER_SET;
    (void)ucPayLoadLen;
#endif

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(pRsp[0]);

    return 0;

}


void ConfigSetTime(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
   UINT8 *pCanMsg = (UINT8 *)&pCmd[CONTENT_POS];

   uint16_t year = pCanMsg[0] + 1900;

   UINT8 ucPayLoadLen = 0;
   
   UINT8 ucIdx = 0;

   RTC_Set(year,pCanMsg[1],pCanMsg[2],pCanMsg[3],pCanMsg[4],pCanMsg[5]);

   pRsp[ucIdx++] = ucPayLoadLen;
   pRsp[ucIdx++] = CMD_HOST2CLIENT_TIME_CTRL;

   *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);
   
}

void ConfigGetTime(uint8 *pCmd,uint8 CmdLen,uint8 *pRsp,uint8 *pucRspLen)
{
    UINT8 ucPayLoadLen;

    UINT8 ucIdx = 0;
    
    RTC_Get();

    ucPayLoadLen = 6;
    
    // backup parameter area
    pRsp[ucIdx++] = ucPayLoadLen;
    pRsp[ucIdx++] = CMD_HOST2CLIENT_TIME_GET;
    
    pRsp[ucIdx++] = timer.w_year - 1900;
    pRsp[ucIdx++] = timer.w_month;
    pRsp[ucIdx++] = timer.w_date;
    pRsp[ucIdx++] = timer.hour;
    pRsp[ucIdx++] = timer.min;
    pRsp[ucIdx++] = timer.sec;

    *pucRspLen += SERIAL_MSG_TOTAL_SIZE(ucPayLoadLen);
   
}


uint8 Config_Entry(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen)
{
    uint8 ucRet = 0;

    //VOS_LOG(VOS_LOG_DEBUG,"%d,%d",pCmd[0],pCmd[1]); // pCmd[0] For length

    switch(pCmd[1])
    {
#if 0   
    case CMD_HOST2CLIENT_DEVICEID_SET:
        ucRet = ConfigSetDeviceId(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_DEVICEID_GET:
        ucRet = ConfigGetDeviceId(pCmd,CmdLen,pRsp,pucRspLen);
        break;
#endif      
    case CMD_HOST2CLIENT_PARAMETER_SET:
        ucRet = ConfigSetParam(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_PARAMETER_GET:
        ucRet = ConfigGetParam(pCmd,CmdLen,pRsp,pucRspLen);
        break;  
    case CMD_HOST2CLIENT_TIME_CTRL:
        ConfigSetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_TIME_GET:
        ConfigGetTime(pCmd,CmdLen,pRsp,pucRspLen);
        break;
    case CMD_HOST2CLIENT_BEEP:
        MainBeepWithDuration(1);
        break;
    case CMD_HOST2CLIENT_LIGHT:
        MainAlarmWithDuration(1);
        break;
    }

    return ucRet;
}

uint8 Config_Sapp_Entry(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen)
{
   
   return Config_Entry(pCmd,pRsp,CmdLen,pucRspLen);
}

void Config_Init(void)
{

    memset(&gAdmin,0,sizeof(gAdmin));
    
    if ( osal_snv_read( NV_DEVID_ID, sizeof ( gusDeviceId ), &gusDeviceId ) != ERROR_SUCCESS )
    {
        gusDeviceId = 0;
    }

    if ( (osal_snv_read( NV_DTYPE_ID, NV_DTYPE_SIZE, &aDevType[0]) != ERROR_SUCCESS) 
        || (0 != memcmp(aDevType,dtype,NV_DTYPE_ID)))
    {
        memcpy(aDevType,dtype,NV_DTYPE_SIZE);
    
        Config_SetItem(NV_DTYPE_ID,NV_DTYPE_SIZE,aDevType);
    }

    if (ERROR_SUCCESS !=  Config_GetItem(APP_NV_SERIALNO_ID,DEVICE_SERIAL_NO_SIZE,gSerialNoCfg.aucSerialNo))
    {
        memcpy(gSerialNoCfg.aucSerialNo,"WDMB-XXXX",9);
    }

    if ( osal_snv_read( STM32_NV_APP_ADMIN, ADMIN_TEXT_LENGTH, &gAdmin.NAME[0] ) != ERROR_SUCCESS )
    {
        strcpy(gAdmin.NAME,"ADMIN");
    }

    if ( osal_snv_read( STM32_NV_APP_ADMIN+1, ADMIN_TEXT_LENGTH, &gAdmin.PSD[0] ) != ERROR_SUCCESS )
    {
        strcpy(gAdmin.PSD,"860860");
    }

    if ( osal_snv_read( APP_NV_LOCAL_CFG_1, sizeof(LOCAL_CONFIG1_STRU), &gLocalCfg.cfg1) != ERROR_SUCCESS )
    {
        memset(&gLocalCfg.cfg1,0XFF,sizeof(LOCAL_CONFIG1_STRU));
    }

    if ( osal_snv_read( APP_NV_LOCAL_CFG_2, sizeof(LOCAL_CONFIG2_STRU), &gLocalCfg.cfg2) != ERROR_SUCCESS )
    {
        memset(&gLocalCfg.cfg2,0XFF,sizeof(LOCAL_CONFIG2_STRU));
    }

    if ( osal_snv_read( APP_NV_LOCAL_CFG_3, sizeof(LOCAL_CONFIG3_STRU), &gLocalCfg.cfg3) != ERROR_SUCCESS )
    {
        memset(&gLocalCfg.cfg3,0XFF,sizeof(LOCAL_CONFIG3_STRU));
    }

    //printf("%d %d %d %d %d %d %d\r\n",gTslibLinearCfg.a[0],gTslibLinearCfg.a[1],gTslibLinearCfg.a[2],gTslibLinearCfg.a[3],gTslibLinearCfg.a[4],gTslibLinearCfg.a[5],gTslibLinearCfg.a[6]);

}

