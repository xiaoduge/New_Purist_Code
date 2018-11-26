#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "App_cfg.h"

#include "osal_snv.h"

#include "common.h"

#include "DICA.h"

#include "cminterface.h"

// common for all

#pragma   pack(1)

typedef struct
{
    UINT16 usTgtAdr;    // for target adr
    UINT8  ucTgtObj;    // for target obj
    UINT8  ucTgtIdx;    // for target index
    UINT8  ucTgtOps;    // for target operation code
	UINT8  ucTgtData;   // delay output time
    UINT8  ucCtlType;   //  input trigger type
    UINT8  ucRsvd1;     //  reserved
}INPUT_MAP_STRU;

typedef struct
{
    UINT8  aucSrcAdr[2];
    UINT8  ucSrcObj;
	UINT8  ucSrcIdx;
	UINT8  ucDstObj;
	UINT8  ucDstIdx;
	UINT8  ucCtlLen;
	UINT8  aucData[1];
}CFG_PEER_CTL;

typedef enum
{
    APP_NV_LOCAL_CFG_1               = 0x80,  // just for example
    APP_NV_LOCAL_CFG_2                     ,
    APP_NV_LOCAL_CFG_3                     ,

    
    APP_NV_SERIALNO_ID,
    

    STM32_NV_APP_PARAM_AREA          = 0xB0, 
    
    STM32_NV_APP_PARAM_BACK          , 

    STM32_NV_APP_ADMIN               ,

    

}APP_NV_ID_ENUM;

#define CMD_HOST2CLIENT_LIGHT 0XF0   //
#define CMD_HOST2CLIENT_BEEP 0xF1    // 
#define CMD_HOST2CLIENT_QUERY_PIN 0xF4    //
#define CMD_HOST2CLIENT_BUZZ 0xF6        // 
#define CMD_HOST2CLIENT_RESET 0xF7        // 
#define CMD_HOST2CLIENT_MOTOR_CTRL 0xFA        // Pulse Oximeter Command
#define CMD_HOST2CLIENT_CAN_CTRL 0xFB        // local simulating can testing
#define CMD_HOST2CLIENT_RS485_CTRL 0xFD // ird configuration bin downloading
        #define CMD_CMD_HOST2CLIENT_RS485_CTRL_SEND 0
        #define CMD_CMD_HOST2CLIENT_RS485_CTRL_GET  1

#define CMD_HOST2CLIENT_PARAMETER_SET 0xD0        // 
#define CMD_HOST2CLIENT_PARAMETER_GET 0xD1        // 

#define CMD_HOST2CLIENT_DEVICEID_SET 0xD2        // 
#define CMD_HOST2CLIENT_DEVICEID_GET 0xD3        // 

#define CMD_HOST2CLIENT_CANID_SET 0xD4        // 
#define CMD_HOST2CLIENT_CANID_GET 0xD5        // 

#define CMD_HOST2CLIENT_DEVICE_RESET 0xD6     // RESET
#define CMD_HOST2CLIENT_FORCE_BOOT   0xd7


#define CMD_HOST2CLIENT_TIME_CTRL 0xC0        // 
#define CMD_HOST2CLIENT_TIME_GET  0xC1        // 
#define CMD_HOST2CLIENT_TIME_ADJ  0xC2        // 

#define CMD_CLIENT2HOST_REPORT 0X00  //管脚状态上报
#define CMD_CLIENT2HOST_PARAMETER_SET_REPORT 0X10  //管脚状态上报
#define CMD_CLIENT2HOST_ID_SET_REPORT 0X11  //设备ID上报
#define CMD_CLIENT2HOST_CANID_SET_REPORT 0X12  //设备ID上报

#define CMD_CLIENT2HOST_TUNNEL_REPORT 0X80  //设备ID上报

typedef struct
{
    UINT32 BaundRate;
    UINT8  ucDataBits;
    UINT8  ucStopBits;
    UINT8  ucParity;
    UINT8  ucIdleCheckThd;
    UINT16 usRsv;
}SERIAL_CONFIG_STRU;


typedef struct
{
    uint16_t usHeatbeatTime;
}HBT_CONFIG_STRU;

#define MAX_DEVICE_SERIAL_NO_LENGTH  APP_SN_LENGTH

#define DEVICE_SERIAL_NO_SIZE        MAX_DEVICE_SERIAL_NO_LENGTH

typedef struct
{
    UINT8  aucSerialNo[DEVICE_SERIAL_NO_SIZE];
}SERIALNO_CONFIG_STRU;

typedef struct
{
    UINT32 language; // late implement
    UINT32 cirtime; // circualtion time (minute)
    UINT32 CELLCONSTANT[3];// 电极常数 ：  0~9.999  (3 decimal)
    UINT32 TEMPCONSTANT[3]; // 温度补偿系数：0~9.999 (3 decimal)

}LOCAL_CONFIG1_STRU;

typedef struct
{
    UINT32 PROD_RES[2];   // 水质设定： 0~18.2MΩ.cm  (1 decimal)
    UINT16 TEMP_MAX[2];   // MAX 55 ℃ 
    UINT16 TEMP_MIN[2];   // MIN  0 ℃ 
    UINT16 PACKLIFEDAY;   //纯化柱:  0~999 DAYS    0~9999L 
    UINT16 PACKLIFEL;   //纯化柱:  0~999 DAYS    0~9999L 
    UINT16 UVLIFEDAY;  // 0~999 DAYS
    UINT16 UVLIFEHOUR; // 0~9999Hr
    UINT16 FILTERLIFE; //过滤器寿命 :0~999DAYS
    UINT16 UNIT;       // MΩ.cm&μs/cm
    UINT16 FLOW;       //流速 0~9.99  (2 decimal)
    UINT16 usRsv;
}LOCAL_CONFIG2_STRU;


typedef struct
{
    // add for v2
    UINT16 usTocWashTime; // UNIT ,second (0~999)
    UINT16 usTocOxiTime;  // UNIT ,second (0~999)
    UINT16 usUFCleanTime; // UNIT ,DAY (30-120)
    UINT16 usUFFlushTime; //   UNIT ,second (0-999)
    UINT16 usAutoUFFlushTime; // UINT ,min(0-999)
    UINT16 usIdleCirPeriod;   // UINT8 ,minute : periodical circulation duration in idle state
    // end for v2

    // following only set by PC
    UINT16 devtype;        // 0 or 1
    UINT16 usRsvd;
	
    UINT8  SERIALNO[16];   //
    // more to add here!
}LOCAL_CONFIG3_STRU;

typedef struct {
	LOCAL_CONFIG1_STRU cfg1;
	LOCAL_CONFIG2_STRU cfg2;
	LOCAL_CONFIG3_STRU cfg3;

}LOCAL_CONFIG_STRU;

enum
{
    DEVICE_TYPE_SERIAL = 0,
    DEVICE_TYPE_LOCAL,
    DEVICE_BUTT,
};

#define ADMIN_TEXT_LENGTH (16)

typedef struct
{
    char NAME[ADMIN_TEXT_LENGTH+1]; // late implement
    char PSD[ADMIN_TEXT_LENGTH+1];

}ADMIN_STRU;

#define PARAMETER_SERIALNO_SIZE      (sizeof(SERIALNO_CONFIG_STRU))
#define PARAMETER_SERIAL_SIZE        (sizeof(SERIAL_CONFIG_STRU))
#define PARAMETER_HBT_SIZE           (sizeof(HBT_CONFIG_STRU))

#define LOCAL_CONFIG_NUM (3)

#define PARAMETER_LOCAL1_OFF (0)
#define PARAMETER_LOCAL2_OFF (sizeof(LOCAL_CONFIG1_STRU))
#define PARAMETER_LOCAL3_OFF (sizeof(LOCAL_CONFIG1_STRU) + sizeof(LOCAL_CONFIG2_STRU))

#define PARAMETER_LOCAL1_SIZE (sizeof(LOCAL_CONFIG1_STRU))
#define PARAMETER_LOCAL2_SIZE (sizeof(LOCAL_CONFIG2_STRU))
#define PARAMETER_LOCAL3_SIZE (sizeof(LOCAL_CONFIG3_STRU))

#define SERIAL_MESSAGE_HEADER_LENGTH (2)
#define SERIAL_MESSAGE_CMD_LENGTH (1)

#define SERIAL_MESSAGE_EXTRA_LENGTH (SERIAL_MESSAGE_HEADER_LENGTH)

//
#define SERIAL_MSG_SIZE(ucPayLoad)(SERIAL_MESSAGE_CMD_LENGTH+ucPayLoad)

#define SERIAL_MSG_BODY_SIZE(ucPayLoad)(SERIAL_MESSAGE_HEADER_LENGTH+ucPayLoad)

#define SERIAL_MSG_TOTAL_SIZE(ucPayLoad)(SERIAL_MESSAGE_EXTRA_LENGTH+ucPayLoad)


#define INVALID_CONFIG_VALUE_SHORT (0XFFFF)

#define INVALID_CONFIG_VALUE_LONG  (0XFFFFFFFF)

typedef enum
{
    BAUD_DATA_8BITS = 0,
    BAUD_DATA_9BITS  ,
    BAUD_DATA_BITS_NUM,

}ENUM_BAUD_DATA_BITS;

typedef enum
{
    BAUD_STOP_1BITS = 0,
    BAUD_STOP_1_5_BITS  ,
    BAUD_STOP_2_BITS ,
    BAUD_STOP_BITS_NUM,

}ENUM_BAUD_STOP_BITS;

typedef enum
{
    BAUD_PARITY_NO = 0,
    BAUD_PARITY_EVEN  ,
    BAUD_PARITY_ODD ,
    BAUD_PARITY_NUM,
}ENUM_BAUD_PARITY;

typedef struct
{
    
    SERIALNO_CONFIG_STRU SerialNoCfg;

}CONFIG_BACKUP_STRU;

extern INT8U * const pParameter ;
extern INT8U * const pRuntimeInfo;

extern SERIALNO_CONFIG_STRU gSerialNoCfg;

extern UINT8 Config_buff[1024];

extern uint8_t aDevType[NV_DTYPE_SIZE];

extern uint16_t CanAddress ;

extern ADMIN_STRU gAdmin;

extern uint16_t gusOperMode;

extern LOCAL_CONFIG_STRU   gLocalCfg;

uint8 Config_SetItem(uint16 id, uint16 len, void *buf);
uint8 Config_GetItem(uint16 id, uint16 len, void *buf);
void Config_Init(void);

int Config_GetSerialNo(uint8_t *pucSerial);
void Config_SaveSerialNo(UINT8 *pData);

void ConfigReset(void);
uint8 Config_Sapp_Entry(uint8 *pCmd,uint8 *pRsp,uint8 CmdLen,uint8 *pucRspLen);

const char *Config_GetVersion(void);
UINT8 Config_SaveData(UINT32 ulAddress,UINT8 *pData,UINT16 usLength);
void Config_Sapp_Cmd(uint8_t cmd,uint8_t *data, uint8_t len);

#pragma pack()

#endif
