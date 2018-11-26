#ifndef _BLE_CMITF_H
#define _BLE_CMITF_H

enum BLE_CMITF_DEVICE_TYPE {
    BLE_CMITF_DEVICE_TYPE_INTELLIGENT_SWITCH = 0,
    BLE_CMITF_DEVICE_TYPE_VIBRATE,
    BLE_CMITF_DEVICE_TYPE_NUM,
};

enum BLE_CMITF_SWITCH_ACTION_ENUM {
    BLE_CMITF_SWITCH_ACTION_OFF = 0,
    BLE_CMITF_SWITCH_ACTION_ON,
    BLE_CMITF_SWITCH_ACTION_NUM,
};

enum BLE_CMITF_CMD_ENUM
{
    BLE_CMITF_CMD_HEART_BEAT = 0,
    BLE_CMITF_CMD_SWITCH_CTL    ,
    BLE_CMITF_CMD_VIBRATOR_CTL  ,
    BLE_CMITF_CMD_PING          ,
    BLE_CMITF_CMD_ENUM          ,
};

typedef struct
{
    uint8 ucLen; /* whole packet length ,include ucLen itself */
    uint8 ucCmd;
    uint8 ucObj;
    uint8 aucData[1];
}BLE_CMDITF_CMD_STRU;

#define BLE_CMDITF_CMD_HEAD_LENGTH (sizeof(BLE_CMDITF_CMD_STRU) - 1)

#endif
