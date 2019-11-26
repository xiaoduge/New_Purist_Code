#include    <ucos_ii.h>

#include    <string.h>
#include    <stdio.h>

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "serial_driver.h"

#include "Serial.h"

#include "relay.h"

#include "modbus.h"

#include "Beep.h"

#include "config.h"

#include "UartCmd.h"

#include "dica.h"

#include "adc.h"

#include "sys.h"

#include "ad7799.h"

#include <math.h>



#define MAX_TADC_VALUE ((1 << (12 + ADC_Additional_Bits)) - 1)
#define RREF_RESISTENCE (10000)

#define MODBUS_ADR_POS (0)
#define MODBUS_CODE_POS (1)
#define MODBUS_CONT_POS (2)

#define MODBUS_HEADER_LEN (2)

#define Start_addr_E  STM32_NV_APP_PARAM_AREA
#define backup_addr_E STM32_NV_APP_PARAM_BACK
#define Start_addr_R  ((uint8_t *)&paramarray[0])
#define dat_len       90


#define T1_ch 0x00
#define T2_ch 0x01
#define T3_ch 0x02



#define C2HR_mode 7
#define C2HH_mode 8
#define C2HL_mode 9
#define C2LH_mode 10
#define C2LL_mode 11
#define C2LR_mode 12

#define C1HR_mode 19
#define C1HH_mode 20
#define C1HL_mode 21
#define C1LH_mode 22
#define C1LL_mode 23
#define C1LR_mode 24

#define C3HR_mode 25
#define C3HH_mode 26
#define C3HL_mode 27
#define C3LH_mode 28
#define C3LL_mode 29
#define C3LR_mode 30

#define meas_mode 31
#define C2RT_mode 33
#define C1RT_mode 35
#define C3RT_mode 36
#define parameter_mode 37
#define backup_mode    38     /* backup */
#define rece_mode      39     /* recover */
#define stop_mode      40
#define save_mode      41

unsigned char RD_mode;                              
#define c2CRD_flg 1                     
#define c1CRD_flg 3                     
#define c3CRD_flg 4                     
#define OR2MEDAT 6                      
#define OR1MEDAT 8                      
#define OR3MEDAT 9                      
#define c0CRD_flg 10                    
#define c0ORD_flg 11                    
#define READSTOP 12                 
#define RDchk_flg 13                


#define RVRH2CAL 24                 
#define RC2HHCAL 25                 
#define RC2HLCAL 26                 
#define RC2LHCAL 27                 
#define RC2LLCAL 28                 
#define RVRL2CAL 29                 
#define RDC2_OFS 30                 
#define RDC2_CON 31                 
#define RC2RTCAL 32                 


#define RVRH1CAL 44
#define RC1HHCAL 45
#define RC1HLCAL 46
#define RC1LHCAL 47
#define RC1LLCAL 48
#define RVRL1CAL 49
#define RDC1_OFS 50
#define RDC1_CON 51
#define RC1RTCAL 52

#define RVRH3CAL 54
#define RC3HHCAL 55
#define RC3HLCAL 56
#define RC3LHCAL 57
#define RC3LLCAL 58
#define RVRL3CAL 59
#define RDC3_OFS 60
#define RDC3_CON 61
#define RC3RTCAL 62

/* add according to Lefeng Brother Little Du */
#define OR2CAL 64                      
#define OR1CAL 66                      
#define OR3CAL 67    

//2019.01.22 Add
#define SAFE
const float maxG25 = 18.2;
const float safePar = 1.05;
//#define B3950



#pragma   pack(1)

struct s_segs {
    unsigned char  aucSeg1[3];
    float          fG25x;
    unsigned short tx;
    unsigned char  aucSeg2[3];
};

typedef struct
{
   struct s_segs s;

}MODBUS_CHL_DATA_STRU;

struct s_debug_segs {
    unsigned char  aucSeg1[3];
    float          fG25x;
    unsigned short vx;
    unsigned short rx;
    unsigned short tx;
    unsigned char  aucSeg2[3];
};

typedef struct
{
   struct s_debug_segs s;

}MODBUS_CHL_DEBUG_STRU;


#pragma   pack()



unsigned char work_mode;
unsigned char cmd[9];

uint8_t ent_flg;
uint8_t Uart1flg;   /* YFL: FLAG for a valid UART FRAME RECEIVED */
uint8_t ES ;

uint8_t ch2_Range_flg;
uint8_t ch1_Range_flg;
uint8_t ch3_Range_flg;

uint8_t debug_range_flg;

uint8_t RDFlag;  

uint16_t value;
float Kcoef,Kp;
uint16_t CRC_Reg;
uint16_t MCRC_Reg;

uint16_t paramarray[90/2];

#define ch1HVR (*((uint16_t *)&paramarray[0]))    
#define ch1HH  (*((uint16_t *)&paramarray[1]))     
#define ch1HL  (*((uint16_t *)&paramarray[2]))     
#define ch1LH  (*((uint16_t *)&paramarray[3]))     
#define ch1LL  (*((uint16_t *)&paramarray[4]))     
#define ch1LVR (*((uint16_t *)&paramarray[5]))    
#define t1offset (*((int16_t *)&paramarray[6]))   
#define  const1 (*((uint16_t *)&paramarray[7]))    

#define  ch2HVR (*((uint16_t *)&paramarray[8]))    
#define  ch2HH (*((uint16_t *)&paramarray[9]))     
#define  ch2HL (*((uint16_t *)&paramarray[10]))     
#define  ch2LH (*((uint16_t *)&paramarray[11]))     
#define  ch2LL (*((uint16_t *)&paramarray[12]))     
#define  ch2LVR (*((uint16_t *)&paramarray[13]))    
#define  t2offset (*((int16_t *)&paramarray[14]))   
#define  const2 (*((uint16_t *)&paramarray[15]))    

#define  ch3HVR (*((uint16_t *)&paramarray[16]))   
#define  ch3HH (*((uint16_t *)&paramarray[17]))     
#define  ch3HL (*((uint16_t *)&paramarray[18]))     
#define  ch3LH (*((uint16_t *)&paramarray[19]))     
#define  ch3LL (*((uint16_t *)&paramarray[20]))     
#define  ch3LVR (*((uint16_t *)&paramarray[21]))  
#define  t3offset (*((uint16_t *)&paramarray[22]))
#define  const3 (*((uint16_t *)&paramarray[23]))  

#define  RT1Ref (*((uint16_t *)&paramarray[24]))
#define  RT2Ref (*((uint16_t *)&paramarray[25]))
#define  RT3Ref (*((uint16_t *)&paramarray[26]))

uint16_t c2addat;
uint16_t c1addat;
uint16_t c3addat;
uint16_t t2addat;
uint16_t t1addat;
uint16_t t3addat;

unsigned char  SendReg[9];

MODBUS_CHL_DATA_STRU  ch2_data;
MODBUS_CHL_DATA_STRU  ch1_data;
MODBUS_CHL_DATA_STRU  ch3_data;

MODBUS_CHL_DEBUG_STRU ch2_dbg_data;
MODBUS_CHL_DEBUG_STRU ch1_dbg_data;
MODBUS_CHL_DEBUG_STRU ch3_dbg_data;

unsigned char  *ch2_dbg_data_addr = (unsigned char *)&ch2_dbg_data;
unsigned char  *ch1_dbg_data_addr = (unsigned char *)&ch1_dbg_data;
unsigned char  *ch3_dbg_data_addr = (unsigned char *)&ch3_dbg_data;

unsigned char  *ch2_data_addr = (unsigned char *)&ch2_data; /*_at_ 0x000b*/;
unsigned char  *ch1_data_addr = (unsigned char *)&ch1_data; /*_at_ 0x0021*/;
unsigned char  *ch3_data_addr = (unsigned char *)&ch3_data; /*_at_ 0x002c*/;

#define   R25x1 ch1_data.s.fG25x                   /* ylf: index into ch1_data_addr */
#define   tx1   ch1_data.s.tx                      /*_at_ 0x0007*/ 

#define   G25x2 ch2_data.s.fG25x                  /* ylf: index into ch1_data_addr */
#define   tx2   ch2_data.s.tx                     /*_at_ 0x0007*/    

#define   R25x3 ch3_data.s.fG25x                   /* ylf: index into ch1_data_addr */
#define   tx3   ch3_data.s.tx                      /*_at_ 0x0007*/   

#define   R25x1_dbg ch1_dbg_data.s.fG25x                   
#define  vx1_dbg    ch1_dbg_data.s.vx                         
#define  rx1_dbg    ch1_dbg_data.s.rx                        
#define  tx1_dbg    ch1_dbg_data.s.tx                        

#define   G25x2_dbg ch2_dbg_data.s.fG25x                  
#define  vx2_dbg    ch2_dbg_data.s.vx                       
#define  rx2_dbg    ch2_dbg_data.s.rx                        
#define  tx2_dbg    ch2_dbg_data.s.tx                        

#define   R25x3_dbg ch3_dbg_data.s.fG25x                   
#define  vx3_dbg    ch3_dbg_data.s.vx                        
#define  rx3_dbg    ch3_dbg_data.s.rx                        
#define  tx3_dbg    ch3_dbg_data.s.tx                        

#if 0
uint16_t RT10k [] = {
44626,42326,40159,38115,36187,34367,32650,  //-6~0
31029,29498,28051,26684,25391,24168,23011,21916,20880,19898,  //1-10
18969,18088,17253,16461,15710,14998,14322,13680,13070,12492,  //11-20
11942,11419,10922,10449,10000,9572,9165,8778,8409,8057, //21-30
7722,7403,7099,6808,6531,6268,6016,5776,5546,5327,  //31-40
5118,4918,4727,4544,4369,4202,4042,3889,3743,3603,  //41-50
3468,3340,3217,3099,2986,2878,2775,2675,2580,2488,  //51-60
2400,2316,2235,2157,2083,2011,1943,1876,1813,1752,  //61-70
1693,1636,1582,1530,1479,1431,1384,1340,1297,1255,  //71-80
1215,1177,1140,1104,1070,1037,1005,974,944,915,     //81-90
888,861,835,810,786,763,741,719,699,679             //90-100
};
#endif

//20190115
uint16_t RT10k [] = {
43450, 41306, 39274, 37349, 35524, 33795, 32116,                        //-6-0
30601, 29128, 27732, 26408, 25152, 23962, 22833, 21762, 20746, 19783,   //1-10
18868, 18000, 17177, 16395, 15652, 14947, 14277, 13641, 13036, 12461,  //11-20
11915, 11395, 10901, 10431, 10000, 9557,  9151,  8765,  8397,  8047,   //21-30
7712,  7394,  7090,  6800,  6523,  6259,  6008,  5767,  5537,  5318,   //31-40
5108,  4907,  4716,  4532,  4357,  4189,  4029,  3875,  3728,  3588,   //41-50
3453,  3324,  3200,  3081,  2968,  2859,  2754,  2654,  2558,  2466,   //51-60
2377,  2293,  2211,  2133,  2058,  1986,  1917,  1850,  1786,  1725,   //61-70
1666,  1610,  1555,  1503,  1452,  1404,  1358,  1313,  1270,  1228,   //71-80
1189,  1150,  1113,  1078,  1044,  1011,  979,   948,   919,   890,    //81-90
863,   837,   811,   787,   763,   740,   718,   697,   676,   657,    //91-100
};


void HEXto_SendReg(uint8_t ch);

#if 0
void setTOffset(int ichl, UINT32 offset)
{
	unsigned short temp = offset/100;
	switch (ichl)
	{
	case 0:
		t1offset = temp;
		break;
	case 1:
		t2offset = temp;
		break;
	case 2:
		t3offset = temp;
		break;
	default:
		break;
	}
}
#endif

uint16_t get_trx(int Rx) 
{
  uint16_t i;
  uint16_t Rx1,Rx2;
  int tx;
  for(i=0;i<107;i++)
  {
    Rx1 = RT10k [i];
    Rx2 = RT10k [i+1];
    if (Rx<=Rx1 && Rx > Rx2)
    {
      tx=((uint16_t)i*10-60+(uint32_t)(Rx1-Rx)*10/(Rx1-Rx2));
      return(tx);
    }
  }
  return (tx = 1001);
}

//old PURIST A+ T Calc
int ex_get_tx(int ADx,uint16_t RTRef)
{
	uint32_t Rtx;
    double tmp;
    int tx;
    Rtx = (long)ADx*RTRef/(MAX_TADC_VALUE-ADx);  
    if(Rtx > 32650)    tmp = 0;    /* ylf: minus 100 celsius */
    else if(Rtx < 679) tmp = 100;  /* ylf: 100 celsius */
	else
	{
#ifdef B3950
		tmp = 3950 / ((3950/298.15) + log(Rtx) - log(10000)) - 273.15;//T Calc
#else
		tmp = 3984 / ((3984/298.15) + log(Rtx) - log(10000)) - 273.15;//T Calc
#endif
	}
	
    tx=(int)(tmp*10);
	
    return(tx);
}

int get_tx(int ADx,uint16_t RTRef)
{
    uint32_t Rtx;
    int tmp;
    int tx;
    Rtx = (long)ADx*RTRef/(MAX_TADC_VALUE-ADx);  
    if(Rtx > 32650)    tmp = -1000; /* ylf: minus 100 celsius */
    else if(Rtx < 679) tmp = 1000;  /* ylf: 100 celsius */
    else 
    {
        tmp = get_trx(Rtx);
        if(tmp > 340) tmp -= 0x04;
        else if(tmp > 330) tmp -= 0x02;
    }
    tx=tmp;
    return(tx);
}
                            
         
void Delay1ms(void)
{
   OSTimeDlyHMSM(0,0,0,1);  
}
void tdelay(uint16_t t)
{
    OSTimeDlyHMSM(0,0,0,t);  
}

typedef void (*modbus_msg_cb)(void);

typedef struct
{
    MsgHead msgHead;
    void *para;
}MODBUS_MSG;

#define MODBUS_MSG_LENGHT (sizeof(MODBUS_MSG)-sizeof(MsgHead))

uint8_t aucModbus_buff[32];

unsigned short calcrc16( unsigned char * buf, int len)
{
    unsigned short crc = 0;
    int i,j;
    for ( j = 0; j < len; j++)
    {
        unsigned char b = buf[j];
        for (i = 0; i < 8; i++)
        {
            crc = ((b ^ (unsigned char)crc) & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
            b >>= 1;
        }
    }
    return crc;
}

UINT8  Modbus_FillSndBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength)
{
    return Serial_FillSndBuf(ucPort + SERIAL_PORT5,pData,usLength);
}

void Modbus_MakeMessage(unsigned char *pModbusMsg,unsigned char ucLength)
{
    unsigned short usCrc;

    usCrc = (unsigned short)calcrc16(pModbusMsg,ucLength); // six bytes for preset single register

    pModbusMsg[ucLength] = (usCrc >> 8) & 0xff;
    pModbusMsg[ucLength+1] = (usCrc >> 0) & 0xff;
}

void modbus_config_cb(uint8_t ucPort)
{
    memset(&Serial[ucPort],0,offsetof(SERIAL_STRU,ucDriverType));

    Serial[ucPort].ucDriverType = MSG_DRIVER;
    Serial[ucPort].ucPortType   = RS232;
    Serial[ucPort].ucPortCtrl   = 0; // DONT CARE FOR RS232


    Serial[ucPort].UsartDef = USART6;
    Serial[ucPort].iIrq     = USART6_IRQn;
    Serial[ucPort].iComIdx  = COM6;
    Serial[ucPort].ucPortCtrl = 0;

    Serial_RetriveConfig(ucPort);

    Serial[ucPort].SerialConfig.BaundRate  = 115200;
    Serial[ucPort].SerialConfig.ucDataBits = BAUD_DATA_8BITS;
    Serial[ucPort].SerialConfig.ucStopBits = BAUD_STOP_1BITS;
    Serial[ucPort].SerialConfig.ucParity   = BAUD_PARITY_NO;

}


void MCRC16(uint8_t *dat_addr,uint8_t start,uint8_t CRC_cnt)
{
    MCRC_Reg = calcrc16(dat_addr+start,CRC_cnt);
}

void CRC16(uint8_t *dat_addr,uint8_t start,uint8_t CRC_cnt)
{
    MCRC_Reg = calcrc16(dat_addr+start,CRC_cnt);
}

uint8_t chk_CRC16(uint8_t start,uint8_t Bcnt)
{
    uint16_t m;
    m=cmd[Bcnt]*256+cmd[Bcnt+1];    
    CRC16(cmd,start,Bcnt);              
    if(m==CRC_Reg) return(1);
    else return(0); 
}

/*
调试确认        C0  03  01  00  00  00  FC  54
校正参数备份    C0  03  02  00  00  00  B8  54
校正参数恢复    C0  03  03  00  00  00  44  55
在线ISP命令     C0  03  04  00  00  00  30  54
保存参数命令    C0  03  06  00  00  00  88  55
校正参数检查    C0  03  07  00  00  00  74  54
ASC MODBUS切换  C0  03  08  00  00  00  60  57
ASC MODBUS切换  C0  03  09  00  00  00  9C  56
指令测试        C0  03  05  00  00  00  CC  55
指令错误        C0  83  01              CC  A0

History: 
    保存参数命令 : save all calibrations to FLash ,add by ylf 
    校正参数检查 : make RDchk_flg complete by adding the command,add by ylf 2018/01/20
*/
void get_c03_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x0100: ent_flg=1;break;
    case 0x0200: work_mode=backup_mode;ent_flg=1;break;
    case 0x0300: work_mode=rece_mode;ent_flg=1;break;
    case 0x0600: work_mode=save_mode;ent_flg=1;break; 
    case 0x0700: RD_mode=RDchk_flg;break; 
    default: Uart1flg=0;break;
    }
}
/*

高量程VR    C1 02 80 81 00 02 38 91
应答        C1 02 02字节  uint_H+uint_L               
                                
高量程高限  C1 02 82 83 00 02 40 31
应答        C1 02 02字节  uint_H+uint_L               
                                
高量程低限  C1 02 84 85 00 02 C9 D1
应答        C1 02 02字节  uint_H+uint_L               
                                
低量程高限  C1 02 86 87 00 02 B1 71
应答        C1 02 02字节  uint_H+uint_L               
                                
低量程低限  C1 02 88 89 00 02 9A 12
应答        C1 02  02字节  uint_H+uint_L               
                               
低量程VR    C1 02 8A 8B 00 02 E2 B2
应答        C1 02 02字节  uint_H+uint_L               
                                
温度采样电阻标定    C1  02  D0  D1  00  02  29  80
应答                C1  02  02字节  uint_H+uint_L               
                                
C1读数据                C1  02  00  03  00  06  13  19
应答数据（float+int）   C1  02  06字节数    EC3-EC0+T1T0                
                                
C1读常数    C1  02  8E  8F  00  02  13  F2
应答        C1  02  02字节数    uint_H+uint_L               
                                
C1读温度偏差补偿值  C1  02  8C  8D  00  02  6B  52
应答                C1  02  02字节数    int_H+int_L             

*/

void get_c12_proc()
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x8081:
        {
            if(work_mode!=C1HR_mode)
            {
                ch1_sw=1;     
                work_mode=C1HR_mode;
            }
            RD_mode=RVRH1CAL; 
            ent_flg = 1;
        }break;         
    case 0x8283:                                
        {
            if(work_mode!=C1HH_mode)
            {
                ch1_sw=1;                       
                work_mode=C1HH_mode;
            }
            RD_mode=RC1HHCAL;               
            ent_flg = 1;
        }break;                                                             
    case 0x8485:                                
        {
            if(work_mode!=C1HL_mode)
            {
                ch1_sw=1; 
                
                work_mode=C1HL_mode;
            }
            RD_mode=RC1HLCAL;               
            ent_flg = 1;
        }break;
    case 0x8687:
        {
            if(work_mode!=C1LH_mode)
            {
                ch1_sw=0;  
                
                work_mode=C1LH_mode;
            }
            RD_mode=RC1LHCAL;               
            ent_flg = 1;
        }break;         
    case 0x8889:                                
        {
            if(work_mode!=C1LL_mode)
            {
                ch1_sw=0;                       
                work_mode=C1LL_mode;
            }
            RD_mode=RC1LLCAL;               
            ent_flg = 1;
        }break;
    case 0x8a8b:                                
        {
            if(work_mode != C1LR_mode)
            {
                ch1_sw=0;                       
                work_mode=C1LR_mode;
            }
            RD_mode=RVRL1CAL;               
            ent_flg = 1;
        }break;
    case 0x8c8d:RD_mode=RDC1_OFS;break;  /* read parameter (0x03e8) */
    case 0x8e8f:RD_mode=RDC1_CON;break;  /* read parameter */
    case 0xd0d1:                         /* set */                             
        {
            if(work_mode != C1RT_mode)
            {
                work_mode = C1RT_mode;
            }
            RD_mode=RC1RTCAL;                               
        }break;
    case 0x0003:RD_mode=OR1MEDAT;break; // ylf:  for temp & water
    case 0x0004:RD_mode=OR1CAL;debug_range_flg = 1;break; // ylf:  for temp & water
    case 0x0005:RD_mode=OR1CAL;debug_range_flg = 0;break; // ylf:  for temp & water
    default:Uart1flg=0;ES=1;break;  /* ylf: invalid frame */
    }
}


/*

写C1温度补偿值  C1  03  8C  8D  int_H   int_L   6A  EE
写C1温度补偿值  C1  03  8C  8D  int_H   int_L   DA  EF
应答            C1  03  8C  8D  int_H   int_L       
                                
写C1电极常数    C1  03  8E  8F  uint_H  uint_L  AC  4E
写C1电极常数    C1  03  8E  8F  03  E7  A8  0E
应答            C1  03  8E  8F  uint_H  uint_L      

*/
void get_c13_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x8c8d: /* set 温度补偿值 */
        {
            t1offset=cmd[4]*256+cmd[5];
            work_mode=parameter_mode;       
            ent_flg=1;                                  
        }break;
    case 0x8e8f: /* set 电极常数 */
        {
            const1=cmd[4]*256+cmd[5];       
            work_mode=parameter_mode;       
            ent_flg=1;
        }break;
    default:Uart1flg=0;ES=1;break;
    }
}

/*
高量程VR    C2  02  90  91  00  02  0E  94
应答    C2  02  02字节  uint_H+uint_L               
                                
高量程高限  C2  02  92  93  00  02  76  34
应答    C2  02  02字节  uint_H+uint_L               
                                
高量程底限  C2  02  94  95  00  02  FF  D4
应答    C2  02  02字节  uint_H+uint_L               
                                
低量程高限  C2  02  96  97  00  02  87  74
应答    C2  02  02字节  uint_H+uint_L               
                                
低量程底限  C2  02  98  99  00  02  AC  17
应答    C2  02  02字节  uint_H+uint_L               
                                
低量程VR标定    C2  02  9A  9B  00  02  D4  B7
应答    C2  02  02字节  uint_H+uint_L               
                                
温度采样电阻标定    C2  02  D2  D3  00  02  62  20
应答    C2  02  02字节  uint_H+uint_L               
                                
C2读数据    C2  02  00  0E  00  06  E3  88
应答数据（float+int）   C2  02  06字节数    EC3-EC0+T1T0                
                                
C2读常数    C2  02  9E  9F  00  02  25  F7
应答    C2  02  02字节数    uint_H+uint_L               
                                
C2读温度偏差补偿值  C2  02  9C  9D  00  02  5D  57
应答    C2  02  02字节数    int_H+int_L             

*/
void get_c22_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0x9091:
            {
                if(work_mode!=C2HR_mode)        
                {
                    ch2_sw=1;    
                    work_mode = C2HR_mode;
                }
                RD_mode=RVRH2CAL;                       
                ent_flg = 1;
            }break;
        case 0x9293:                                        
            {
                if(work_mode!=C2HH_mode)        
                {
                    ch2_sw=1;                               
                    work_mode=C2HH_mode;
                }
                RD_mode=RC2HHCAL;                       
                ent_flg = 1;
            }break;
        case 0x9495:                                        
            {
                if(work_mode!=C2HL_mode)        
                {
                    ch2_sw=1;                               
                    work_mode=C2HL_mode;
                }
                RD_mode=RC2HLCAL;                       
                ent_flg = 1;
            }break;
        case 0x9697:                                        
            {
                if(work_mode!=C2LH_mode)        
                {
                    ch2_sw=0;                               
                    work_mode=C2LH_mode;
                }
                RD_mode=RC2LHCAL;                       
                ent_flg = 1;
            }break;
        case 0x9899:                                        
            {
                if(work_mode!=C2LL_mode)        
                {
                    ch2_sw=0;                               
                    work_mode=C2LL_mode;
                }
                RD_mode=RC2LLCAL;                       
                ent_flg = 1;
            }break;
        case 0x9a9b:                                        
            {
                if(work_mode!=C2LR_mode)        
                {
                    ch2_sw=0;                               
                    work_mode=C2LR_mode;
                }
                RD_mode=RVRL2CAL;                       
                ent_flg = 1;
            }break;
        case 0x9c9d:RD_mode=RDC2_OFS;break;
        case 0x9e9f:RD_mode=RDC2_CON;break;
        case 0xd2d3:                                                
            {
                if(work_mode!=C2RT_mode)
                {
                    work_mode=C2RT_mode;
                }
                RD_mode=RC2RTCAL;                               
                ent_flg = 1;
            }break;
        case 0x000e:RD_mode=OR2MEDAT;break; 
        case 0x000f:RD_mode=OR2CAL;debug_range_flg = 1; break; 
        case 0x0010:RD_mode=OR2CAL;debug_range_flg = 0; break; 
        default:Uart1flg=0;ES=1;break;
    }
}

/*

写C2温度补偿值  C2  03  9C  9D  int_H   int_L   5C  EB
写C2温度补偿值  C2  03  9C  9D  int_H   int_L       
应答            C2  03  9C  9D  int_H   int_L       
                                
写C2电极常数    C2  03  9E  9F  uint_H  uint_L  9A  4B
写C2电极常数    C2  03  9E  9F  03  E7  9E  0B
应答            C2  03  9E  9F  uint_H  uint_L      

*/
void get_c23_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x9c9d:
        {
            t2offset=cmd[4]*256+cmd[5];         
            work_mode=parameter_mode;               
            ent_flg=1;                                          
        }break;
    case 0x9e9f:
        {
            const2=cmd[4]*256+cmd[5];               
            work_mode=parameter_mode;               
            ent_flg=1;
        }break;
    default:Uart1flg=0;ES=1;break;
    }
}

/*
高量程VR    C3  02  A0  A1  00  02  D0  9A
应答    C3  02  02字节  uint_H+uint_L               
                                
高量程高限  C3  02  A2  A3  00  02  A8  3A
应答    C3  02  02字节  uint_H+uint_L               
                                
高量程底限  C3  02  A4  A5  00  02  21  DA
应答    C3  02  02字节  uint_H+uint_L               
                                
低量程高限  C3  02  A6  A7  00  02  59  7A
应答    C3  02  02字节  uint_H+uint_L               
                                
低量程底限  C3  02  A8  A9  00  02  72  19
应答    C3  02  02字节  uint_H+uint_L               
                                
低量程VR标定    C3  02  AA  AB  00  02  0A  B9
应答    C3  02  02字节  uint_H+uint_L               
                                
温度采样电阻标定    C3  02  D4  D5  00  02  3A  C1
应答    C3  02  02字节  uint_H+uint_L               
                                
C3读数据    C3  02  00  19  00  06  36  39
应答数据（float+int）   C3  02  06字节数    EC3-EC0+T1T0                
                                
C3读常数    C3  02  AE  AF  00  02  FB  F9
应答    C3  02  02字节数    uint_H+uint_L               
                                
C3读温度偏差补偿值  C3  02  AC  AD  00  02  83  59
应答    C3  02  02字节数    int_H+int_L             

*/

void get_c32_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xa0a1:                                                
            {
                if(work_mode!=C3HR_mode)                
                {
                    ch3_sw=1;                                       
                    work_mode=C3HR_mode;
                }
                RD_mode=RVRH3CAL;                               
                ent_flg = 1;
            }break;         
        case 0xa2a3:                                                
            {
                if(work_mode!=C3HH_mode)                
                {
                    ch3_sw=1;                                       
                    work_mode=C3HH_mode;
                }
                RD_mode=RC3HHCAL;                                   
                ent_flg = 1;
            }break;                                                             
        case 0xa4a5:                                                    
            {
                if(work_mode!=C3HL_mode)                    
                {
                    ch3_sw=1;                                           
                    work_mode=C3HL_mode;
                }
                RD_mode=RC3HLCAL;                                   
                ent_flg = 1;
            }break;
        case 0xa6a7:                                                    
            {
                if(work_mode!=C3LH_mode)                    
                {
                    ch3_sw=0;                                           
                    work_mode=C3LH_mode;
                }
                RD_mode=RC3LHCAL;                                   
                ent_flg = 1;
            }break;         
        case 0xa8a9:                                                    
            {
                if(work_mode!=C3LL_mode)                    
                {
                    ch3_sw=0;                                           
                    work_mode=C3LL_mode;
                }
                RD_mode=RC3LLCAL;                                   
                ent_flg = 1;
            }break;
        case 0xaaab:                                                    
            {
                if(work_mode!=C3LR_mode)                    
                {
                    ch3_sw=0;                                           
                    work_mode=C3LR_mode;
                }
                RD_mode=RVRL3CAL;                                   
                ent_flg = 1;
            }break;
        case 0xacad:RD_mode=RDC3_OFS;break;     
        case 0xaeaf:RD_mode=RDC3_CON;break;     
        case 0xd4d5:                                                    
            {
                if(work_mode!=C3RT_mode)
                {
                    work_mode=C3RT_mode;
                }
                RD_mode=RC3RTCAL;                                   
            }break;
        case 0x0019:RD_mode=OR3MEDAT;break;     
        case 0x001a:RD_mode=OR3CAL;debug_range_flg = 1;break;     
        case 0x001b:RD_mode=OR3CAL;debug_range_flg = 0;break;     
        default:Uart1flg=0;ES=1;break;
    }
}


/*

写C3温度补偿值  C3  03  AC  AD  int_H   int_L   82  E5
写C3温度补偿值  C3  03  AC  AD  int_H   int_L       
应答            C3  03  AC  AD  int_H   int_L       
                                
写C3电极常数    C3  03  AE  AF  uint_H  uint_L   
写C3电极常数    C3  03  AE  AF  03  E8  44  45
应答            C3  03  AE  AF  uint_H  uint_L      

*/

void get_c33_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xacad:
            {
                t3offset=cmd[4]*256+cmd[5];         
                work_mode=parameter_mode;               
                ent_flg=1;                                          
            }break;
        case 0xaeaf:
            {
                const3=cmd[4]*256+cmd[5];               
                work_mode=parameter_mode;               
                ent_flg=1;
            }break;
        default:Uart1flg=0;ES=1;break;
    }
}


void Tx_data(uint8_t *dat_addr,uint8_t cnt) 
{
    Modbus_FillSndBuf(0,dat_addr,cnt);
}

uint16_t get_RtnRef(int ADx)
{
    uint16_t m;
    m=(uint32_t)(MAX_TADC_VALUE-ADx)*RREF_RESISTENCE/ADx; /* ylf: 0x2710 = 10000 */
    return(m);
}


uint16_t get_c1addat()
{
    return (AD7799_ReadAdc(0) >> 8);
}

uint16_t get_c2addat()
{
    return (AD7799_ReadAdc(1) >> 8);
}

uint16_t get_c3addat()
{
    return (AD7799_ReadAdc(2) >> 8);
}

uint16_t getRx_proc(uint16_t ADx,uint16_t RRef,uint16_t VR) 
{
    uint16_t Rx;
    /* ylf: simplify circuit to : power source---Rfix----Rx---GND */
    Rx = (10000*(long)(VR-RRef))/RRef;  
    Rx = ((long)ADx*Rx)/(VR-ADx); 
    return(Rx);
}


void get_Kcoef_Kp(float t)
{
    if(t<20)
    {
        Kcoef=0.00079f*t*t-0.0539f*t+1.873f;
        Kp=0.365e-4f*t*t+0.775e-3f*t+0.0119f;
    }
    else if(t<40)
    {
        Kcoef=0.0000125f*t*t-0.02705f*t+1.602f;
        Kp=0.59e-4f*t*t-0.9e-4f*t+0.0202f;
    }
    else
    {
        Kcoef=-0.000065f*t*t-0.00215f*t+0.91f;
        Kp=0.849e-4f*t*t-0.205e-2f*t+0.057f;
    }
}


void Tx_data_init()
{
    ch1_data_addr[0]=0xc1;
    ch1_data_addr[1]=0x02;
    ch1_data_addr[2]=0x06;

    ch2_data_addr[0]=0xc2;
    ch2_data_addr[1]=0x02;
    ch2_data_addr[2]=0x06;

    ch3_data_addr[0]=0xc3;
    ch3_data_addr[1]=0x02;
    ch3_data_addr[2]=0x06;

    ch1_dbg_data_addr[0]=0xc1;
    ch1_dbg_data_addr[1]=0x03;
    ch1_dbg_data_addr[2]=0x0A;
	
    ch2_dbg_data_addr[0]=0xc2;
    ch2_dbg_data_addr[1]=0x03;
    ch2_dbg_data_addr[2]=0x0A;
    
    ch3_dbg_data_addr[0]=0xc3;
    ch3_dbg_data_addr[1]=0x03;
    ch3_dbg_data_addr[2]=0x0A;
    
}

int ad_mean(uint8_t ch)   
{
   return  GetAdcData(ch);
}
void ch1_meas_proc_Inner(float *c, float *t, uint16_t *Rt)
{
    uint16_t Rx;
    t1addat=ad_mean(T1_ch);
    c1addat=get_c1addat();	
    //tx1=get_tx(t1addat,RT1Ref)+t1offset;
	tx1= ex_get_tx(t1addat,RT1Ref) + t1offset;
	
    get_Kcoef_Kp((float)tx1/10);
    if(ch1_Range_flg)
    {
        Rx=getRx_proc(c1addat,ch1HH,ch1HVR); /* satisfy : ch3HH(10k) < ch3HL < ch3HVR */
        R25x1=1/(Kcoef*(1e7f/((float)Rx*(float)const1)-Kp)+0.05482f);
        if(c1addat>(ch1HL+100))
        {
            ch1_sw=0;
            ch1_Range_flg=0;
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c1addat,ch1LL,ch1LVR);
        R25x1=1/(Kcoef*(1e6f/((float)Rx*(float)const1)-Kp)+0.05482f);
        if(c1addat<(ch1LH-100))
        {
            ch1_sw=1;
            ch1_Range_flg=1;
            tdelay(200);
        }
    }
	
#ifdef SAFE
	R25x1 *= safePar;
	if(R25x1 > maxG25)
	{
		R25x1 = 18.2;
	}
#endif

    if (c) *c = R25x1;
    if (t) *t = tx1;
	if (Rt) *Rt = (long)t1addat*RT1Ref/(MAX_TADC_VALUE-t1addat);
    
}

void ch2_meas_proc_Inner(float *c,float *t)
{
    uint16_t Rx;
   // t2addat=ad_mean(T2_ch);
    c2addat=get_c2addat();  
   // tx2=get_tx(t2addat,RT2Ref)+t2offset; 
	tx2 = tx1;
    if(ch2_Range_flg)                                                   
    {
        Rx=getRx_proc(c2addat,ch2HH,ch2HVR);    /* satisfy : ch2HH(10k) < ch2HL < ch2HVR */
        G25x2=(float)const2/1000/(float)Rx*1E6f/(1+0.02f*((float)tx2/10-25));
        if(c2addat>(ch2HL+100))
        {
            ch2_sw=0;  
            ch2_Range_flg=0;                                
            tdelay(200);                                        
        }
    }
    else                                                                        
    {
        Rx=getRx_proc(c2addat,ch2LL,ch2LVR);/* satisfy : ch2LH < ch2LL(100k) < ch2LVR */
        G25x2=(float)const2/1000/(float)Rx*1E5f/(1+0.02f*((float)tx2/10-25));
        if(c2addat<(ch2LH-100))
        {
            ch2_sw=1;
            ch2_Range_flg=1;    
            tdelay(200);            
        }
    }
    
    if (c) *c = G25x2;
    if (t) *t = tx2;

}


void ch3_meas_proc_Inner(float *c,float *t)
{
    uint16_t Rx;
    //t3addat=ad_mean(T3_ch);
    c3addat=get_c3addat();
    //tx3=get_tx(t3addat,RT3Ref)+t3offset;
	tx3 = tx1;
    get_Kcoef_Kp((float)tx3/10);
    if(ch3_Range_flg)
    {
        Rx=getRx_proc(c3addat,ch3HH,ch3HVR);/* satisfy : ch3HH(10k) < ch3HL < ch3HVR */
        R25x3=1/(Kcoef*(1e7f/((float)Rx*(float)const3)-Kp)+0.05482f);
        if(c3addat>(ch3HL+100))
        {
            ch3_sw=0;
            ch3_Range_flg=0;
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c3addat,ch3LL,ch3LVR);
        R25x3=1/(Kcoef*(1e6f/((float)Rx*(float)const3)-Kp)+0.05482f);
        if(c3addat<(ch3LH-100))
        {
            ch3_sw=1;
            ch3_Range_flg=1;
            tdelay(200);
        }
    }

    if (c) *c = R25x3;
    if (t) *t = tx3;
}

void ch1_meas_proc()
{
    ch1_meas_proc_Inner(NULL,NULL,NULL);
}


void ch2_meas_proc()
{
    ch2_meas_proc_Inner(NULL,NULL);
}

void ch3_meas_proc()
{
    ch3_meas_proc_Inner(NULL,NULL);
}


void c2_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch2_dbg_data_addr,0,13);
    ch2_dbg_data_addr[13]=MCRC_Reg>>8;
    ch2_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch2_dbg_data_addr,15);  

}

void ch2_debug_proc()
{
    uint16_t Rx;
    
    t2addat=ad_mean(T2_ch);
    c2addat=get_c2addat(); 
    
    tx2_dbg=get_tx(t2addat,RT2Ref)+t2offset;        
    vx2_dbg=c2addat;

    if(debug_range_flg)                                                   
    {
        Rx=getRx_proc(c2addat,ch2HH,ch2HVR);    
        G25x2_dbg=(float)const2/1000/(float)Rx*1E6f/(1+0.02f*((float)tx2_dbg/10-25));
    }
    else                                                                        
    {
        Rx=getRx_proc(c2addat,ch2LL,ch2LVR);
        G25x2_dbg=(float)const2/1000/(float)Rx*1E5f/(1+0.02f*((float)tx2_dbg/10-25));
    }
    rx2_dbg = Rx;
}



void c1_puts()
{
    {
        /* switch to big endian */
        unsigned char ucTemp;
        
        ucTemp = ch1_data_addr[3];
        ch1_data_addr[3] = ch1_data_addr[3 + 3];
        ch1_data_addr[3 + 3] = ucTemp;

        ucTemp = ch1_data_addr[4];
        ch1_data_addr[4] = ch1_data_addr[5];
        ch1_data_addr[5] = ucTemp;

        ucTemp = ch1_data_addr[7];
        ch1_data_addr[7] = ch1_data_addr[8];
        ch1_data_addr[8] = ucTemp;
        
        MCRC16(ch1_data_addr,0,9);
        ch1_data_addr[9]=MCRC_Reg>>8;
        ch1_data_addr[10]=MCRC_Reg;
        Tx_data(ch1_data_addr,11);  
    }
    Uart1flg=0;ES=1;
}


void c1_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch1_dbg_data_addr,0,13);
    ch1_dbg_data_addr[13]=MCRC_Reg>>8;
    ch1_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch1_dbg_data_addr,15);  

}


void c2_puts()
{
    {
        /* switch to big endian */
        unsigned char ucTemp;
        
        ucTemp = ch2_data_addr[3];
        ch2_data_addr[3] = ch2_data_addr[3 + 3];
        ch2_data_addr[3 + 3] = ucTemp;

        ucTemp = ch2_data_addr[4];
        ch2_data_addr[4] = ch2_data_addr[5];
        ch2_data_addr[5] = ucTemp;

        ucTemp = ch2_data_addr[7];
        ch2_data_addr[7] = ch2_data_addr[8];
        ch2_data_addr[8] = ucTemp;
    
        MCRC16(ch2_data_addr,0,9);
        ch2_data_addr[9]=MCRC_Reg>>8;
        ch2_data_addr[10]=MCRC_Reg;
        Tx_data(ch2_data_addr,11);
    }
    Uart1flg=0;ES=1;
}


void c3_puts()
{
    {
        unsigned char ucTemp;
        
        ucTemp = ch3_data_addr[3];
        ch3_data_addr[3] = ch3_data_addr[3 + 3];
        ch3_data_addr[3 + 3] = ucTemp;

        ucTemp = ch3_data_addr[4];
        ch3_data_addr[4] = ch3_data_addr[5];
        ch3_data_addr[5] = ucTemp;

        ucTemp = ch3_data_addr[7];
        ch3_data_addr[7] = ch3_data_addr[8];
        ch3_data_addr[8] = ucTemp;   
        
        MCRC16(ch3_data_addr,0,9);
        ch3_data_addr[9]=MCRC_Reg>>8;
        ch3_data_addr[10]=MCRC_Reg;
        Tx_data(ch3_data_addr,11);
    }
    Uart1flg=0;ES=1;
}

void ch1_debug_proc()
{
    uint16_t Rx;
    t1addat=ad_mean(T1_ch);
    c1addat=get_c1addat();
    tx1_dbg=get_tx(t1addat,RT1Ref)+t1offset;
    get_Kcoef_Kp(((float)tx1_dbg)/10);
    vx1_dbg=c1addat;

    if(debug_range_flg)
    {
        Rx=getRx_proc(c1addat,ch1HH,ch1HVR);
        R25x1_dbg=1/(Kcoef*(1e7f/((float)Rx*(float)const1)-Kp)+0.05482f);
    }
    else
    {
        Rx=getRx_proc(c1addat,ch1LL,ch1LVR);
        R25x1_dbg=1/(Kcoef*(1e6f/((float)Rx*(float)const1)-Kp)+0.05482f);
    }
    rx1_dbg = Rx;
}

void c3_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch3_dbg_data_addr,0,13);
    ch3_dbg_data_addr[13]=MCRC_Reg>>8;
    ch3_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch3_dbg_data_addr,15);  

}

void ch3_debug_proc()
{
    uint16_t Rx;
    t3addat=ad_mean(T3_ch);
    c3addat=get_c3addat();
    tx3_dbg=get_tx(t3addat,RT3Ref)+t3offset;
    get_Kcoef_Kp(((float)tx3_dbg)/10);
    vx3_dbg=c3addat;

    if(debug_range_flg)
    {
        Rx=getRx_proc(c3addat,ch3HH,ch3HVR);
        R25x3_dbg=1/(Kcoef*(1e7f/((float)Rx*(float)const3)-Kp)+0.05482f);
    }
    else
    {
        Rx=getRx_proc(c3addat,ch3LL,ch3LVR);
        R25x3_dbg=1/(Kcoef*(1e6f/((float)Rx*(float)const3)-Kp)+0.05482f);
    }
    rx3_dbg = Rx;
}


void IapProgramMoreByte(uint16_t addr,uint8_t *Mcu_addr,uint8_t len)
{
     Config_SetItem(addr,len,Mcu_addr); 
}

void backup_proc(void)
{
    Config_GetItem(STM32_NV_APP_PARAM_AREA,dat_len,Start_addr_R);

    Config_SetItem(STM32_NV_APP_PARAM_BACK,dat_len,Start_addr_R);
}

void rece_proc(void)
{
    if (ERROR_SUCCESS == Config_GetItem(backup_addr_E,dat_len,Start_addr_R))
    {
        Config_SetItem(Start_addr_E,dat_len,Start_addr_R);
    }
}


void Send_Cal(uint8_t No)
{

    switch(No)
    {
        case 1:HEXto_SendReg(1);break;  /* for channel 1 */
        case 2:HEXto_SendReg(1);break;
        case 3:HEXto_SendReg(1);break;
        case 4:HEXto_SendReg(1);break;
        case 5:HEXto_SendReg(1);break;
        case 6:HEXto_SendReg(1);break;
        case 7:HEXto_SendReg(2);break; /* for channel 2 */
        case 8:HEXto_SendReg(2);break;  
        case 9:HEXto_SendReg(2);break;  
        case 10:HEXto_SendReg(2);break;
        case 11:HEXto_SendReg(2);break;
        case 12:HEXto_SendReg(2);break;
        case 13:HEXto_SendReg(3);break; /* for channel 3 */
        case 14:HEXto_SendReg(3);break;
        case 15:HEXto_SendReg(3);break;
        case 16:HEXto_SendReg(3);break;
        case 17:HEXto_SendReg(3);break;
        case 18:HEXto_SendReg(3);break;

        case 31:HEXto_SendReg(6);break;
        case 32:HEXto_SendReg(7);break;
        case 33:HEXto_SendReg(8);break;
        
        case 41:HEXto_SendReg(9);break;
        case 42:HEXto_SendReg(11);break;
        case 43:HEXto_SendReg(13);break;
        
        case 51:HEXto_SendReg(10);break;
        case 52:HEXto_SendReg(12);break;
        case 53:HEXto_SendReg(14);break;
    }
    Tx_data(SendReg,7);
    Uart1flg=0;ES=1;
}


void HEXto_SendReg(uint8_t ch)                        
{
    uint16_t tmp;
    switch(ch)
    {
        case  1:SendReg[0]=0xC1;tmp=c1addat;break;   
        case  2:SendReg[0]=0xC2;tmp=c2addat;break;
        case  3:SendReg[0]=0xC3;tmp=c3addat;break;
        case  6:SendReg[0]=0xC1;RT1Ref=get_RtnRef(t1addat);tmp=RT1Ref;break; /* ylf: first get then send ,maybe later save to e2prom */
        case  7:SendReg[0]=0xC2;RT2Ref=get_RtnRef(t2addat);tmp=RT2Ref;break;
        case  8:SendReg[0]=0xC3;RT3Ref=get_RtnRef(t3addat);tmp=RT3Ref;break;
        case 9:SendReg[0]=0xC1;tmp=const1;break;
        case 10:SendReg[0]=0xC1;tmp=t1offset;break;
        case 11:SendReg[0]=0xC2;tmp=const2;break;
        case 12:SendReg[0]=0xC2;tmp=t2offset;break;
        case 13:SendReg[0]=0xC3;tmp=const3;break;
        case 14:SendReg[0]=0xC3;tmp=t3offset;break;
    }
    SendReg[1]=0x02;
    SendReg[2]=0x02;
    SendReg[3]=tmp>>8;
    SendReg[4]=tmp;
    MCRC16(SendReg,0,5);            
    SendReg[5]=MCRC_Reg>>8;
    SendReg[6]=MCRC_Reg;
}


void Modbus_MsgHandler(uint8_t *pData, uint8_t ucLen)
{
    MainAlarmWithDuration(1);
    memcpy(cmd,pData,ucLen);
    
    switch(cmd[0]*256+cmd[1])
    {
    case 0xc003:
        {
            if((cmd[4]*256+cmd[5])==0)
            {
                if(chk_CRC16(0,6))  
                {
                    ES=0;
                    Uart1flg=1;         
                    get_c03_proc(); 
                }
            }
            else return;
        }break;
    case 0xc102: /* get wq & temperature measurement result */
        {
            if((cmd[4]*256+cmd[5])==0x0002
               ||(cmd[4]*256+cmd[5])==0x0006
               ||(cmd[4]*256+cmd[5])==0x000A) /* ylf: we go 0006 */
            {
                if(chk_CRC16(0,6))      
                {
                    ES=0;Uart1flg=1;    
                    get_c12_proc();     
                }
            }
        }break;
    case 0xc103: 
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c13_proc();     
            }
        }break;
    case 0xc202:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))  
                {
                    ES=0;Uart1flg=1;
                    get_c22_proc(); 
                }
            }
        }break;
    case 0xc203:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c23_proc();     
            }
        }break;
    case 0xc302:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))      
                {
                    ES=0;Uart1flg=1;    
                    get_c32_proc(); 
                }
            }
        }break;
    case 0xc303:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c33_proc();     
            }
        }break;
    }

    switch(RD_mode)
    {
    case RVRH1CAL: c1addat=get_c1addat();Send_Cal(1);RD_mode=READSTOP;break;
    case RC1HHCAL: c1addat=get_c1addat();Send_Cal(2);RD_mode=READSTOP;break;
    case RC1HLCAL: c1addat=get_c1addat();Send_Cal(3);RD_mode=READSTOP;break;
    case RC1LHCAL: c1addat=get_c1addat();Send_Cal(4);RD_mode=READSTOP;break;
    case RC1LLCAL: c1addat=get_c1addat();Send_Cal(5);RD_mode=READSTOP;break;
    case RVRL1CAL: c1addat=get_c1addat();Send_Cal(6);RD_mode=READSTOP;break;
    case RC1RTCAL: t1addat=ad_mean(T1_ch);Send_Cal(31);RD_mode=READSTOP;break;
    case RDC1_CON: Send_Cal(41);RD_mode=READSTOP;break;
    case RDC1_OFS: Send_Cal(51);RD_mode=READSTOP;break;

    case RVRH2CAL: c2addat=get_c2addat();Send_Cal(7);RD_mode=READSTOP;break;  
    case RC2HHCAL: c2addat=get_c2addat();Send_Cal(8);RD_mode=READSTOP;break;  
    case RC2HLCAL: c2addat=get_c2addat();Send_Cal(9);RD_mode=READSTOP;break;  
    case RC2LHCAL: c2addat=get_c2addat();Send_Cal(10);RD_mode=READSTOP;break;  
    case RC2LLCAL: c2addat=get_c2addat();Send_Cal(11);RD_mode=READSTOP;break;  
    case RVRL2CAL: c2addat=get_c2addat();Send_Cal(12);RD_mode=READSTOP;break;  
    case RC2RTCAL: t2addat=ad_mean(T2_ch);Send_Cal(32);RD_mode=READSTOP;break;
    case RDC2_CON: Send_Cal(42);RD_mode=READSTOP;break;
    case RDC2_OFS: Send_Cal(52);RD_mode=READSTOP;break;

    case RVRH3CAL: c3addat=get_c3addat();Send_Cal(13);RD_mode=READSTOP;break;
    case RC3HHCAL: c3addat=get_c3addat();Send_Cal(14);RD_mode=READSTOP;break;
    case RC3HLCAL: c3addat=get_c3addat();Send_Cal(15);RD_mode=READSTOP;break;
    case RC3LHCAL: c3addat=get_c3addat();Send_Cal(16);RD_mode=READSTOP;break;
    case RC3LLCAL: c3addat=get_c3addat();Send_Cal(17);RD_mode=READSTOP;break;
    case RVRL3CAL: c3addat=get_c3addat();Send_Cal(18);RD_mode=READSTOP;break;
    case RC3RTCAL: t3addat=ad_mean(T3_ch);Send_Cal(33);RD_mode=READSTOP;break;
    case RDC3_CON: Send_Cal(43);RD_mode=READSTOP;break;
    case RDC3_OFS: Send_Cal(53);RD_mode=READSTOP;break;
    case OR1MEDAT: ch1_meas_proc();c1_puts();RD_mode=READSTOP;break;
    case OR2MEDAT: ch2_meas_proc();c2_puts();RD_mode=READSTOP;break;
    case OR3MEDAT: ch3_meas_proc();c3_puts();RD_mode=READSTOP;break;
    case OR1CAL: ch1_debug_proc();c1_debug_puts();RD_mode=READSTOP;break;
    case OR2CAL: ch2_debug_proc();c2_debug_puts();RD_mode=READSTOP;break;
    case OR3CAL: ch3_debug_proc();c3_debug_puts();RD_mode=READSTOP;break;
    case READSTOP: break;
    }
    
    if(ent_flg)
    {
        switch(work_mode)
        {
        case C1HR_mode:
            {
                ch1HVR=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1HVR:%d \n", ch1HVR);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1HH_mode:
            {
                ch1HH=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1HH:%d \n", ch1HH);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1HL_mode:
            {
                ch1HL=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1HL:%d \n", ch1HL);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LH_mode:
            {
                ch1LH=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1LH:%d \n", ch1LH);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LL_mode:
            {
                ch1LL=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1LL:%d \n", ch1LL);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LR_mode:
            {
                ch1LVR=c1addat;
				//UartCmdPrintf(VOS_LOG_DEBUG,"ch1LVR:%d \n", ch1LVR);
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
                
        case C2HR_mode:
            {
                ch2HVR=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2HH_mode:
            {
                ch2HH=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2HL_mode:
            {
                ch2HL=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LH_mode:
            {
                ch2LH=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LL_mode:
            {
                ch2LL=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LR_mode:
            {
                ch2LVR=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3HR_mode:
            {
                ch3HVR=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3HH_mode:
            {
                ch3HH=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3HL_mode:
            {
                ch3HL=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LH_mode:
            {
                ch3LH=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LL_mode:
            {
                ch3LL=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LR_mode:
            {
                ch3LVR=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;

        case C1RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;

        case C2RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;

        case C3RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case parameter_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case backup_mode:       
            {
                backup_proc();
            }break;
        case rece_mode:         
            {
                rece_proc();
            }break;
        case save_mode:
            {
                IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);

                work_mode = meas_mode; /* prevent repeatly saving */
            }
            break;
        }
        Uart1flg=0;
        ES = 1;                       /* ready for the next uart frame */
        ent_flg=0;
    }

}

void modbus_ItfProcess(Message *pMsg)
{
    

    // first parse message
    // 1. check modbus address
    UINT8   ucCnt = pMsg->msgHead.nMsgLen;
    UINT16  usCrc;
    // UINT8  *pModbus = (UINT8 *)pMsg->data;

    // VOS_LOG(VOS_LOG_DEBUG, "CNT%x,%x,%x,%x,%x,%x,%x,%x,%x",ucCnt,pMsg->data[0],pMsg->data[1],pMsg->data[2],pMsg->data[3],pMsg->data[4],pMsg->data[5],pMsg->data[6],pMsg->data[7]);

	MainAlarmWithDuration(1);

    // check crc
    if (ucCnt != 8)
    {
        return;
    }

    ucCnt -= 2;
    
    usCrc = calcrc16((UINT8 *)pMsg->data,ucCnt);

    if (((usCrc >> 8 ) & 0xff) != (uint8_t)pMsg->data[ucCnt]
        || ((usCrc >> 0 ) & 0xff) != (uint8_t)pMsg->data[ucCnt+1])
    {
        VOS_LOG(VOS_LOG_DEBUG, "mitf crc fail %x,%x",usCrc,(UINT16)(pMsg->data[ucCnt]<<8)|pMsg->data[ucCnt+1]);
        return;
    }

    Modbus_MsgHandler((uint8_t *)pMsg->data,ucCnt);
    
}

void modbus_msg_Handler(Message *Msg)
{
    MODBUS_MSG *dmsg = (MODBUS_MSG *)Msg;

    if (dmsg->para)
    {
        ((modbus_msg_cb)dmsg->para)();
    }
}


void ModbusInit(void)
{
   uint8_t ucPortIdx = SERIAL_PORT5;
   
   work_mode = meas_mode;
   RD_mode   = READSTOP;

   ent_flg   = 0;
   Uart1flg  = 0;   /* YFL: FLAG for a valid UART FRAME RECEIVED */
   ES = 0;
   
   //2018.11.21 : 1->0
   ch2_Range_flg=1;
   ch1_Range_flg=1;
   ch3_Range_flg=1;
   
   ch1_sw = 1;
   ch2_sw = 1;
   ch3_sw = 1;

   if (ERROR_SUCCESS != Config_GetItem(STM32_NV_APP_PARAM_AREA,dat_len,Start_addr_R))
   {
       memset(Start_addr_R,0,dat_len);
   }

   Tx_data_init();   

   {
       Serial[ucPortIdx].sccb = modbus_config_cb;
   
       Serial[ucPortIdx].mcb  = modbus_ItfProcess;
   
       SerialInitPort(ucPortIdx);   
   }

   
	//VOS_LOG(VOS_LOG_DEBUG,"c1addat:%d \n", c1addat);
	VOS_LOG(VOS_LOG_DEBUG,"ch1HVR:%d \n", ch1HVR);
	VOS_LOG(VOS_LOG_DEBUG,"ch1HH:%d \n", ch1HH);
	VOS_LOG(VOS_LOG_DEBUG,"ch1HL:%d \n", ch1HL);
	VOS_LOG(VOS_LOG_DEBUG,"ch1LH:%d \n", ch1LH);
	VOS_LOG(VOS_LOG_DEBUG,"ch1LL:%d \n", ch1LL);
	VOS_LOG(VOS_LOG_DEBUG,"ch1LVR:%d \n", ch1LVR);
	VOS_LOG(VOS_LOG_DEBUG,"const1:%d \n", const1);
	VOS_LOG(VOS_LOG_DEBUG,"t1offset:%d \n", t1offset);

	//VOS_LOG(VOS_LOG_DEBUG,"c2addat:%d \n", c2addat);
	VOS_LOG(VOS_LOG_DEBUG,"ch2HVR:%d \n", ch2HVR);
	VOS_LOG(VOS_LOG_DEBUG,"ch2HH:%d \n", ch2HH);
	VOS_LOG(VOS_LOG_DEBUG,"ch2HL:%d \n", ch2HL);
	VOS_LOG(VOS_LOG_DEBUG,"ch2LH:%d \n", ch2LH);
	VOS_LOG(VOS_LOG_DEBUG,"ch2LL:%d \n", ch2LL);
	VOS_LOG(VOS_LOG_DEBUG,"ch2LVR:%d \n", ch2LVR);
	VOS_LOG(VOS_LOG_DEBUG,"const2:%d \n", const2);
	VOS_LOG(VOS_LOG_DEBUG,"t2offset:%d \n", t2offset);

	//VOS_LOG(VOS_LOG_DEBUG,"c3addat:%d \n", c3addat);
	VOS_LOG(VOS_LOG_DEBUG,"ch3HVR:%d \n", ch3HVR);
	VOS_LOG(VOS_LOG_DEBUG,"ch3HH:%d \n", ch3HH);
	VOS_LOG(VOS_LOG_DEBUG,"ch3HL:%d \n", ch3HL);
	VOS_LOG(VOS_LOG_DEBUG,"ch3LH:%d \n", ch3LH);
	VOS_LOG(VOS_LOG_DEBUG,"ch3LL:%d \n", ch3LL);
	VOS_LOG(VOS_LOG_DEBUG,"ch3LVR:%d \n", ch3LVR);
	VOS_LOG(VOS_LOG_DEBUG,"const3:%d \n", const3);
	VOS_LOG(VOS_LOG_DEBUG,"t3offset:%d \n", t3offset);
	
}


