#ifndef _MANUFACTURE_MANAGER_H_
#define _MANUFACTURE_MANAGER_H_

#pragma   pack(1)

/* 16bytes alignment */
typedef struct
{
    unsigned short usYear;
    unsigned char  ucMon;
    unsigned char  ucDay;
}MFD_DATE_STRU;

typedef struct
{
    unsigned char id[6];
}MFD_OEM_STRU;


typedef struct
{
    /* BARCODE :24bytes Maximum  */
    unsigned char id[24];
}MFD_BID_STRU;


typedef struct
{
    /* mid: 0860210001XXXX
            XXXX: product category
       sid: subversion 
    */
    unsigned char mid[12]; /* Main Id */
    unsigned char sid[6];
    
}MFD_CID_STRU;

typedef struct
{
    unsigned char id[6];  /* eerom unique id */
}MFD_UID_STRU;


typedef struct
{
    unsigned char id[12]; /* mcu chip id */
}MFD_EID_STRU;


typedef struct
{
    MFD_UID_STRU  uid;
    MFD_DATE_STRU date;
    MFD_EID_STRU  eid;
    MFD_OEM_STRU  oem;
    MFD_CID_STRU  cid;
    MFD_BID_STRU  bid;
}MFD_STRU;

#define MFD_LOAD_LENGTH (sizeof(MFD_STRU) - sizeof(MFD_UID_STRU))

#pragma pack()

#endif

