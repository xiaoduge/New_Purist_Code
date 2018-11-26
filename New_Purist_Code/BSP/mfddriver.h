#ifndef _MFDDRIVER_H_
#define _MFDDRIVER_H_

#include <stdbool.h>

void mfd_Init(int gpio,uint8_t ucAdr);
int mfd_readid(uint8_t *pucId);
int mfd_readmfd(uint8_t *pucId);
int mfd_isPresent(void);
int read_mfd(uint8_t *aucIn,uint8_t *aucOut);
int write_mfd(uint8_t *aucInfo);
void mfd_EnableDebug(bool bEnable);

#endif
