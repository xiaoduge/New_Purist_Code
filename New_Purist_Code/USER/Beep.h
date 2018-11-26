#ifndef _BEEP_H_
#define _BEEP_H_

#include "DtypeStm32.h"
#include "memory.h"
#include "timer.h"

void MainAlarmWithDuration(UINT8 ucDuration);
void AlarmInit(void);
void Alarm(int On);
void AlarmToggle(void);

void MainBeepWithDuration(UINT8 ucDuration);
void BeepInit(void);
void Beep(int On);
void BeepToggle(void);
void RunInit(void);
void RunToggle(void);

#endif
