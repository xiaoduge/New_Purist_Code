#ifndef _MODBUS_H_
#define _MODBUS_H_

void ModbusInit(void);
void ch1_meas_proc_Inner(float *c,float *t, uint16_t *Rt);
void ch2_meas_proc_Inner(float *c,float *t);
void ch3_meas_proc_Inner(float *c,float *t);
//void setTOffset(int ichl, UINT32 offset);

#endif
