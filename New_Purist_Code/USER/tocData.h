#ifndef _TOCDATA_H_
#define _TOCDATA_H_

#include "DtypeStm32.h"

#define D_SIZE 100

typedef struct 
{
	double a[D_SIZE];
	int cursor;
	int pos;
	double min;
	int isFull;
}TocData;


/************************************************
*Description: TOC calculation new algorithm data structure
*date: 2019.03.25
************************************************/
#define D_SAMPLESNUM 100     //data samples
#define D_VALUESNUM  10       //ValuesNum
typedef struct
{
	double a[D_VALUESNUM];
	int cursor;
	int total;
	double value;

}TocValues;

void addTocData(TocData* datas, double value);
double calcValue(TocData datas);


/************************************************
*Description: TOC calculation new algorithm
*date: 2019.03.25
************************************************/
void addTocValue(TocValues* TocValues, double value);
void calcTargetValue(TocValues* TocValues);
BOOL checkValuesDiff(TocValues* TocValues);

#endif

