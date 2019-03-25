#include "tocData.h"
#include "math.h"

void addTocData(TocData* datas, double value)
{
	if(datas->cursor >= D_SIZE)
	{
		return;
	}
	
	if(0 == datas->cursor)
	{
		datas->min = value;
		datas->pos = 0;
	}

	datas->a[datas->cursor] = value;

	if(datas->a[datas->cursor] < datas->min)
	{
		double diff = datas->a[datas->cursor - 1] - datas->a[datas->cursor];
		
		if(diff < 2)
		{
			datas->min = datas->a[datas->cursor];
			datas->pos = datas->cursor;
		}
	}

	if(datas->cursor == (D_SIZE - 1))
	{
		datas->isFull = 1;
	}

	datas->cursor++;
}

double calcValue(TocData datas)
{
	int pos = datas.pos;
	double value = 0;
	if(pos > 5)
	{
		for(int i = pos - 4; i < pos + 6; i++)
		{
			value += datas.a[i];
		}

		value /= 10;
	}
	else
	{
		for(int i = pos; i < pos + 5; i++)
		{
			value += datas.a[i];
		}
		value /= 5;
	}

	//
	if(fabs(value - datas.min) > 2)
	{
		return 0;
	}
	return value;
	
}

void addTocValue(TocValues* TocValues, double value)
{
	if(TocValues->total < D_VALUESNUM)
	{
		TocValues->total++;
	}

	if(TocValues->cursor >= D_VALUESNUM)
	{
		TocValues->cursor = 0;
	}
	TocValues->a[TocValues->cursor] = value;
	TocValues->cursor++;

	if(D_VALUESNUM == TocValues->total++)
	{
		calcTargetValue(TocValues);
	}

}
void calcTargetValue(TocValues* TocValues)
{
	double temp = 0;
	for(int i = 0; i < D_VALUESNUM; i++)
	{
		temp += TocValues->a[i];
	}
	temp /= D_VALUESNUM;
	
	if(0 == TocValues->value)
	{
		TocValues->value = temp;
	}
	
	if(TocValues->value > temp)
	{
		if(checkValuesDiff(TocValues))
		{
			TocValues->value = temp;
		}
	}

}

BOOL checkValuesDiff(TocValues* TocValues)
{
	double min = TocValues->a[0];
	double max = TocValues->a[0];

	for(int i = 1; i < D_VALUESNUM; i++)
	{
		if(TocValues->a[i] < min)
		{
			min = TocValues->a[i];
		}
		if(TocValues->a[i] > max)
		{
			max = TocValues->a[i];
		}
	}

	double diff = max - min;
	if(diff > 1.0)
	{
		return FALSE;
	}
	return TRUE;

}



