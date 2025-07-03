#ifndef _FILTER_H
#define _FILTER_H

#include "main.h"
#include "kalman.h"

typedef struct
{
	float now[2];
	float last[2];
	float output[2];
}Filter_param;
extern Filter_param LimitFilter;

void FilterLimit(void);
void my_filter(void);

#endif
