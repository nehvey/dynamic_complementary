/*
 * queue_avg.h
 *
 *  Created on: 05.08.2018
 *      Author: fliess
 */

#ifndef QUEUE_AVG_H_
#define QUEUE_AVG_H_

#include "stdlib.h"
#include "stdio.h"

typedef struct {
	float *items;
	int maxsize;
	int head;
	int tail;
	volatile int size;
	volatile int offset;
	volatile int remains;
	volatile float total;
} QueueAvg;

QueueAvg *newQueueAvg(int size);

int sizeAvg(QueueAvg *pt);

int isEmptyAvg(QueueAvg *pt);

void enqueueAvg(QueueAvg *pt, float v);

float dequeueAvg(QueueAvg *pt);

float nextAvg(QueueAvg *pt);

char hasNextAvg(QueueAvg *pt);

void resetOffsetAvg(QueueAvg *pt);

float getAvg(QueueAvg *pt);

#endif /* QUEUE_AVG_H_ */
