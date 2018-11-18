/*
 * queue_avg.c
 *
 *  Created on: 05.08.2018
 *      Author: fliess
 */

#include "queue_avg.h"
#include <float.h>

QueueAvg *newQueueAvg(int size) {
	QueueAvg *q = NULL;
	q = (QueueAvg *) malloc(sizeof(QueueAvg));
	q->items = (float *) malloc(size * sizeof(float));
	q->maxsize = size;
	q->head = -1;
	q->tail = -1;
	q->size = 0;
	q->offset = -1;
	q->remains = 0;
	q->total = 0;
	return q;
}

int sizeAvg(QueueAvg *pt) {
	return pt->size;
}

int isEmptyAvg(QueueAvg *pt) {
	return !size(pt);
}

void enqueueAvg(QueueAvg *pt, float v) {
	if (pt->size == pt->maxsize) {
		float d = dequeueAvg(pt);
		pt->total -= d;
	}
	pt->total += v;

	pt->tail = (pt->tail + 1) % pt->maxsize;
	pt->items[pt->tail] = v;
	pt->size++;
}

float dequeueAvg(QueueAvg *pt) {
	pt->head = (pt->head + 1) % pt->maxsize;
	pt->size--;
	return pt->items[pt->head];
}

float nextAvg(QueueAvg *pt) {
	pt->remains--;
	pt->offset = (pt->offset + 1) % pt->maxsize;
	return pt->items[pt->offset];
}

char hasNextAvg(QueueAvg *pt) {
	return pt->remains > 0;
}

void resetOffsetAvg(QueueAvg *pt) {
	pt->remains = pt->size;
	pt->offset = pt->head;
}

float getAvg(QueueAvg *pt) {
	return pt->total / pt->size;
}
