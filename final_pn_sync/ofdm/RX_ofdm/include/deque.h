#ifndef __DEQUE_H
#define __DEQUE_H

typedef struct{
    float value;
    int position;
}pair;

typedef struct{
    pair *base_ptr;
    int size;
    int rear;
    int front;
}dequeue;

void initialize(dequeue *p, pair *queue, int size);
int empty(dequeue *p);
int full(dequeue *p);
void enqueueR(dequeue *p, pair x);
void enqueueF(dequeue *p, pair x);
void dequeueF(dequeue *p);
void dequeueR(dequeue *p);
void print(dequeue *p);

#endif
