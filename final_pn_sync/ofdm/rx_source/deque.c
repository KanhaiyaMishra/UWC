#include <stdio.h>
#include "deque.h"

void initialize(dequeue *P, pair *queue, int size)
{
    P->size = size;
    P->base_ptr = queue;
    P->rear=-1;
    P->front=-1;
}

int empty(dequeue *P)
{
    if(P->rear==-1)
        return(1);
    return(0);
}

int full(dequeue *P)
{
    if((P->rear+1)%P->size==P->front)
        return(1);
    return(0);
}

void enqueueR(dequeue *P, pair x)
{
    if(empty(P))
    {
        P->rear=0;
        P->front=0;
        P->base_ptr[0]=x;
    }
    else
    {
        P->rear=(P->rear+1)%P->size;
        P->base_ptr[P->rear]=x;
    }
}

void enqueueF(dequeue *P, pair x)
{
    if(empty(P))
    {
        P->rear=0;
        P->front=0;
        P->base_ptr[0]=x;
    }
    else
    {
        P->front=(P->front-1+P->size)%P->size;
        P->base_ptr[P->front]=x;
    }
}

void dequeueF(dequeue *P)
{
    if(P->rear==P->front){
        P->rear=-1;
        P->front=-1;
    }
    else
        P->front=(P->front+1)%P->size;
}

void dequeueR(dequeue *P)
{
    if(P->rear==P->front){
        P->rear=-1;
        P->front=-1;
    }
    else
        P->rear=(P->rear-1+P->size)%P->size;
}
void print(dequeue *P)
{
    if(empty(P))
        printf("\nQueue is empty!!");
    int i;
    i=P->front;
    while(i!=P->rear)
    {
        fprintf(stdout,"\nvalue = %f, position = %d \n", P->base_ptr[i].value, P->base_ptr[i].position);
        i=(i+1)%P->size;
    }
    fprintf(stdout,"\nvalue = %f, position = %d \n", P->base_ptr[P->rear].value, P->base_ptr[P->rear].position);
}
