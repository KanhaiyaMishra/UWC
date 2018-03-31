#include <stdio.h>
#include <deque.h>

void initialize(dequeue *P)
{
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
    if((P->rear+1)%MAX==P->front)
        return(1);
    return(0);
}

void enqueueR(dequeue *P, pair x)
{
    if(empty(P))
    {
        P->rear=0;
        P->front=0;
        P->data[0]=x;
    }
    else
    {
        P->rear=(P->rear+1)%MAX;
        P->data[P->rear]=x;
    }
}

void enqueueF(dequeue *P, pair x)
{
    if(empty(P))
    {
        P->rear=0;
        P->front=0;
        P->data[0]=x;
    }
    else
    {
        P->front=(P->front-1+MAX)%MAX;
        P->data[P->front]=x;
    }
}

void dequeueF(dequeue *P)
{
    if(P->rear==P->front)    //delete the last element
        initialize(P);
    else
        P->front=(P->front+1)%MAX;
}

void dequeueR(dequeue *P)
{
    if(P->rear==P->front)
        initialize(P);
    else
        P->rear=(P->rear-1+MAX)%MAX;
}
void print(dequeue *P)
{
    if(empty(P))
    {
        printf("\nQueue is empty!!");
        exit(0);
    }
    int i;
    i=P->front;
    while(i!=P->rear)
    {
        fprintf(stdout,"\nvalue = %f, position = %d \n", P->data[i].value, P->data[i].position);
        i=(i+1)%MAX;
    }
    fprintf(stdout,"\nvalue = %f, position = %d \n", P->data[P->rear].value, P->data[P->rear].position);
}
