#include<ofdm.h>

#define MAX (POST_DSF*N_CP_SYNC+1)
typedef struct{
    float value;
    int position;
}pair;

typedef struct{
    pair data[MAX];
    int rear,front;
}dequeue;

void initialize(dequeue *p);
int empty(dequeue *p);
int full(dequeue *p);
void enqueueR(dequeue *p, pair x);
void enqueueF(dequeue *p, pair x);
void dequeueF(dequeue *p);
void dequeueR(dequeue *p);
void print(dequeue *p);
