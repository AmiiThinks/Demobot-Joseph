#include "baseline.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct timeval timeBegin, timeNow;

// Agent thread code
void agentInitialize(){
  gettimeofday(&timeBegin, NULL);
}
void agentFuneral(){
}

const int FORWARD=0;
const int BACKWARD=3;
const int CLOCKWISE=1;
const int COUNTERCLOCKWISE=2;

float epsilon=.3;
int lastact=0;

int agentPolicy(int s) { 
  if (rand()/((double)RAND_MAX+1) < epsilon) {
    printf("random action\n");
    return rand()%4;
  } else {
    return lastact;
  }
}

void agentUpdate(int obs, int act,int pktNum) { 
  lastact=act;
  gettimeofday(&timeNow, NULL);
  float elapsedTime=(timeNow.tv_sec-timeBegin.tv_sec)
      + (timeNow.tv_usec-timeBegin.tv_usec)*.000001;
  printf("Time elapsed: %8.4f pkt: %d\n", elapsedTime,pktNum);

}
int main(int argc, char **argv) {
  return bmain(argc,argv);
}
