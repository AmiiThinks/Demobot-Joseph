#include "baseline.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int global_argc;
char**global_argv;

struct timeval timeBegin, timeNow;
int timestep=0;
// Agent thread code
void setupLearner();
void learningUpdate(int obs, int act,int lastact);
void printTiles();

double Q[16][4], e[16][4];
double stepsize = 0.1, lambda = 0.9, gamma = 0.5, epsilon = 0.1;

void agentInitialize(){
  gettimeofday(&timeBegin, NULL);
  setupLearner();
}

void agentFuneral(){
}

const int FORWARD=0;
const int BACKWARD=3;
const int CLOCKWISE=1;
const int COUNTERCLOCKWISE=2;
int lastact;

int front_outside(int obs);
int back_outside(int obs);

int agentPolicy(int s) { 
  int max, i;
  if (rand()/((double)RAND_MAX+1) < epsilon) {
    printf("random action\n\n");
    return rand()%4;
  } else {
    max = 3;
    for (i = 0; i < 4; i++)
      if (Q[s][i] > Q[s][max])
        max = i;
    return max;
  }
}

double elapsedTime;

void agentUpdate(int obs, int act,int pktNum) { //post policy
  //save logs as needed.
  gettimeofday(&timeNow, NULL);
  elapsedTime=(timeNow.tv_sec-timeBegin.tv_sec)
      + (timeNow.tv_usec-timeBegin.tv_usec)*.000001;
  printf("Time elapsed: %8.4f pkt: %d\n", elapsedTime,pktNum);
  learningUpdate(obs,act,lastact);
  timestep++;
  lastact=act;
}


//---------------------------------- Learning infrastructure
void setupLearner() {
  int i,j;
  //initialize questions and resources
  for (i = 0; i < 16; i++)
    for (j = 0; j < 4; j++) {
      Q[i][j] = 5 + 0.001*( rand()/((double) RAND_MAX) - 0.5);
      e[i][j] = 0;
    }
}

int s=0,a=0;
int rewardReport=0;
int rewardSign=1;
void learningUpdate(int obs, int act, int lastact) {
  int reward=rewardSign*getDistanceReward();
  rewardReport+=reward;
  int sprime=obs;
  int aprime=act;
  int i,j;
  float delta;
    delta = reward + gamma*Q[sprime][aprime] - Q[s][a];
    for (j = 0; j < 4; j++)
      e[s][j] = 0;
    e[s][a] = 1;
    printf("s a r s' a':%d %d %d %d %d\n", s, a, reward, sprime, aprime);
    for (i = 0; i < 16; i++) {
      //printf("Action values for state %d: %f %f %f %f\n",i,
      //	     Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
      for (j = 0; j < 4; j++) {
        Q[i][j] = Q[i][j] + stepsize*delta*e[i][j];
        e[i][j] = gamma*lambda*e[i][j];
      }
    }
    s = sprime;
    a = aprime;
    
    // check the remote
    int rc=lastRemoteByte();
    if (rc==130) { // forward
      agentInitialize();
    } else if (rc== 137) {
      endProgram(1234);
      printf("Calling halt.\n");
    } else if (rc== 131) {
      rewardSign*=-1;
      agentInitialize();
    }

    if (rewardReport > 50) {
      rewardReport-=50;
      beep();
    }
      
}

int main(int argc, char **argv) {
  global_argc=argc;
  global_argv=argv;
  return bmain(argc,argv);
}
