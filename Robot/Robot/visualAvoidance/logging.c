#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "logging.h"

FILE *logfile=NULL;
bool terminate=false;

char **dataLines=NULL;

int msSleep=-1;
int numLines=-1;
int nextStore=0;
int nextLogn=0;
void* tsafeLogger(void *arg);

pthread_mutex_t nextMutex;

int lockReadStore() {
  int val;
  pthread_mutex_lock( &nextMutex );
  val = nextStore;
  pthread_mutex_unlock( &nextMutex );
  return val;
}

void lockWriteStore(int val) {
  pthread_mutex_lock( &nextMutex );
  nextStore=val;
  pthread_mutex_unlock( &nextMutex );
}

int lockReadLogn() {
  int val;
  pthread_mutex_lock( &nextMutex );
  val = nextLogn;
  pthread_mutex_unlock( &nextMutex );
  return val;
}

void lockWriteLogn(int val) {
  pthread_mutex_lock( &nextMutex );
  nextLogn=val;
  pthread_mutex_unlock( &nextMutex );
}
void stoplog() {
  terminate=true;
  while(logfile != NULL) {
    usleep(msSleep*1000);
  }

}
void start_log(const char* filename, const char * header,int max_line_size, int ms_sleep, int num_lines) {
  pthread_t tid;
  int t_err;
  numLines=num_lines;
  dataLines=new char*[numLines];
  for (int i=0;i<numLines;i++)
    dataLines[i]=new char[max_line_size];
  
  msSleep=ms_sleep;
  pthread_mutex_init(&nextMutex,NULL);

  logfile=fopen(filename,"w");
  if (!logfile) {
    fprintf(stderr,"Logging file could not be opened.\n");
    return;
  }
  printf("Logging to %s started\n",filename);
  fputs(header,logfile);
  printf("Starting log thread\n");
  t_err = pthread_create(&tid, NULL, &tsafeLogger, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create logger thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }
}

void writelog(const char *line) {
  int mylogn= lockReadLogn();
  strcpy(dataLines[mylogn],line);
  mylogn=(mylogn + 1) % numLines;
  int mystore= lockReadStore();
  if (mystore!=mylogn)
    lockWriteLogn(mylogn);
  else {
    printf("Log is not writing to disk fast enough!");
  }
}

void* tsafeLogger(void *arg) {
  //open a logging file and write all packets to it with sleeps outside

  printf("Threadsafe logging started\n");
  if (!logfile) {
    fprintf(stderr,"Logging file could not be opened.\n");
    return NULL;
  }
  while(!terminate) {
    //save lines
    int k=lockReadStore();
    int l=lockReadLogn();
    if (l==k) {
      usleep(msSleep*1000);
      continue;
    }
    fputs(dataLines[k],logfile);
    //printf("Wrote:%d\n",k);
    k=(k + 1) % numLines;
    lockWriteStore(k);
  }
  fclose(logfile);
  logfile=NULL;
  return NULL;
}

/*
int testingmain() {
  //test logging
  start_log("test.log", "#what's up\n",80,100, 10);
  char ff[80];
  for (int i=0;i< 50;i++) {
    sprintf(ff,"%d\n",i);
    writelog(ff);
    printf("Logged:%d\n",i);
    usleep(50000);
  }
 stoplog();
 printf("Done\n");
}
*/
