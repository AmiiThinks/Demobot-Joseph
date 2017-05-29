#include "robotBase.h"
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v4l2write.h"
#include <pthread.h>
#include "logging.h"
#include <unistd.h>
#include "server_udp.h"
#include <math.h>
#include "sounds.h"
#include "pixels.h"

const char logfile_prefix[]="../logfiles/demowallerCore";

int global_argc;
char**global_argv;

// MM is the number of bytes used from the image
#define MM 500
// RSHIFT provides a fast division by a power of 2
#define BYTE_RSHIFT 3
// The number of features per byte
#define BYTE_HAS_NUMFEATURES (1<< (8-BYTE_RSHIFT))
//#define BYTE_MULTIPLIER (1<< BYTE_RSHIFT)

//NN is the number of features (the size of a weight vector)
#define  NN  MM*BYTE_HAS_NUMFEATURES

#define DEMONS 1
#define NND    NN*DEMONS


const int FORWARD=0;
const int BACKWARD=3;
const int CLOCKWISE=2;
const int COUNTERCLOCKWISE=1;
const int STOP=4;
const int SAFE_SHUTDOWN=5;

const int FORWARD_MODE=0;
const int REACTA_MODE=1;
const int REACTB_MODE=2;
const int EXPLORE_MODE=3;
const int RAND_LEFT=4;
const int RAND_RIGHT=5;

int action_prime=STOP;
int mode_duration=0;
const int explore_on=20;
const int exit_turn_on=5;

struct timeval timeBegin, timeNow,timeVid,timeUpdate;
int timestep=0;
// Agent thread code

float make_prediction(int x[], int ind);
void init_learning() ;
void load_weights(const char *filename);
void save_weights(const char *filename);
void select_vid();
void fill_features(int vc,int x[]);
int my_vc=0;
int xplus[NN];
int xnow[NN];

float et[NND];
float w[NND];
float ww[NND];

float gammas[DEMONS];
float rs[DEMONS];
float rhos[DEMONS];
float predictions[DEMONS];
const float alpha = 0.1/ MM;
const float alpha2 = alpha/1000.;
const float lambda = .9;

#define LOGSTR_SZ 3000
char logstring[LOGSTR_SZ];
char logstring2[LOGSTR_SZ]="";
int iteration=-1;
int era=0;
const float prediction_threshold=0.505;

float delta, rho;


int selected_pixels[2][MM];
float hit_decay=0;
int startingLights=0;

int bump_robot_observation;
double elapsedTime;


pthread_t tid;
pthread_t net_tid;
void * mainNetLoop(void*); 

int use_prediction=0;
int use_random=0;
int learn_prediction=0;

int data_size=0;
int *pkt_data;
int last_act=-1;


void ACInit();
//void flip_era_if_needed();
void agentInitialize(){
  ACInit();
  learn_prediction=1; 
  if (global_argc <=1) {
    printf("No args\n");
    printf("Default Pavlov mode\n");
    use_prediction=1;
    init_learning();
  } else {
    printf("argument: %s\n",global_argv[1]);
    char c=global_argv[1][1];
    if (c=='p') {
       printf("Pavlov mode\n");
       use_prediction=1;
       init_learning();
    } else if (c=='f') {
       printf("Fixed weight mode\n");
       use_prediction=1;
       learn_prediction=0;
       init_learning();
       load_weights(global_argv[2]);
    } else if (c=='R') {
       printf("Random mode\n");
       use_random=1;
       use_prediction=1;
       init_learning();
    } else  { // if (c=='r')
      printf(" Reflex mode\n");
      use_prediction=0;
      init_learning();
    } 
  }
  
  int t_err = pthread_create(&tid, NULL, &mainimageloop, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create video thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }
  t_err = pthread_create(&tid, NULL, &mainNetLoop, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create networking thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }
}

void agentFuneral(){
  stop_video();
  printf("Saving final weights\n");
  save_weights("./my_weights.end");
}
//---- network
void * mainNetLoop(void*) {
  printf("Started networking\n");
  server(12345);
  return NULL;
}

char * get_packet(int *sz) {
  *sz=strlen(logstring2);
  return logstring2;
}
//---




int vid_count=0;

//int pixelstep=20011;
bool want_snapshot=false;
int videoFailures=0;
void process_image(const void *p, int size){ // implemented by the client now.
  gettimeofday(&timeVid, NULL);
  if (want_snapshot) {
    char name[50];
    sprintf(name,"snapshot-%ld-%ld.yuv",timeVid.tv_sec,timeVid.tv_usec);
    FILE *f=fopen(name,"w");
    fwrite(p,1,size,f);
    fclose(f);      
    want_snapshot=false;
  }
  if (size != pixelsize){
    printf ("Pixel size is WRONG!");
    videoFailures++;
  } else {
    videoFailures=0;
  }
  if (videoFailures > 10)
    endProgram(0);
  
  //lock
  vid_count++;
  //time and video buffer number printf("%f %d\n",netTime,vid_count);
  for (int i=0;i<MM;i++) {
    selected_pixels[vid_count%2][i]=((unsigned char*) p) [pixelinds[i]] >> BYTE_RSHIFT;
  }
  //unlock
}

float uniform_random() {
  return (random()%10000)/10000.;
}

//--------------- Modeless support

bool exploration_required() {
  return  1 > (random()%explore_on) ;
}


void process_externals(int s) {
  if (s !=0) 
    hit_decay =1;
  else
    hit_decay *=0.95;

  int q= lastRemoteByte();
  int qq= lastButtonByte();

  if (q==REMOTE_SPOT || (qq & 4) ) { //pause
    action_prime=STOP;
  } else if (q==REMOTE_FORWARD || (qq & 1)) { //start
    action_prime=FORWARD;
    set_make_noise(false);
    startingLights=10;
  } else  if (q==REMOTE_MAX ) {
    init_learning(); 
    use_prediction = true;
  } else if (q==REMOTE_CLEAN ) {
    use_prediction = false;
  }
}

int oneline_policy(){
  return  (predictions[0] > 0.7) ? COUNTERCLOCKWISE : FORWARD;
}


float behaviour_selected_action_probability=1;
int weighted_preferences_policy() {
  float p0=predictions[0];

  float decisions_per_second=100/3.;
  float expected_switches_per_second=5;
  float T=decisions_per_second/expected_switches_per_second;
  int num_actions=2;
  float k_switch=log((T-1)*(num_actions-1));
  printf("k_switch :%f\n",k_switch);
  float k1=k_switch*4,k2=k_switch;//,k3=k1/2;
  
  float pref[3],weights[4];
  pref[0]=-k1*(p0-0.5) + k2*(action_prime==FORWARD) ;
  pref[1]=k2*(action_prime==COUNTERCLOCKWISE);
  pref[2]=k2*(action_prime==CLOCKWISE);

  float last=0;
  weights[0]=0;
  for (int i=0;i<num_actions;i++) { 
    weights[i+1]=last+exp(pref[i]);
    last=weights[i+1];
  }

  float roll=uniform_random()*last;
  int selection=-1;
  for (int i=0;i<num_actions;i++) {
    if (roll < weights[i+1]) {
      selection=i;
      behaviour_selected_action_probability=(weights[i+1]-weights[i])/last;
      //behaviour_selected_action_probability=1;
      break;
    }
  }

	 
  int out= (selection==0) ? FORWARD : (selection==1) ? COUNTERCLOCKWISE: CLOCKWISE;       
  if (behaviour_selected_action_probability < .5 || 0==iteration %100) 
     printf("iteration: %d T: %4.3f pred:%4.3f \t b(a|s): %5.3f  inverse: %5.3f\n", iteration, T, p0, behaviour_selected_action_probability , 1./behaviour_selected_action_probability );
  return out;
}
double pi(int actK, float x[]);

int select_from_pi() {
  int num_actions=2;
  float x[5];
  x[0]=1;
  x[1]=predictions[0];
  x[2]=action_prime==FORWARD;
  x[3]=action_prime==COUNTERCLOCKWISE;
  x[4]=0;

  float last=0;
  float weights[4];
  weights[0]=0;
  for (int i=0;i<num_actions;i++) { 
    weights[i+1]=last+pi(i,x);
    last=weights[i+1];
  }

  float roll=uniform_random()*last;
  int selection=-1;
  for (int i=0;i<num_actions;i++) {
    if (roll < weights[i+1]) {
      selection=i;
      behaviour_selected_action_probability=(weights[i+1]-weights[i])/last;
      break;
    }
  }
  
  int out= (selection==0) ? FORWARD :  COUNTERCLOCKWISE;
  return out;
}

int real_policy() {
  //return oneline_policy();
  //return weighted_preferences_policy();
  //return select_from_pi();
  return (hit_decay > .8 || (last_act==COUNTERCLOCKWISE && uniform_random() > 2*.03 )) ? COUNTERCLOCKWISE : FORWARD;
}


int agentSelectAction(int s) {
  //remote
  process_externals(s);

  //video
  select_vid();
  fill_features(my_vc,xplus);
  for (int i=0;i<DEMONS;i++)
    predictions[i]=make_prediction(xplus,i);
  
  //agent state
  bump_robot_observation=s;
  if (bump_robot_observation)
    play_hit();
  if (action_prime==STOP)
    return action_prime;
  //select action

  action_prime=real_policy();
  return action_prime;
}


//----------------


void init_learning() {
  for (int i=0;i<NN;i++) {
    xplus[i]=0;xnow[i]=0;
  }  
  for (int i=0;i<NND;i++) {
    et[i]=0;w[i]=0;ww[i]=0;    
  }
}

void load_weights(const char *filename){
  FILE *f=fopen(filename,"r");
  for (int i=0;i<NND;i++ )
    if (1!=fscanf(f,"%f",w+i)) {
      printf("Failed to read weights from file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NND;i++ )
    if (1!=fscanf(f,"%f",ww+i)) {
      printf("Failed to read weights from file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NND;i++ )
    et[i]=0;
  fclose(f);
}

void save_weights(const char *filename){
  FILE *f=fopen(filename,"w");
  for (int i=0;i<NND;i++ )
    if (0>fprintf(f,"%f\n",*(w+i))) {
      printf("Failed to write weights to file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NND;i++ )
    if (0>fprintf(f,"%f\n",*(ww+i))) {
      printf("Failed to read weights to file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NND;i++ )
    et[i]=0;
  fclose(f);
}

void select_vid() {
  my_vc=vid_count%2;
}

void fill_features(int vc,int x[]){
  for (int i=0;i< NN; i++) {
    x[i] = 0;
  }
  int *s=selected_pixels[vc]; // this needs to be made thread safe
  for (int i=0;i< MM; i++) {
    int k=i*BYTE_HAS_NUMFEATURES + s[i];
    x[k] = 1;
  }
}


float make_prediction(int x[],int demon){
  float lprediction=0;
  int k=demon*NN;
  for (int i=0;i< NN; i++) {
    float q=w[k+i] *x[i];
    lprediction += q;
  }
  return lprediction;
}

float dot(float a[],float b[],int n) {
  float out=0;
  for (int i=0;i<n;i++) 
    out += a[i]*b[i];
  return out;
}

float idot(int a[],float b[],int n) {
  float out=0;
  for (int i=0;i<n;i++) 
    out += a[i]*b[i];
  return out;
}

void GTDupdate(int demon) {
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  int offset=k*NN;
  delta=0;
  
  //GTD(lambda) with rho
  if (learn_prediction) {
    // set variable gamma going to zero on r.
    delta = r;
    for (int i=0;i< NN; i++)
      delta += gamma * w[i+offset] * xplus[i]  - w[i+offset] * xnow[i];
    for (int i=0;i<NN;i++) {
      et[i+offset] =rho *(xnow[i] + et[i+offset]);
    }
    float ew=dot(et+offset,ww+offset , NN);
    float xw=idot(xnow,ww+offset,NN);
    float glew=gamma * (1-lambda) * ew;
    
    for (int i=0;i< NN; i++) {
      w[i+offset] += alpha * ( delta * et[i+offset] - glew * xplus[i]);
      ww[i+offset] += alpha2 * (delta * et[i+offset] - xw * xnow[i]);
    }
    
    //eligibility trace
    for (int i=0;i<NN;i++) {
      et[i+offset] *= lambda * gamma;
    }
  }
}

float alphaR=.01,alphaU=.00001,alphaV=.00001;
float lambdaAC=0.8;
#define nAC 4
#define CEQ 0.5

// log_e(T-1) for T=20/3 
#define KS 1.7346


float xACprime[nAC];
float xAC[nAC];

float v_now=0;
float v_plus=0;
float r_delta;
float rbar;
float u[3];
float v[nAC];
float eU[3];
float eV[nAC];

void ACInit() {
  u[0]=KS*4*CEQ;
  u[1]=-KS*4;
  u[2]=KS;
}

double pi(int actK, float x[]) {
  float p[2];
  p[0]= exp(u[0]*x[0] + u[1]*x[1] + u[2]*x[2]);
  p[1]= exp(u[2]*x[3]);
  //p[2]= exp(u[2]*x[4]);
  float tot=p[0]+p[1];//+p[2];
  return p[actK]/tot;
}

double grad_X(int actK,int uI, float x[] ) {
  //compute the gradient for our given policy
  if (2==uI)
    return 1-pi(0,x)*x[2] -pi(1,x)*x[3];// -pi(2,x)*x[4] ;
  if (0==actK) {
    if (0==uI) return (1-pi(0,x))*x[0];
    //if (1==uI) 
    return (1-pi(0,x))*x[1];
  }
  if (0==uI) 
    return -pi(0,x)*x[0];
  return -pi(0,x)*x[1];
  
}

void ActorCriticUpdate(float reward, int actK) {
  // set act,lastact
  //set xAC , xACprime
  v_now=dot(xAC,v,nAC);
  v_plus=dot(xACprime,v,nAC);
  float delta = reward- rbar + v_plus - v_now;
  
  // avg
  r_delta=delta;
  rbar += alphaR* delta; 
  //critic
  for (int i=0;i<nAC;i++) {
    eV[i] = lambdaAC * eV[i]  + xAC[i];
    v[i] += alphaV * delta *eV[i];
  }
  //actor
  for (int i=0;i<3;i++) {
    eU[i] = lambdaAC* eU[i] + grad_X(actK,i,xAC);
    u[i] += alphaU * delta * eU[i];
  }
}

int last_printed=0;

void agentUpdateAfterAction(int obs, int act,int pktNum) { //post policy
  gettimeofday(&timeNow, NULL);
  int time_delta=timeNow.tv_sec*1.-timeBegin.tv_sec*1.;
  double netTime = time_delta+ (timeNow.tv_usec*1.-timeBegin.tv_usec)/1000000.;
  notes(netTime,predictions[0]);
  if (startingLights >0) {
    setLEDs(255,255, 1 , 1);
    startingLights--;
  }
  //  setExtLEDs(predictions[0] > .36 ,  predictions[0] > .5 ,predictions[0] > .18 );
  

  //start of learning
  rs[0] =obs > 0 ? 1 : 0;
  gammas[0]=rs[0]>0 ? 0 : 1-3./100.;
  rhos[0]=last_act==FORWARD ? 1: 0;
  //printf("biased\n");
  behaviour_selected_action_probability=1; //biased
  for (int k=0;k<DEMONS;k++) {
    if (behaviour_selected_action_probability < .1)  {
      if (10*behaviour_selected_action_probability < uniform_random()) {
	rhos[k]/=0.1;
      } else {
	rhos[k]=0;
      }
    }  else {      
      rhos[k]/= behaviour_selected_action_probability;
    }
  }

  for (int k=0;k<DEMONS;k ++) {
    GTDupdate(k);
  }
  int dist= getDistanceReward();
  float reward=dist - 100 * rs[0];
  //printf("Reward: %f\n", reward);
  xACprime[0]=1;
  xACprime[1]=predictions[0];
  xACprime[2]=last_act==FORWARD;
  xACprime[3]=last_act==COUNTERCLOCKWISE;
  //  xACprime[4]=0;

  ActorCriticUpdate(reward,act);
  for (int i=0;i<nAC;i++)
    xAC[i]=xACprime[i];


  for (int i=0;i< NN; i++)
    xnow[i]=xplus[i];
  // end of learning


  if (abs(pktNum-last_printed)>10) {
    last_printed=pktNum;
    //printf("pktnum: %d delta=% 5.2f avgr=% 5.2f r=% 5.2f v=% 5.2f u0=% 5.2f u1=% 5.2f u2=% 5.2f\n",pktNum, r_delta, rbar, reward, v_now , u[0],u[1],u[2]);    
  }
  getPktValues(pktNum,pkt_data);


  gettimeofday(&timeUpdate, NULL);
  double updateTime = (timeUpdate.tv_sec*1.-timeNow.tv_sec*1.)
		      + (timeUpdate.tv_usec*1.-timeNow.tv_usec)/1000000.;
  {//logging
    int c=sprintf(logstring,"%9.3f %9.3f %d %4.3f %4.3f %d %d %d %d %d %d %d",netTime, updateTime, iteration, rs[0], gammas[0], obs,act, dist, pktNum, action_prime, mode_duration,vid_count);
    for (int i=0;i<DEMONS;i++) 
      c+=sprintf(logstring+c," %4.3f",predictions[i]);
    if (action_prime!= STOP)
      iteration++;
    c+=sprintf(logstring+c," %d %d %d",era, learn_prediction, use_prediction);
    for (int k=0;k<data_size;k++) {
      c+=sprintf(logstring+c," %d",pkt_data[k]);
    }
    //loc += sprintf(start_comment+loc," deltaAC avgR R V Vplus u0 u1 u2");
    c+=sprintf(logstring+c," %f %f %f %f %f %f %f %f", r_delta, rbar, reward, v_now, v_plus, u[0], u[1], u[2] );
    c+=sprintf(logstring+c," ");
    for (int k=0;k<MM;k++) {
      *(logstring+c)='0'+selected_pixels[my_vc][k];
      c+=1;
    }
    sprintf(logstring+c,"\n");
    writelog(logstring);  
    strncpy(logstring2,logstring,LOGSTR_SZ);
  }

  //flip_era_if_needed();
  if (lastRemoteByte()==REMOTE_PAUSE) {
    endProgram(0);
  }
  if ((lastRemoteByte()==REMOTE_RIGHT) && (!want_snapshot)) {
    save_weights("./my_weights");
  }
  if (lastRemoteByte()==REMOTE_POWER) {
    load_weights("./my_weights");
  }  
  last_act=act;
}

//void flip_era_if_needed() {
//}

//---------------------------------- Learning infrastructure
void init_logging() {
  int ms_sleep=100;
  int num_lines=500;
  struct tm *tB = localtime(&(timeBegin.tv_sec));
  char logfilename[100];
  // I should log the source files into an associated directory.
  // This means making the directory, and copying the files into it
  // i.e. two calls to system.
  sprintf(logfilename,"mkdir %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src",
	  logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  int err1=system(logfilename);
  if (err1 < 0){
    printf("Failure to make directory: %s\n",logfilename);
    exit(1);
  }
  sprintf(logfilename,"cp Makefile *.c *.h %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src",
	  logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  int err2=system(logfilename);
  if (err2 < 0){
    printf("Failure to copy files: %s\n",logfilename);
    exit(1);
  }

  sprintf(logfilename,"%s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.logfile",
	  logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);

  char start_comment[5000];
  int loc=0;
  loc+=sprintf(start_comment+loc,"#Called with %d arguments: ",global_argc);
  for (int i=0;i<global_argc;i++) 
    loc +=sprintf(start_comment+loc," %s",global_argv[i]);
  loc+=sprintf(start_comment+loc,"\n");
  loc+=sprintf(start_comment+loc,"#Time %ld %ld y%d m%02d d%02d h%02d m%02d s%02d\n"
	       , timeBegin.tv_sec, timeBegin.tv_usec,
	       (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
	       tB->tm_hour,tB->tm_min,tB->tm_sec);
  loc+=sprintf(start_comment+loc,"#Global_constants: alpha=%f alpha2=%g lambda=%f prediction_threshold=%f use_prediction=%d learn_prediction=%d explore_on=%d exit_turn_on=%d use_random=%d alphaR=%f alphaU=%f alphaV=%f\n lambdaAC=%f",alpha,alpha2,lambda,prediction_threshold,use_prediction,learn_prediction,explore_on,exit_turn_on, use_random,alphaR,alphaU,alphaV,lambdaAC);
  loc += sprintf(start_comment+loc,"#Labels netTime updateTime iteration r gamma obs act dist pktNum action_prime  mode_duration vid_count");
  for (int i=0;i<DEMONS;i++)
    loc += sprintf(start_comment+loc," prediction_%d",i);
  loc += sprintf(start_comment+loc," era learn_prediction use_prediction");
  const char **names=getPacketNames();
  for (int i=0;names[i]!=NULL; i++,data_size=i) {
    loc += sprintf(start_comment+loc," %s",names[i]);
    
  }
  pkt_data=new int[data_size];
  loc += sprintf(start_comment+loc," deltaAC avgR R V Vplus u0 u1 u2");
  loc += sprintf(start_comment+loc," image");
  loc+=sprintf(start_comment+loc,"\n");
  start_log(logfilename, start_comment,LOGSTR_SZ, ms_sleep, num_lines);
}

void norobotmain(int argc,char ** argv){
  printf("no robot");
  for ( int i=0;i<argc;i++) 
    printf("%d:%s\n",i,argv[i]);
  printf("\n");
  agentInitialize();
    while (1) { // main agent loop
	usleep(30000);
	agentUpdateAfterAction(0, 0,0);
    }
    agentFuneral();
}

int main(int argc, char **argv) {
  global_argc=argc;
  global_argv=argv;
  gettimeofday(&timeBegin, NULL);
  init_video();
  init_logging();
  if (argc > 1 && 0==strcmp("-r",argv[1])) {
    norobotmain(argc,argv);
    return 0;
  }else {
  return bmain(argc,argv);
  }
}
