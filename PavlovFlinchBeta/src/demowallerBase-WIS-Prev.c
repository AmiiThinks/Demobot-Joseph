// choose to merge or separate the pavlovian and reward maximizing
// influences through a cascade.

// Write a report documenting the system

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

const char logfile_prefix[]="../../logfiles/demowallerBase-WIS";

int global_argc;
char**global_argv;

// MM is the number of bytes used from the image
#define MM 400
#define NUMACTIVEFEATURES MM
// RSHIFT provides a fast division by a power of 2
#define BYTE_RSHIFT 4
// The number of features per byte
#define BYTE_HAS_NUMFEATURES (1<< (8-BYTE_RSHIFT))

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

#define OBLIVIOUS 0
#define MODES 1
#define MODELESS 2
#define ACTOR_CRITIC 3
int policy_type=ACTOR_CRITIC;

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

float et[NND]; // eligibility trace
float w[NND];  // main parameter vector
float uu[NND];  // usage vector
float vv[NND];  // provisional parameters for usage
float d[NND];  // provisional parameters for w
float ww[NND];

float gammas[DEMONS], lambdaprevs[DEMONS], gammaprevs[DEMONS], predprevs[DEMONS];
float rs[DEMONS];
float rhos[DEMONS];
float Is[DEMONS]; // for GQ and WISTD
float Fs[DEMONS]; // for WISTD
float predictions[DEMONS];
const float alpha = 0.1/ MM;
const float alpha2 = alpha/1000.;
const float eta = 0.001/ NUMACTIVEFEATURES;   // recency-weighting factor
const float u_0 = NUMACTIVEFEATURES*50;                   // initial usage
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
float last_act_prob=1.;
float curr_act_prob=1.;
char start_comment[5000];


void agentInitialize(){
  setBackupOnBump(true);
  learn_prediction=1; 
  if (global_argc <=1) {
    printf("No args\n");
    printf("Default Pavlov mode\n");
    use_prediction=1;
    init_learning();
  } else {
    printf("argument: %s\n",global_argv[1]);
    //char c=global_argv[1][1];
  }
  
  int t_err = pthread_create(&tid, NULL, &mainimageloop, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create video thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }

  t_err = pthread_create(&net_tid, NULL, &mainNetLoop, NULL);
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

const char policytypes[][20]={"oblivious","modes","modeless","Actor-critic"};
const char *response_to_special_client_request(char *abuf,int *len2) {
  {
    const char cmd[]="info";
    if (0==strncmp(abuf,cmd,strlen(cmd))) {
      *len2=strlen(start_comment);
      return start_comment;
    }
  }
  {
    const char cmd[]="policy";
    if (0==strncmp(abuf,cmd,strlen(cmd))) {
      int val=0;
      if (1==sscanf(abuf+strlen(cmd),"%d",&val) && val >=0 && val <= 3) {
	policy_type=val;
      }
      const char *str_policy=policytypes[policy_type];
      *len2=strlen(str_policy);
      return str_policy;
    }
  }
  *len2=0;
  return NULL;
  
}
//------------ Image featuriztion

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
//-------------------- math helpers 
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

float uniform_random() {
  return (random()%10000)/10000.;
}

//--------------- externals



void process_remote_buttons(int s) {

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

//--------------- policies

float behaviour_selected_action_probability=1;
const float hit_decay_rate=0.95;
const float min_timesteps_to_turn=4;
const float hit_decay_turn_threshold=pow(hit_decay_rate,min_timesteps_to_turn);
const float timesteps_per_second=1.0/0.03;
const float expected_seconds_to_drive_after_cleared=0.3;
const float expected_timesteps_to_drive_after_cleared=expected_seconds_to_drive_after_cleared * timesteps_per_second;
const float beta_drive_after_cleared= fmin(1.0/expected_timesteps_to_drive_after_cleared , 1);

// The real_policy is also responsible for setting the probability 
// for the action selection
int oblivious_policy() {
  int action=FORWARD;
  curr_act_prob=1;
  if (hit_decay > hit_decay_turn_threshold) {
    action=COUNTERCLOCKWISE;
    curr_act_prob=1;
  }  else if (last_act==COUNTERCLOCKWISE) {
    if ( uniform_random() > beta_drive_after_cleared ) {
      action=COUNTERCLOCKWISE;
      curr_act_prob=1-beta_drive_after_cleared;
    } else {
      action=FORWARD;
      curr_act_prob=beta_drive_after_cleared;
    }
  }
  return action; 
}


int pavlovian_mode_policy() { 
  curr_act_prob=1;
  int action=FORWARD;
  if ( hit_decay >=1 || predictions[0] > prediction_threshold) {
    action=COUNTERCLOCKWISE;
    curr_act_prob=1;
  }  else if (last_act==COUNTERCLOCKWISE) {
    if (uniform_random() > beta_drive_after_cleared) {
      action=COUNTERCLOCKWISE;
      curr_act_prob=1-beta_drive_after_cleared;
    } else {
      action=FORWARD;
      curr_act_prob=beta_drive_after_cleared;
    }
  }
  return action;
}


int broken_modeless_policy() { 
//Does not work.  Gets stuck in a loop, and probabilities still get pushed high due to large e-trace vectors.
  // Am I really preserving E_b [rho] =1? 
  int action=FORWARD;
  curr_act_prob=1;
  if (bump_robot_observation) {
    action=COUNTERCLOCKWISE;
    curr_act_prob=1;
  } else {
    float q=1-predictions[0];//*predictions[0];
    q = (q > 1) ? 1 : (q < 0) ? 0  : q;
    if ( q < uniform_random()) {
      action=COUNTERCLOCKWISE;
      curr_act_prob=1-q;
    } else {
      action=FORWARD;
      curr_act_prob=q;
    }
  }
  return action; 
}


int pavlovian_modeless_policy() {
  float p0=predictions[0];

  float decisions_per_second=100/3.;
  float expected_switches_per_second=5;
  float T=decisions_per_second/expected_switches_per_second;
  int num_actions=2;
  float k_switch=log((T-1)*(num_actions-1));

  float k1=k_switch*4,k2=k_switch;//,k3=k1/2;
  
  float pref[3],weights[4];
  pref[0]=exp(-k1*(p0-0.5) + k2*(last_act==FORWARD));
  pref[1]=exp(k2*(last_act==COUNTERCLOCKWISE));
  pref[2]=exp(k2*(last_act==CLOCKWISE));
  //printf("\nmodeless prefs: %4.3f %4.3f \t k1=%4.3f k2=%4.3f p0=%4.3f %d\n",pref[0],pref[1], k1, k2, p0, last_act);
  
  float last=0;
  weights[0]=0;
  for (int i=0;i<num_actions;i++) { 
    weights[i+1]=last+pref[i];
    last=weights[i+1];
  }

  float roll=uniform_random()*last;
  int selection=-1;
  for (int i=0;i<num_actions;i++) {
    if (roll < weights[i+1]) {
      selection=i;
      curr_act_prob=(weights[i+1]-weights[0])/last;
      break;
    }
  }


	 
  int out= (selection==0) ? FORWARD : (selection==1) ? COUNTERCLOCKWISE: CLOCKWISE;       
  return out;
}
//------------------------------

float alphaR=.001,alphaU=.00001,alphaV=.00001;
float lambdaAC=0.8;

// number of features in the value function (critic)
#define nCritic 4
#define nActor 3
// equilibrium constant for the prediction of collision
#define CEQ 0.5


// log_e(T-1) for T=20/3 ... this should be timesteps to switch at equilibrium
// which is 100/3 and 5 switches per second -> T= 20/3
#define KS 1.7346
//float k_switch=log((T-1)*(num_actions-1));

float r_delta;
float rbar;

float xACprime[nCritic];
float xAC[nCritic];
float v[nCritic];
float eV[nCritic];
float v_now=0;
float v_plus=0;


float u[nActor];//={KS*4*(-CEQ), KS*4, KS};
float eU[nActor];


void ActorCriticInit() {
  //initial parameters
  u[0]=KS*4*(CEQ); // x[0] = bias
  u[1]=-KS*4;      // x[1] = last_act==FORWARD
  u[2]=KS;        // x[2] = last_act==COUNTERCLOCKWISE

  eU[0]=eU[1]=eU[2]=0;
  v[0]=v[1]=v[2]=v[3]=0;
  eV[0]=eV[1]=eV[2]=eV[3]=0;
}

double piAC(int actK, float x[]) {
  float p[2];
  p[0]= exp(u[0]*x[0] + u[1]*x[1] + u[2]*x[2]); // exp-linear form
  p[1]= exp(u[2]*x[3]);
  //p[2]= exp(u[2]*x[4]);
  //printf("Actor prefs: %4.3f %4.3f \t %4.3f %4.3f %4.3f\n",p[0],p[1],u[0],u[1],u[2]);
  float tot=p[0]+p[1];//+p[2];
  return p[actK]/tot;
}

int actor_critic_policy() {
  //pavlovian_modeless_policy();
  xACprime[0]=1;
  xACprime[1]=predictions[0];
  xACprime[2]=last_act==FORWARD;
  xACprime[3]=last_act==COUNTERCLOCKWISE;

  float pi0=piAC(0,xACprime);
  float val=uniform_random();
  int out=-1;
  if (val < pi0) {
    out=FORWARD;
    curr_act_prob=pi0;
  } else {
    out=COUNTERCLOCKWISE;
    curr_act_prob=1-pi0;
  }
  return out;
}

double grad_X(int actK,int uI, float x[] ) {
  //compute the gradient for our given policy (where are the zeros?)
  if (2==uI)
    return 1-piAC(0,x)*x[2] -piAC(1,x)*x[3];// -pi(2,x)*x[4] ;
  if (0==actK) {
    if (0==uI) return (1-piAC(0,x))*x[0];
    //if (1==uI) 
    return (1-piAC(0,x))*x[1];
  }
  if (0==uI) 
    return -piAC(0,x)*x[0];
  return -piAC(0,x)*x[1];
  
}

void ActorCriticUpdate(float reward, int actK) {
  xACprime[0]=1;
  xACprime[1]=predictions[0];
  xACprime[2]=last_act==FORWARD;
  xACprime[3]=last_act==COUNTERCLOCKWISE;
  
  v_now=dot(xAC,v,nCritic);
  v_plus=dot(xACprime,v,nCritic);
  float delta = reward- rbar + v_plus - v_now;
  
  // avg
  r_delta=delta;
  rbar += alphaR* delta; 
  //critic
  for (int i=0;i<nCritic;i++) {
    eV[i] = lambdaAC * eV[i]  + xAC[i];
    v[i] += alphaV * delta *eV[i];
  }
  //actor
  for (int i=0;i<nActor;i++) {
    eU[i] = lambdaAC* eU[i] + grad_X(actK,i,xAC);
    u[i] += alphaU * delta * eU[i];
  }

  
  for (int i=0;i<nCritic;i++)
    xAC[i]=xACprime[i];

}

//------------------------------


int real_policy() {
  if (ACTOR_CRITIC==policy_type)
    return actor_critic_policy();
  if (MODELESS==policy_type)
    return pavlovian_modeless_policy();
  if (MODES==policy_type)
    return pavlovian_mode_policy();
  return oblivious_policy();
}
//--------------------------
int known_bump=false;

int agentSelectAction(int s) {
  if (s !=0) 
    hit_decay =1;
  else
    hit_decay *=hit_decay_rate;

  //remote
  process_remote_buttons(s);

  //video
  select_vid();
  fill_features(my_vc,xplus);
  for (int i=0;i<DEMONS;i++)
    predictions[i]=make_prediction(xplus,i);
  
  //agent state
  bump_robot_observation=s;
  if (bump_robot_observation && !known_bump)
     play_hit();
  known_bump = bump_robot_observation;

  if (action_prime==STOP)
    return action_prime;

  //select action
  action_prime=real_policy();
  return action_prime;
}


//----------------  Initialization of weights and serialization to disk


void init_learning() {
  for (int i=0;i<NN;i++) {
    xplus[i]=0;xnow[i]=0;
  }  
  for (int i=0;i<NND;i++) {
    et[i]=0;w[i]=0;ww[i]=0;
    uu[i]=u_0;
    vv[i]=0;d[i]=0;
  }
  
  for (int i=0;i<DEMONS;i++) {
    lambdaprevs[i]=gammaprevs[i]=predprevs[i]=0.0;
    Fs[i] = 0;
  }
  
  ActorCriticInit();
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

//------------- Core prediction code

float make_prediction(int x[],int demon){
  float lprediction=0;
  int k=demon*NN;
  for (int i=0;i< NN; i++) {
    float q=w[k+i] *x[i];
    lprediction += q;
  }
  return lprediction;
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

// Mahmood, Sutton 2015, UAI, WIS-TD(lambda)
void WISTDupdate(int demon) {
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  int offset=k*NN;
  float alpha;
  float lambdaprev = lambdaprevs[k];
  float gammaprev  = gammaprevs[k];
  float predprev   = predprevs[k];
  
  //WIS-TD(lambda) with rho
  if (learn_prediction) {
    
    float pred      = idot(xnow, w+offset, NN);
    float prednext  = idot(xplus, w+offset, NN);
    float dx        = idot(xnow, d+offset, NN);
    float etx       = idot(xnow, et+offset, NN);
    //printf("offset %d: pred=%10.6f prednext=%10.6f dx=%10.6f ex=%10.6f\n",offset,pred,prednext,dx,etx);
    for (int i=0;i< NN; i++) {
      // update of usage vector
      uu[i+offset]     += - eta*xnow[i]*xnow[i]*uu[i+offset] \
      + rho*xnow[i]*xnow[i] \
      + (rho-1)*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset];
      // update of provisional parameters for u
      vv[i+offset]      = rho*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset] + rho*xnow[i]*xnow[i];
      // setting the vector step size according to usage
      alpha            = uu[i+offset]!=0? 1/uu[i+offset] : 0;
      // eligibility trace update
      et[i+offset]     = rho*alpha*xnow[i] \
      + rho*lambdaprev*gammaprev*(et[i+offset] - rho*alpha*xnow[i]*etx);
      // update of the main weight vector
      w[i+offset]     += alpha*rho*(predprev - pred)*xnow[i]  \
      + (r + gamma*prednext - predprev)*et[i] \
      + (rho-1)*lambdaprev*gammaprev*(d[i+offset] - rho*alpha*xnow[i]*dx);
      // update of the provisional parameters of w
      d[i+offset]      = rho*lambdaprev*gammaprev*(d[i+offset] - rho*alpha*xnow[i]*dx)  \
      + (r + prednext - predprev)*et[i+offset];
      
      //printf("%d %d: u=%10.6f v=%10.6f alpha=%10.6f e=%10.6f w=%10.6f d=%10.6f\n", i, offset, uu[i+offset], vv[i+offset], alpha, et[i+offset], w[i+offset], d[i+offset]);
    }
    
    predprevs[k]      = prednext;
    lambdaprevs[k]    = lambda;
    gammaprevs[k]     = gamma;
  }
}

// Mahmood, Sutton 2015, UAI, WIS-GTD(lambda)
void WISGTDupdate(int demon) {
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  int offset=k*NN;
  float alpha;
  float lambdaprev = lambdaprevs[k];
  float gammaprev  = gammaprevs[k];
  float predprev   = predprevs[k];
  
  if (learn_prediction) {
    
    float pred      = idot(xnow, w+offset, NN);
    float prednext  = idot(xplus, w+offset, NN);
    float dx        = idot(xnow, d+offset, NN);
    float etx       = idot(xnow, et+offset, NN);
    float ew        = dot(et+offset,ww+offset , NN);
    float xw        = idot(xnow,ww+offset,NN);
    //printf("offset %d: pred=%10.6f prednext=%10.6f dx=%10.6f ex=%10.6f\n",offset,pred,prednext,dx,etx);
    for (int i=0;i< NN; i++) {
      // update of usage vector
      uu[i+offset]     += - eta*xnow[i]*xnow[i]*uu[i+offset] \
      + rho*xnow[i]*xnow[i] \
      + (rho-1)*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset];
      // update of provisional parameters for u
      vv[i+offset]      = rho*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset] + rho*xnow[i]*xnow[i];
      // setting the vector step size according to usage
      alpha            = uu[i+offset]!=0? 1/uu[i+offset] : 0;
      // eligibility trace update
      et[i+offset]     = rho*(xnow[i] \
      + lambdaprev*gammaprev*et[i+offset]);
      // update of the main weight vector
      w[i+offset]     += alpha*(r + gamma*prednext - pred)*et[i] \
                      - alpha*gamma*(1-lambda)*ew*xplus[i];
      ww[i+offset] += alpha2 * (delta * et[i+offset] - xw * xnow[i]);
      
      //printf("%d %d: u=%10.6f v=%10.6f alpha=%10.6f e=%10.6f w=%10.6f d=%10.6f\n", i, offset, uu[i+offset], vv[i+offset], alpha, et[i+offset], w[i+offset], d[i+offset]);
    }
    
    predprevs[k]      = prednext;
    lambdaprevs[k]    = lambda;
    gammaprevs[k]     = gamma;
  }
}

// Combining WIS step size with Emphatic TD
void WISETDupdate(int demon) {
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  int offset=k*NN;
  float alpha;
  float lambdaprev  = lambdaprevs[k];
  float gammaprev   = gammaprevs[k];
  float predprev    = predprevs[k];
  float I           = Is[k];
  float F           = Fs[k];
  
  float M;
  
  if (learn_prediction) {
    
    float pred      = idot(xnow, w+offset, NN);
    float prednext  = idot(xplus, w+offset, NN);
    
    F       += I;
    M        = lambdaprev*I + (1-lambdaprev)*F;
    
    for (int i=0;i< NN; i++) {
      // update of usage vector
      uu[i+offset]     += - eta*xnow[i]*xnow[i]*uu[i+offset] \
      + rho*M*xnow[i]*xnow[i] \
      + (rho-1)*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset];
      // update of provisional parameters for u
      vv[i+offset]      = rho*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*vv[i+offset] + rho*M*xnow[i]*xnow[i];
      // setting the vector step size according to usage
      alpha            = uu[i+offset]!=0? 1/uu[i+offset] : 0;
      // eligibility trace update
      et[i+offset]     = rho*(M*xnow[i] \
                              + lambdaprev*gammaprev*et[i+offset]);
      // update of the main weight vector
      update = alpha*(r + gamma*prednext - pred)*et[i];
      if (update>0.1) update = 0.1;
      else if (update<-0.1) update = -0.1;
      w[i+offset]     += update;      
    }
    
    F       *= gamma*rho;
    
    predprevs[k]      = prednext;
    lambdaprevs[k]    = lambda;
    gammaprevs[k]     = gamma;
    Fs[k]             = F;
  }
}

void GQupdate(int demon) {
  // Implements pseudocode in the GQ(lambda) Users Guide
  //GQ (without action dependence in the features).  Note that "I" plays a
  //similar role to rho in GTD
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  float I=Is[k];
  int offset=k*NN;
  delta=0;
  
  //GQ(lambda) with rho and I
  if (learn_prediction) {
    // set variable gamma going to zero on r.
    delta = r;
    for (int i=0;i< NN; i++)
      delta += gamma * w[i+offset] * xplus[i]  - w[i+offset] * xnow[i];
    for (int i=0;i<NN;i++) {
      et[i+offset] =rho * et[i+offset] + I*xnow[i];
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

//------------- end of core prediction code

int last_printed=0;

double netTime;
void updateLogfile(int obs, int act, int pktNum);
void updateLearning(int obs,int act);
void updateRemoteButtons();
void updateLightsAndSound();
int dist;
float reward;

void agentUpdateAfterAction(int obs, int act,int pktNum) { //post policy
  gettimeofday(&timeNow, NULL);
  int time_delta=timeNow.tv_sec*1.-timeBegin.tv_sec*1.;
  netTime = time_delta+ (timeNow.tv_usec*1.-timeBegin.tv_usec)/1000000.;
  updateLightsAndSound();
  updateLearning(obs,act);
  getPktValues(pktNum,pkt_data);
  updateLogfile(obs,act,pktNum);
  updateRemoteButtons();
  last_act=act;
  last_act_prob=curr_act_prob;
  
  //printf("pkt: %d u_0: %10.6f\n ",pktNum,u_0);
}


//const float min_probability_for_learning=0.5;
const float min_prob_action=0.3;
void updateLearning(int obs,int act){
  rs[0] =obs > 0 ? 1 : 0;
  gammas[0]=rs[0]>0 ? 0 : 1-3./100.;
  Is[0]=1;
  //rhos[0]=last_act==FORWARD && last_act_prob > min_probability_for_learning? 1.0/last_act_prob: 0;
  //rhos[0]=last_act==FORWARD ? 1 :0;

  //float v=uniform_random();
  if (last_act !=FORWARD)
    rhos[0]=0; // target
  else
    rhos[0] = 1./last_act_prob;
  /*
    if (last_act_prob >= min_prob_action)
    rhos[0] = 1./last_act_prob; // standard rho
  else if (v > min_prob_action* last_act_prob) {
    // GQ with rho=0,I=0 and no action-dependent features is GTD with rho=0
    rhos[0]=0; //We want no influence on earlier states from this continuation 
    Is[0]=0;
  } else if (v < last_act_prob) {
    rhos[0]=1./last_act_prob;
  } else {
    rhos[0]=0;
  }
*/
  for (int k=0;k<DEMONS;k ++) {
    //GQupdate(k);
    //WISTDupdate(k);
    //GTDupdate(k);
    //WISGTDupdate(k);
    WISETDupdate(k);
  }
  dist= getDistanceReward();
  reward=dist;// - 100 * rs[0];
  ActorCriticUpdate(reward, act);

  for (int i=0;i< NN; i++)
    xnow[i]=xplus[i];
}


void updateLightsAndSound(){
  notes(netTime,predictions[0]);
  if (startingLights >0) {
    setLEDs(255,255, 1 , 1);
    startingLights--;
  }
  //  setExtLEDs(predictions[0] > .36 ,  predictions[0] > .5 ,predictions[0] > .18 );
}

void updateRemoteButtons(){
  if (lastRemoteByte()==REMOTE_PAUSE) {
    endProgram(0);
  }
  if ((lastRemoteByte()==REMOTE_RIGHT) && (!want_snapshot)) {
    save_weights("./my_weights");
  }
  if (lastRemoteByte()==REMOTE_POWER) {
    load_weights("./my_weights");
  }  
}

//---------------------------------- Logging infrastructure

void updateLogfile(int obs, int act, int pktNum){

  gettimeofday(&timeUpdate, NULL);
  double updateTime = (timeUpdate.tv_sec*1.-timeNow.tv_sec*1.)
		      + (timeUpdate.tv_usec*1.-timeNow.tv_usec)/1000000.;
  {//logging
    int c=sprintf(logstring,"%9.3f %9.3f %d %4.3f %4.3f %d %d %d %d %d %d %d",netTime, updateTime, iteration, rs[0], gammas[0], obs,act, dist, pktNum, action_prime, mode_duration,vid_count);
    for (int i=0;i<DEMONS;i++) 
      c+=sprintf(logstring+c," %4.3f",predictions[i]);
    if (action_prime!= STOP)
      iteration++;
    c+=sprintf(logstring+c," %d %d %d %d %10.6f %10.6f",era, learn_prediction, use_prediction, policy_type,curr_act_prob,rhos[0]);
    for (int k=0;k<data_size;k++) {
      c+=sprintf(logstring+c," %d",pkt_data[k]);
    }
    //loc += sprintf(start_comment+loc," deltaAC avgR R V Vplus u0 u1 u2");
    //c+=sprintf(logstring+c," %f %f %f %f %f %f %f %f", r_delta, rbar, reward, v_now, v_plus, u[0], u[1], u[2] );
    c+=sprintf(logstring+c," ");
    for (int k=0;k<MM;k++) {
      *(logstring+c)='0'+selected_pixels[my_vc][k];
      c+=1;
    }
    sprintf(logstring+c,"\n");
    writelog(logstring);  
    strncpy(logstring2,logstring,LOGSTR_SZ);
  }

}


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

  int loc=0;
  loc+=sprintf(start_comment+loc,"#Called with %d arguments: ",global_argc);
  for (int i=0;i<global_argc;i++) 
    loc +=sprintf(start_comment+loc," %s",global_argv[i]);
  loc+=sprintf(start_comment+loc,"\n");
  loc+=sprintf(start_comment+loc,"#Time %ld %ld y%d m%02d d%02d h%02d m%02d s%02d\n"
	       , timeBegin.tv_sec, timeBegin.tv_usec,
	       (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
	       tB->tm_hour,tB->tm_min,tB->tm_sec);
  loc+=sprintf(start_comment+loc,"#Global_constants: alpha=%f alpha2=%g lambda=%f prediction_threshold=%f use_prediction=%d learn_prediction=%d explore_on=%d exit_turn_on=%d use_random=%d bytes_from_image=%d byte_rshift=%d features_per_byte=%d\n",
	       //"// alphaR=%f alphaU=%f alphaV=%f\n lambdaAC=%f",
	       alpha,alpha2,lambda,prediction_threshold,use_prediction,learn_prediction,explore_on,exit_turn_on, use_random, MM, BYTE_RSHIFT, BYTE_HAS_NUMFEATURES);
  //,alphaR,alphaU,alphaV,lambdaAC);
  loc += sprintf(start_comment+loc,"#Labels netTime updateTime iteration r gamma obs act dist pktNum action_prime  mode_duration vid_count");
  for (int i=0;i<DEMONS;i++)
    loc += sprintf(start_comment+loc," prediction_%d",i);
  loc += sprintf(start_comment+loc," era learn_prediction use_prediction policy_type curr_act_prob rho");
  const char **names=getPacketNames();
  for (int i=0;names[i]!=NULL; i++,data_size=i) {
    loc += sprintf(start_comment+loc," %s",names[i]);
    
  }
  pkt_data=new int[data_size];
  //  loc += sprintf(start_comment+loc," deltaAC avgR R V Vplus u0 u1 u2");
  loc += sprintf(start_comment+loc," image");
  loc+=sprintf(start_comment+loc,"\n");
  start_log(logfilename, start_comment,LOGSTR_SZ, ms_sleep, num_lines);
}

int main(int argc, char **argv) {
  global_argc=argc;
  global_argv=argv;
  gettimeofday(&timeBegin, NULL);
  init_video();
  init_logging();
  return bmain(argc,argv);
}
