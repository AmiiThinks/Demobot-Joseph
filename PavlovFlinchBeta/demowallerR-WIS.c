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

const char logfile_prefix[]="../logfiles/demowallerR-WIS";

int global_argc;
char**global_argv;

// number of bytes
#define MM 400
#define NOFFSETS 1000

#define BYTE_MULTIPLIER 32
#define BYTE_RSHIFT 3
// number of number of image features
#define NN  MM*32


#define DEMONS 1
#define NND    NN*DEMONS


struct timeval timeBegin, timeNow,timeVid,timeUpdate;
int timestep=0;
// Agent thread code

int last_act=-1;
bool make_noise=true;
void  play_explore();
void  play_hit();
float make_prediction(int x[], int ind);
void init_learning() ;
void load_weights(const char *filename);
void save_weights(const char *filename);
void select_vid();
void fill_features(int vc,int x[]);
int my_vc=0;
int xplus[NN];
int xnow[NN];

float lambdaprev=0.0, gammaprev=0.0;
float et[NND]; // eligibility trace
float w[NND];  // main parameter vector
float u[NND];  // usage vector
float v[NND];  // provisional parameters for usage
float d[NND];  // provisional parameters for w
float predprev = 0.0;

float gammas[DEMONS];
float rs[DEMONS];
float rhos[DEMONS];
float predictions[DEMONS];
const float u_0 = MM;         // initial usage
const float eta = 0.01/ u_0;  // recency-weighting factor
const float lambda = .9;

#define LOGSTR_SZ 3000
char logstring[LOGSTR_SZ];
char logstring2[LOGSTR_SZ]="";
int iteration=-1;
int era=0;
const float prediction_threshold=0.505;

float delta, rho;

int pixelsize=120*160*2;
int pixelinds[NOFFSETS]
={4396, 7882, 6214, 13401, 28419, 31917, 32134, 18636, 35215, 32733, 17001, 28236, 37547, 29600, 10230, 17256, 13026, 28199, 19887, 16305, 5131, 34770, 31807, 34762, 994, 5756, 15158, 34807, 25817, 35689, 18061, 753, 4491, 20295, 16044, 4494, 9172, 21702, 29955, 16312, 9451, 34135, 26715, 12689, 6049, 11524, 14897, 10725, 7943, 29762, 36939, 6465, 28875, 1671, 37430, 36707, 1110, 5932, 32154, 27069, 3776, 2489, 87, 22306, 10756, 27424, 9250, 30892, 35971, 27194, 255, 31169, 35891, 24148, 16925, 22291, 7420, 16760, 181, 26446, 13148, 34399, 9438, 26849, 11387, 4548, 9239, 13812, 13278, 25713, 14639, 5250, 33887, 22156, 22649, 21075, 23485, 14706, 4704, 6309, 30766, 13037, 11146, 16404, 14060, 29567, 1384, 11647, 30734, 18308, 14824, 23641, 27735, 3249, 1177, 14485, 19151, 35519, 25088, 30981, 2461, 19614, 1039, 15873, 5025, 1013, 12441, 14343, 28887, 4986, 31221, 33584, 23976, 17775, 13964, 14560, 30169, 31913, 26700, 7195, 2697, 37297, 13316, 18757, 37231, 26663, 30813, 29107, 21802, 31727, 6146, 38151, 37761, 25041, 7224, 20403, 26998, 29900, 22255, 29592, 32319, 9194, 28069, 3966, 27491, 9379, 35652, 22584, 12765, 17029, 7536, 34966, 19301, 10062, 10328, 27879, 10015, 20860, 12057, 19434, 22527, 24614, 17937, 5556, 36450, 13438, 2706, 30320, 28245, 24743, 27617, 15648, 5992, 19400, 35174, 33711, 35991, 34072, 20575, 31741, 3802, 20710, 20986, 19109, 1596, 25573, 26511, 16242, 24453, 11205, 24915, 8455, 18781, 23112, 5898, 4877, 26103, 18547, 6862, 21186, 4507, 23079, 3221, 12754, 19762, 24923, 10987, 23907, 31034, 9378, 19392, 37894, 34264, 29556, 1738, 29844, 8239, 25246, 900, 25389, 38084, 34261, 13726, 38059, 5559, 26304, 36274, 16534, 30226, 37288, 35524, 376, 12081, 10887, 35406, 35718, 23750, 20004, 34760, 10601, 28366, 9849, 32307, 9201, 19042, 17607, 596, 30690, 14447, 14999, 7756, 27033, 24238, 7806, 20215, 4314, 34895, 20278, 35401, 9673, 21447, 33864, 22681, 24034, 2386, 33932, 38345, 26296, 33560, 2023, 20475, 5904, 30999, 32504, 38142, 4634, 35926, 3007, 4343, 16626, 38393, 2805, 15398, 1524, 27218, 13994, 23790, 22107, 5921, 19982, 25603, 2645, 34802, 24319, 18747, 34590, 9217, 9401, 7022, 32726, 27993, 31498, 33980, 28880, 30707, 20817, 21480, 35209, 8160, 10507, 23163, 17330, 33686, 15332, 7131, 3912, 2596, 28116, 16753, 1086, 30623, 803, 21417, 35614, 15789, 4961, 2765, 26496, 24766, 6888, 733, 13878, 18325, 9730, 23889, 25333, 2505, 30038, 24993, 35649, 26881, 30672, 6381, 9168, 18963, 21540, 16461, 702, 11520, 36926, 2631, 6079, 37763, 5600, 11778, 31895, 12512, 12960, 26842, 38277, 36502, 36299, 33994, 34042, 31448, 17723, 38238, 5211, 36599, 11266, 21977, 33033, 31577, 14848, 26538, 31738, 4990, 10900, 38108, 16886, 25417, 34594, 31135, 5104, 23881, 28449, 30948, 36421, 22537, 37964, 31761, 35618, 27856, 10950, 3898, 36565, 11079, 37190, 18504, 2036, 2421, 18818, 24767, 3878, 17731, 31887, 4847, 36774, 34029, 16518, 15701, 2015, 17263, 22283, 19733, 11109, 12759, 23236, 5918, 17580, 31131, 2217, 2014, 19587, 24891, 21579, 30026, 19944, 20157, 21406, 36297, 6669, 5701, 20715, 24859, 20031, 8286, 11686, 29555, 27951, 29050, 33070, 33185, 36898, 30146, 29755, 31531, 21366, 26883, 26185, 2453, 4828, 27517, 20205, 14219, 22308, 19554, 15780, 21274, 10157, 38220, 21994, 12320, 10742, 1, 37023, 13068, 22799, 9937, 18397, 28375, 4157, 12593, 15791, 38016, 2707, 9519, 10443, 23312, 5273, 6045, 6187, 14531, 17269, 35481, 15236, 27798, 867, 28943, 28712, 23031, 81, 15171, 8195, 11096, 32634, 17587, 8750, 25932, 4807, 19789, 10374, 16703, 26943, 11770, 2139, 20147, 17412, 2998, 7062, 17072, 620, 19220, 23254, 9292, 23258, 3507, 16122, 14970, 4262, 12033, 31532, 16600, 29381, 5817, 23186, 25149, 35467, 34673, 903, 17811, 36294, 20311, 2246, 19002, 24208, 21306, 20088, 21263, 18202, 3405, 29741, 3698, 17841, 17927, 31679, 12007, 4637, 25384, 28339, 7954, 23405, 29746, 17689, 24896, 21899, 34805, 35080, 20646, 29899, 19433, 24507, 27747, 35046, 21764, 3101, 2978, 26864, 33803, 5942, 28892, 32836, 36484, 19746, 12144, 32449, 30900, 37975, 7351, 37506, 8222, 1300, 29667, 32001, 18771, 34251, 35360, 5809, 3673, 37659, 37228, 3397, 29252, 33024, 17918, 26934, 8637, 25361, 799, 1915, 8763, 22739, 33269, 35466, 18262, 20435, 950, 34840, 21861, 1744, 32507, 25562, 12171, 15578, 16912, 38010, 13654, 14986, 31430, 18395, 4098, 36127, 38333, 20894, 7089, 34879, 1910, 35605, 12890, 31706, 29763, 38172, 24494, 30271, 7968, 17174, 21011, 1122, 35331, 16173, 21608, 12672, 29367, 163, 7030, 12967, 11905, 8624, 26344, 34114, 16679, 18762, 36995, 29976, 12839, 36347, 30373, 7773, 7302, 12686, 34814, 29356, 20606, 30292, 19773, 20988, 10027, 16420, 7972, 13720, 31566, 3292, 31188, 30722, 19867, 35803, 17842, 11662, 34107, 7596, 3506, 37665, 10709, 237, 36511, 9419, 37273, 25387, 12552, 37034, 37969, 23697, 6871, 14596, 23393, 1870, 12317, 14115, 2226, 7629, 8272, 8665, 19844, 32816, 4127, 7804, 37632, 25636, 27726, 8450, 16896, 31877, 15174, 10313, 27645, 36568, 9731, 15383, 10819, 30906, 11802, 28715, 10318, 14205, 21719, 7649, 14495, 36232, 10000, 28697, 31686, 25998, 7477, 17215, 36135, 34724, 3133, 6460, 31747, 12231, 8742, 4282, 14833, 8965, 3289, 17542, 5800, 26347, 19775, 28725, 1765, 29059, 14828, 12264, 7202, 36893, 25270, 30089, 36840, 28639, 35740, 10085, 36182, 32273, 20984, 28711, 8388, 19062, 30805, 24798, 27538, 10407, 4428, 37519, 17493, 9907, 4675, 11042, 21499, 28384, 37965, 830, 6153, 26509, 20513, 19957, 21026, 25026, 36738, 12490, 2899, 66, 32386, 31863, 4822, 2688, 13508, 30003, 5538, 8373, 14372, 2377, 24347, 19235, 21901, 11150, 25495, 15098, 31445, 14876, 37611, 26570, 4203, 7145, 4133, 2935, 16082, 29001, 8242, 22146, 5637, 36998, 21470, 31665, 34704, 38125, 4973, 31343, 23569, 13514, 10926, 34025, 24958, 33734, 2852, 29614, 35764, 13286, 9636, 22693, 16159, 3834, 24074, 26260, 76, 12548, 22561, 10379, 17467, 15677, 29709, 31184, 22068, 22742, 14156, 437, 33206, 36517, 185, 36844, 6394, 18392, 6768, 20296, 12524, 15113, 15409, 26167, 2831, 37017, 5198, 20921, 9792, 34800, 11391, 37164, 3323, 21801, 809, 34557, 10862, 23806, 1769, 20573, 36525, 9857, 13490, 22680, 721, 14831, 34330, 25606, 22968, 28247, 27072, 36222, 955, 4985, 15205, 12113, 9644, 2330, 8250, 297, 37885, 5746, 28127, 27670, 9745, 26075, 3407, 11442, 37615, 12201, 34699, 5541, 585, 5427, 6211, 12845, 26212, 8471, 10749, 16425, 37601, 18911, 13933, 31740, 24519, 19649, 17695, 26080, 7153, 34870, 3935, 35106, 16556, 22843, 23968, 2190, 5946, 27225, 14820, 732, 2334, 26736, 25721, 28426, 16790, 29288, 28043, 17336, 9050, 18839, 1839, 21792, 35139, 27325, 37806, 33571, 27594, 35806, 22396, 12474, 26432, 5887, 23846, 15261, 21611, 17000, 32878, 852, 10768, 25316, 9454, 14702, 12777, 5230, 13757, 7223};
int selected_pixels[2][MM];


pthread_t tid;
pthread_t net_tid;
void * mainNetLoop(void*);

int use_prediction=0;
int use_random=0;
int learn_prediction=0;
void SarsaInit();
void flip_era_if_needed();
void agentInitialize(){
  SarsaInit();
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
    //printf("%x ",selected_pixels[i]);
  }
  //unlock
}


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

float uniform_random() {
  return (random()%10000)/10000.;
}

float hit_decay=0;
//--------------- Modeless support

int startingLights=0;

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
    make_noise=0;
    startingLights=10;
  } else  if (q==REMOTE_MAX ) {
    init_learning();
    use_prediction = true;
  } else if (q==REMOTE_CLEAN ) {
    use_prediction = false;
  }
}

int bump_robot_observation;
int SarsaSelect();

int real_policy() {
  //return SarsaSelect();
  if (bump_robot_observation >0 ||
      (last_act == CLOCKWISE && uniform_random() > .05) ) 
    return CLOCKWISE;
  return FORWARD;
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

  // base unlearned behaviors
  if (bump_robot_observation)
    play_hit();
  if (action_prime==STOP)
    return action_prime;

  //select action
  action_prime=real_policy();
  return action_prime;
}


//----------------
double elapsedTime;
bool mute=false;
bool rc_space=false;


void init_learning() {
  for (int i=0;i<NN;i++) {
    xplus[i]=0;xnow[i]=0;
  }
  for (int i=0;i<NND;i++) {
    et[i]=0;w[i]=0;//ww[i]=0;
  }
}

void load_weights(const char *filename){
  FILE *f=fopen(filename,"r");
  for (int i=0;i<NND;i++ )
    if (1!=fscanf(f,"%f",w+i)) {
      printf("Failed to read weights from file %s at weight %d.\n",filename,i);
      exit(0);
    }
  //  for (int i=0;i<NND;i++ )
  //  if (1!=fscanf(f,"%f",ww+i)) {
  //    printf("Failed to read weights from file %s at weight %d.\n",filename,i);
  //    exit(0);
  //  }
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
    int k=i*BYTE_MULTIPLIER + s[i];
    x[k] = 1;
  }
}

int lastSongTime=0;
int song[15];
void notes(int netTime, float prediction) {
    if (lastRemoteByte()==REMOTE_LEFT)
      make_noise=!make_noise;
    if (netTime>lastSongTime + 3) {
      int q=prediction*70 + 30;
      song[0]= q > 100 ? 100 : q < 31 ? 31 : q;
      song[1]=8;
      printf ("pred: %f q: %d song: %d\n",prediction,q, song[0]);
      if (!make_noise) {
	//printf ("quiet\n.");
      }else {
	playSong(1, song);
      }
      lastSongTime=netTime;
    } else {
      //printf ("times: %d %d",netTime,lastSongTime);
    }
  }

void play_explore() {
  if (!make_noise)
    return;
  song[0]=70;
  song[1]=7;
  song[2]=80;
  song[3]=7;
  playSong(3, song);
}

void play_none() {
}

static int trek[]=
  {62, 32, 67, 16, 72, 48,
   128, 2, 71, 22, 67, 22,
   64, 22, 69, 22, 74, 64};

void play_switch() {
  playSong(9, trek);
}

void play_hit() {
  song[0]=60;
  song[1]=10;
  playSong(1, song);
}

void play_nohit() {
  if (!make_noise)
    return;
  song[0]=100;
  song[1]=10;
  playSong(1, song);
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
int data_size=0;
int *pkt_data;


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

void WISTDupdate(int demon) {
  // only works for a single demon to preserve predprev, lambdaprev and gammaprev.
  int k=demon;
  float r=rs[k];
  float gamma=gammas[k];
  float rho=rhos[k];
  int offset=k*NN;
  float alpha[NND];

  //WIS-TD(lambda) with rho
  if (learn_prediction) {

    float pred      = idot(xnow, w+offset, NN);
    float prednext  = idot(xplus, w+offset, NN);
    float dx        = idot(xnow, d+offset, NN);
    float etx       = idot(xnow, et+offset, NN);

    for (int i=0;i< NN; i++) {
      // update of usage vector
      u[i+offset]     += - eta*xnow[i]*xnow[i]*u[i+offset] \
                        + rho*xnow[i]*xnow[i] \
                        + (rho-1)*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*v[i+offset];
      // update of provisional parameters for u
      v[i+offset]      = rho*lambdaprev*gammaprev*(1-eta*xnow[i]*xnow[i])*v[i+offset] + rho*xnow[i]*xnow[i];
      // setting the vector step size according to usage
      alpha[i+offset]  = u[i+offset]!=0? 1/u[i+offset] : 0;
      // eligibility trace update
      et[i+offset]     = rho*alpha[i+offset]*xnow[i] \
                        + rho*lambdaprev*gammaprev*(et[i+offset] - rho*alpha[i+offset]*xnow[i]*etx);
      // update of the main weight vector
      w[i+offset]     += alpha[i+offset]*rho*(predprev - pred)*xnow[i]  \
                        + (r + gamma*prednext - predprev)*et[i] \
                        + (rho-1)*lambdaprev*gammaprev*(d[i+offset] - rho*alpha[i+offset]*xnow[i]*dx);
      // update of the provisional parameters of w
      d[i+offset]      = rho*lambdaprev*gammaprev*(d[i+offset] - rho*alpha[i+offset]*xnow[i]*dx)  \
                        + (r + prednext - pred)*et[i+offset];
    }

    predprev      = prednext;
    lambdaprev    = lambda;
    gammaprev     = gamma;
  }
}

float alphaR=.01,alphaU=.00001,alphaV=.00001;
float alphaAC=.1/2;
float lambdaAC=0.8;

const int numAct=2;

#define nAC 4

float xACprime[nAC];
float xAC[nAC];
float wAC[numAct][nAC];

void SarsaInit() {
  for (int i=0;i<nAC;i++) {
    xAC[i]=xACprime[i]=0;
    for (int j=0;j<numAct;j++)
      wAC[j][i]=0;
  }
}

void loadxACprime(){
  xACprime[0]=int(predictions[0] <= 0.5);
  xACprime[1]=int(predictions[0] > 0.5);
  xACprime[2]=int(bump_robot_observation >0);
  xACprime[3]=int(bump_robot_observation ==0);
}

float action_prime_probability=0.0;
float last_action_probability=0.0;
float epsilon=.01;

int SarsaSelect() {
  loadxACprime();
  last_action_probability=action_prime_probability;
  float max=dot(wAC[0],xACprime,nAC);
  int maxi=0;
  for (int i =1;i<numAct;i++) {
    float val=dot(wAC[i],xACprime,nAC);
    if (val > max) {
      max=val; maxi=i;
    }
  }

  int sel=maxi;
  float prob=1- ((numAct-1.0)/numAct) *epsilon;
  if (uniform_random() < epsilon) {
    int rsel=int(uniform_random() * numAct);
    if (rsel!=sel){
      prob=epsilon/numAct;
      sel=rsel;
    }
  }
  action_prime_probability=prob;
  return sel;
}


void logData(int pktNum, int obs, int act, int dist);
void handleButtons();
void displayPredictions();

double netTime;
void agentUpdateAfterAction(int obs, int act,int pktNum) { //post policy
  //pre-learning
  gettimeofday(&timeNow, NULL);
  int time_delta=timeNow.tv_sec*1.-timeBegin.tv_sec*1.;
  netTime = time_delta+ (timeNow.tv_usec*1.-timeBegin.tv_usec)/1000000.;
  displayPredictions();

  //prediction learning
  bool bump= obs > 0;
  rs[0] = bump ? 1 : 0;
  gammas[0]=rs[0]>0 ? 0 : 1-3/100.;
  rhos[0]=last_act==FORWARD ? 1: 0;  // rhos should be for the s,a, not s'a'
  
  WISTDupdate(0);
  for (int i=0;i< NN; i++)
    xnow[i]=xplus[i];

  //control learning
  int dist= getDistanceReward();
  float reward=dist > 0 && !bump ? 1 : 0;
  float gamma =.99;

  float delta = reward - gamma * dot(xACprime, wAC[act],nAC) + dot(xAC,wAC[last_act],nAC);
  for (int i=0;i<nAC;i++)
    wAC[last_act][i] += alphaAC * delta * xAC[i];
  for (int i=0;i<nAC;i++)
    xAC[i]=xACprime[i];

  // post-learning
  logData(pktNum, obs,  act, dist);
  handleButtons();

  last_act=act;
}

void displayPredictions(){
  notes(netTime,predictions[0]);
  if (startingLights >0) {
    setLEDs(255,255, 1 , 1);
    startingLights--;
  }
}


void handleButtons(){

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

void logData(int pktNum, int obs, int act, int dist){

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
    //    c+=sprintf(logstring+c," %f %f %f %f %f %f %f %f", r_delta, rbar, reward, v_now, v_plus, u[0], u[1], u[2] );
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


//---------------------------------- Learning infrastructure
void init_logging() {
  int ms_sleep=100;
  int num_lines=500;
  struct tm *tB = localtime(&(timeBegin.tv_sec));
  char logfilename[100];
  // I should log the source files into an associated directory.
  // This means making the directory, and copying the files into it
  // i.e. two calls to system.
  sprintf(logfilename,"mkdir %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src", logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  int err1=system(logfilename);
  if (err1 < 0){
    printf("Failure to make directory: %s\n",logfilename);
    exit(1);
  }
  sprintf(logfilename,"cp Makefile *.c *.h %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src", logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  int err2=system(logfilename);
  if (err2 < 0){
    printf("Failure to copy files: %s\n",logfilename);
    exit(1);
  }

  sprintf(logfilename,"%s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.logfile", logfile_prefix,
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
  loc+=sprintf(start_comment+loc,"#Global_constants: alpha=%f alpha2=%g lambda=%f prediction_threshold=%f use_prediction=%d learn_prediction=%d explore_on=%d exit_turn_on=%d use_random=%d alphaR=%f alphaU=%f alphaV=%f\n lambdaAC=%f", -1.0f ,-1.0f,lambda,prediction_threshold,use_prediction,learn_prediction,explore_on,exit_turn_on, use_random,alphaR,alphaU,alphaV,lambdaAC);
  loc += sprintf(start_comment+loc,"#Labels netTime updateTime iteration r gamma obs act dist pktNum action_prime  mode_duration vid_count");
  for (int i=0;i<DEMONS;i++)
    loc += sprintf(start_comment+loc," prediction_%d",i);
  loc += sprintf(start_comment+loc," era learn_prediction use_prediction");
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
  if (argc > 1 && 0==strcmp("-r",argv[1])) {
    //norobotmain(argc,argv);
    return 0;
  }else {
  return bmain(argc,argv);
  }
}
