#include "baseline-bumper.h"
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

const char logfile_prefix[]="../logfiles/demowallerModes";

int global_argc;
char**global_argv;

#define MM 500
#define NOFFSETS 1000

#define BYTE_RSHIFT 3
#define BYTE_MULTIPLIER (1 << BYTE_RSHIFT)

#define  NN  MM*BYTE_MULTIPLIER


struct timeval timeBegin, timeNow,timeVid,timeUpdate;
int timestep=0;
// Agent thread code


bool make_noise=true;
void  play_explore();
void  play_hit();
void  play_nohit();
void  play_none();
float make_prediction(int x[]);
void init_learning() ;
void load_weights(char *filename);
void save_weights(char *filename);
void select_vid();
void fill_features(int vc,int x[]);
int my_vc=0;
int xplus[NN];
int xnow[NN];
float et[NN];
const float alpha = 0.1/ MM;
const float alpha2 = alpha/1000.;
const float lambda = .9;
float gamma = .95;
float w[NN];
float ww[NN];
#define LOGSTR_SZ 3000
char logstring[LOGSTR_SZ];
char logstring2[LOGSTR_SZ]="";
int iteration=-1;
int era=0;
const float prediction_threshold=0.505;

float delta, rho;

int pixelsize=120*160*2;
int pixelinds[NOFFSETS]
//={11049, 762, 28773, 6970, 21827, 16422, 10899, 22019, 3608, 34002, 34436, 33649, 7659, 35809, 22383, 22378, 12723, 7753, 29159, 2642, 33939, 34839, 16440, 12830, 451, 15998, 21677, 36795, 25323, 11949, 10673, 26031, 32362, 28244, 28983, 37918, 10359, 10897, 22487, 8772, 33205, 25344, 8735, 13493, 21716, 15370, 3324, 35946, 26235, 38134, 19140, 21498, 2768, 15987, 15133, 28747, 12062, 5839, 19553, 1625, 14781, 12984, 21374, 19385, 7350, 18197, 23097, 14495, 35811, 20564, 20571, 7781, 17033, 13293, 26657, 30509, 33114, 34005, 43, 19859, 14354, 28634, 27113, 22894, 18172, 19826, 3389, 3143, 8098, 34368, 13399, 20070, 36407, 4291, 25395, 26469, 28814, 1849, 7253, 32620};
={4396, 7882, 6214, 13401, 28419, 31917, 32134, 18636, 35215, 32733, 17001, 28236, 37547, 29600, 10230, 17256, 13026, 28199, 19887, 16305, 5131, 34770, 31807, 34762, 994, 5756, 15158, 34807, 25817, 35689, 18061, 753, 4491, 20295, 16044, 4494, 9172, 21702, 29955, 16312, 9451, 34135, 26715, 12689, 6049, 11524, 14897, 10725, 7943, 29762, 36939, 6465, 28875, 1671, 37430, 36707, 1110, 5932, 32154, 27069, 3776, 2489, 87, 22306, 10756, 27424, 9250, 30892, 35971, 27194, 255, 31169, 35891, 24148, 16925, 22291, 7420, 16760, 181, 26446, 13148, 34399, 9438, 26849, 11387, 4548, 9239, 13812, 13278, 25713, 14639, 5250, 33887, 22156, 22649, 21075, 23485, 14706, 4704, 6309, 30766, 13037, 11146, 16404, 14060, 29567, 1384, 11647, 30734, 18308, 14824, 23641, 27735, 3249, 1177, 14485, 19151, 35519, 25088, 30981, 2461, 19614, 1039, 15873, 5025, 1013, 12441, 14343, 28887, 4986, 31221, 33584, 23976, 17775, 13964, 14560, 30169, 31913, 26700, 7195, 2697, 37297, 13316, 18757, 37231, 26663, 30813, 29107, 21802, 31727, 6146, 38151, 37761, 25041, 7224, 20403, 26998, 29900, 22255, 29592, 32319, 9194, 28069, 3966, 27491, 9379, 35652, 22584, 12765, 17029, 7536, 34966, 19301, 10062, 10328, 27879, 10015, 20860, 12057, 19434, 22527, 24614, 17937, 5556, 36450, 13438, 2706, 30320, 28245, 24743, 27617, 15648, 5992, 19400, 35174, 33711, 35991, 34072, 20575, 31741, 3802, 20710, 20986, 19109, 1596, 25573, 26511, 16242, 24453, 11205, 24915, 8455, 18781, 23112, 5898, 4877, 26103, 18547, 6862, 21186, 4507, 23079, 3221, 12754, 19762, 24923, 10987, 23907, 31034, 9378, 19392, 37894, 34264, 29556, 1738, 29844, 8239, 25246, 900, 25389, 38084, 34261, 13726, 38059, 5559, 26304, 36274, 16534, 30226, 37288, 35524, 376, 12081, 10887, 35406, 35718, 23750, 20004, 34760, 10601, 28366, 9849, 32307, 9201, 19042, 17607, 596, 30690, 14447, 14999, 7756, 27033, 24238, 7806, 20215, 4314, 34895, 20278, 35401, 9673, 21447, 33864, 22681, 24034, 2386, 33932, 38345, 26296, 33560, 2023, 20475, 5904, 30999, 32504, 38142, 4634, 35926, 3007, 4343, 16626, 38393, 2805, 15398, 1524, 27218, 13994, 23790, 22107, 5921, 19982, 25603, 2645, 34802, 24319, 18747, 34590, 9217, 9401, 7022, 32726, 27993, 31498, 33980, 28880, 30707, 20817, 21480, 35209, 8160, 10507, 23163, 17330, 33686, 15332, 7131, 3912, 2596, 28116, 16753, 1086, 30623, 803, 21417, 35614, 15789, 4961, 2765, 26496, 24766, 6888, 733, 13878, 18325, 9730, 23889, 25333, 2505, 30038, 24993, 35649, 26881, 30672, 6381, 9168, 18963, 21540, 16461, 702, 11520, 36926, 2631, 6079, 37763, 5600, 11778, 31895, 12512, 12960, 26842, 38277, 36502, 36299, 33994, 34042, 31448, 17723, 38238, 5211, 36599, 11266, 21977, 33033, 31577, 14848, 26538, 31738, 4990, 10900, 38108, 16886, 25417, 34594, 31135, 5104, 23881, 28449, 30948, 36421, 22537, 37964, 31761, 35618, 27856, 10950, 3898, 36565, 11079, 37190, 18504, 2036, 2421, 18818, 24767, 3878, 17731, 31887, 4847, 36774, 34029, 16518, 15701, 2015, 17263, 22283, 19733, 11109, 12759, 23236, 5918, 17580, 31131, 2217, 2014, 19587, 24891, 21579, 30026, 19944, 20157, 21406, 36297, 6669, 5701, 20715, 24859, 20031, 8286, 11686, 29555, 27951, 29050, 33070, 33185, 36898, 30146, 29755, 31531, 21366, 26883, 26185, 2453, 4828, 27517, 20205, 14219, 22308, 19554, 15780, 21274, 10157, 38220, 21994, 12320, 10742, 1, 37023, 13068, 22799, 9937, 18397, 28375, 4157, 12593, 15791, 38016, 2707, 9519, 10443, 23312, 5273, 6045, 6187, 14531, 17269, 35481, 15236, 27798, 867, 28943, 28712, 23031, 81, 15171, 8195, 11096, 32634, 17587, 8750, 25932, 4807, 19789, 10374, 16703, 26943, 11770, 2139, 20147, 17412, 2998, 7062, 17072, 620, 19220, 23254, 9292, 23258, 3507, 16122, 14970, 4262, 12033, 31532, 16600, 29381, 5817, 23186, 25149, 35467, 34673, 903, 17811, 36294, 20311, 2246, 19002, 24208, 21306, 20088, 21263, 18202, 3405, 29741, 3698, 17841, 17927, 31679, 12007, 4637, 25384, 28339, 7954, 23405, 29746, 17689, 24896, 21899, 34805, 35080, 20646, 29899, 19433, 24507, 27747, 35046, 21764, 3101, 2978, 26864, 33803, 5942, 28892, 32836, 36484, 19746, 12144, 32449, 30900, 37975, 7351, 37506, 8222, 1300, 29667, 32001, 18771, 34251, 35360, 5809, 3673, 37659, 37228, 3397, 29252, 33024, 17918, 26934, 8637, 25361, 799, 1915, 8763, 22739, 33269, 35466, 18262, 20435, 950, 34840, 21861, 1744, 32507, 25562, 12171, 15578, 16912, 38010, 13654, 14986, 31430, 18395, 4098, 36127, 38333, 20894, 7089, 34879, 1910, 35605, 12890, 31706, 29763, 38172, 24494, 30271, 7968, 17174, 21011, 1122, 35331, 16173, 21608, 12672, 29367, 163, 7030, 12967, 11905, 8624, 26344, 34114, 16679, 18762, 36995, 29976, 12839, 36347, 30373, 7773, 7302, 12686, 34814, 29356, 20606, 30292, 19773, 20988, 10027, 16420, 7972, 13720, 31566, 3292, 31188, 30722, 19867, 35803, 17842, 11662, 34107, 7596, 3506, 37665, 10709, 237, 36511, 9419, 37273, 25387, 12552, 37034, 37969, 23697, 6871, 14596, 23393, 1870, 12317, 14115, 2226, 7629, 8272, 8665, 19844, 32816, 4127, 7804, 37632, 25636, 27726, 8450, 16896, 31877, 15174, 10313, 27645, 36568, 9731, 15383, 10819, 30906, 11802, 28715, 10318, 14205, 21719, 7649, 14495, 36232, 10000, 28697, 31686, 25998, 7477, 17215, 36135, 34724, 3133, 6460, 31747, 12231, 8742, 4282, 14833, 8965, 3289, 17542, 5800, 26347, 19775, 28725, 1765, 29059, 14828, 12264, 7202, 36893, 25270, 30089, 36840, 28639, 35740, 10085, 36182, 32273, 20984, 28711, 8388, 19062, 30805, 24798, 27538, 10407, 4428, 37519, 17493, 9907, 4675, 11042, 21499, 28384, 37965, 830, 6153, 26509, 20513, 19957, 21026, 25026, 36738, 12490, 2899, 66, 32386, 31863, 4822, 2688, 13508, 30003, 5538, 8373, 14372, 2377, 24347, 19235, 21901, 11150, 25495, 15098, 31445, 14876, 37611, 26570, 4203, 7145, 4133, 2935, 16082, 29001, 8242, 22146, 5637, 36998, 21470, 31665, 34704, 38125, 4973, 31343, 23569, 13514, 10926, 34025, 24958, 33734, 2852, 29614, 35764, 13286, 9636, 22693, 16159, 3834, 24074, 26260, 76, 12548, 22561, 10379, 17467, 15677, 29709, 31184, 22068, 22742, 14156, 437, 33206, 36517, 185, 36844, 6394, 18392, 6768, 20296, 12524, 15113, 15409, 26167, 2831, 37017, 5198, 20921, 9792, 34800, 11391, 37164, 3323, 21801, 809, 34557, 10862, 23806, 1769, 20573, 36525, 9857, 13490, 22680, 721, 14831, 34330, 25606, 22968, 28247, 27072, 36222, 955, 4985, 15205, 12113, 9644, 2330, 8250, 297, 37885, 5746, 28127, 27670, 9745, 26075, 3407, 11442, 37615, 12201, 34699, 5541, 585, 5427, 6211, 12845, 26212, 8471, 10749, 16425, 37601, 18911, 13933, 31740, 24519, 19649, 17695, 26080, 7153, 34870, 3935, 35106, 16556, 22843, 23968, 2190, 5946, 27225, 14820, 732, 2334, 26736, 25721, 28426, 16790, 29288, 28043, 17336, 9050, 18839, 1839, 21792, 35139, 27325, 37806, 33571, 27594, 35806, 22396, 12474, 26432, 5887, 23846, 15261, 21611, 17000, 32878, 852, 10768, 25316, 9454, 14702, 12777, 5230, 13757, 7223};
int selected_pixels[2][MM];


pthread_t tid;
pthread_t net_tid;
void * mainNetLoop(void*); 

int use_prediction=0;
int use_random=0;
int learn_prediction=0;
void flip_era_if_needed();
void agentInitialize(){
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

int beh_mode=-1;
int mode_duration=0;
const int explore_on=20;
const int exit_turn_on=5;
float prediction=0;

float hit_decay=0;

bool exploration_required() {
  return  1 > (random()%explore_on) ;
}

int startingLights=0;
int agentPolicy(int s) {
  if (s !=0) 
    hit_decay =1;
  else
    hit_decay *=0.95;

  int q= lastRemoteByte();
  int qq= lastButtonByte();

  //if (-1!=q)
  //  printf("remote: %d\n",q);
  if (q==REMOTE_SPOT || (qq & 4) ) { //pause
    beh_mode=-1;
  } else if (q==REMOTE_FORWARD || (qq & 1)) { //start
    beh_mode=0;
    make_noise=0;
    startingLights=10;
  } else  if (q==REMOTE_MAX ) {
    init_learning(); 
    use_prediction = true;
  } else if (q==REMOTE_CLEAN ) {
    //init_learning(); 
    use_prediction = false;
  }

  select_vid();
  fill_features(my_vc,xplus);
  prediction=make_prediction(xplus);

  if (beh_mode==-1) {
    return STOP;
  }

  // drive until bump then backup 5 steps and turn ccw 5 steps
  //figure out new mode
  int new_mode=beh_mode;
  if (beh_mode != REACTA_MODE && s!=0) {
    new_mode=REACTA_MODE;
    if (beh_mode!=EXPLORE_MODE)
      play_hit();
  } else if (beh_mode==REACTA_MODE && s==0 && mode_duration > 10 ) {
    new_mode=REACTB_MODE;

  } else if ((beh_mode ==FORWARD_MODE || beh_mode==EXPLORE_MODE) && s != 0) {
    new_mode=REACTB_MODE;
    play_hit();
    //} else if (beh_mode==REACTA_MODE && mode_duration > 5) {
    // new_mode=REACTB_MODE;
  }  else if  (beh_mode ==FORWARD_MODE && use_prediction && prediction > prediction_threshold)  {
    if (!exploration_required())
      new_mode=REACTB_MODE;
    else {
      new_mode=EXPLORE_MODE;
      play_explore();
    }
  }else if (beh_mode ==FORWARD_MODE && use_random && 1 > (random() % 50) )  {
    if (1==random()%2) 
      new_mode=RAND_LEFT;
    else
      new_mode=RAND_RIGHT;
  } else if ((beh_mode==RAND_LEFT || beh_mode==RAND_RIGHT) && 1 > (random() % 10) ) {
    new_mode=FORWARD_MODE;
  
  } else if (beh_mode==EXPLORE_MODE) {
    if ( mode_duration > 33+7 ) {//previously .97
      new_mode=FORWARD_MODE;
      play_nohit();
    }
  }
  else if (s== 0 && beh_mode ==REACTB_MODE && mode_duration >= 10 
	   && 1 > (random() % exit_turn_on) // stochastic exit out of turn.
	   ) {
    new_mode=FORWARD_MODE;//(beh_mode +1) %3;
  } 
  if (new_mode!= beh_mode)
    mode_duration=0;
  else
    mode_duration++;
 
  beh_mode=new_mode;
  //printf("era %d iteration %d mode %d steps %d state: %d prediction: %8.6f rho=%4.2f delta=%4.2f \n",era, iteration, beh_mode, mode_duration, s,prediction,rho,delta);
  return FORWARD_MODE==beh_mode ? FORWARD :
    EXPLORE_MODE==beh_mode ? (mode_duration < 7 ? STOP : FORWARD) :
    REACTA_MODE==beh_mode ? STOP:
    RAND_LEFT==beh_mode ?  COUNTERCLOCKWISE: 
    RAND_RIGHT==beh_mode ? CLOCKWISE:
  COUNTERCLOCKWISE; 
}

double elapsedTime;
bool mute=false;
bool rc_space=false;


void init_learning() {
  for (int i=0;i<NN;i++) {
    xplus[i]=0;xnow[i]=0;
    et[i]=0;w[i]=0;ww[i]=0;    
  }
}

void load_weights(char *filename){
  FILE *f=fopen(filename,"r");
  for (int i=0;i<NN;i++ )
    if (1!=fscanf(f,"%f",w+i)) {
      printf("Failed to read weights from file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NN;i++ )
    if (1!=fscanf(f,"%f",ww+i)) {
      printf("Failed to read weights from file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NN;i++ )
    et[i]=0;
  fclose(f);
}


void save_weights(char *filename){
  FILE *f=fopen(filename,"w");
  for (int i=0;i<NN;i++ )
    if (0>fprintf(f,"%f\n",*(w+i))) {
      printf("Failed to write weights to file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NN;i++ )
    if (0>fprintf(f,"%f\n",*(ww+i))) {
      printf("Failed to read weights to file %s at weight %d.\n",filename,i);
      exit(0);
    }
  for (int i=0;i<NN;i++ )
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
  //printf("%d ",vc);
  int *s=selected_pixels[vc]; // this needs to be made thread safe
  for (int i=0;i< MM; i++) {
    int k=i*BYTE_MULTIPLIER + s[i];
    x[k] = 1;
    //printf("%d %d %d (%f) ",i, s[i],k, w[k]);
  }
  //printf("\n");
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
  //if (!make_noise)
  //  return;
  song[0]=70;
  song[1]=7;
  song[2]=30;
  song[3]=2;
  song[2]=80;
  song[3]=7;
  playSong(3, song);
  //song[0]=40;
  //song[1]=64;
  //  song[2]=80;
  //song[3]=10;
  //playSong(1, song);
}


void play_none() {
  //song[0]=50;
  //song[1]=10;
  //  song[2]=80;
  //song[3]=10;
  //playSong(1, song);
}

  static int trek[]=
    {62, 32, 67, 16, 72, 48,
    128, 2, 71, 22, 67, 22, 
     64, 22, 69, 22, 74, 64};

void play_switch() {
  playSong(9, trek);
}


void play_hit() {
  //if (!make_noise)
  //  return;
  song[0]=60;
  song[1]=10;
  //song[2]=50;
  //song[3]=10;
  playSong(1, song);
}

void play_nohit() {
  //if (!make_noise)
  //  return;
  song[0]=100;
  song[1]=10;
  //song[2]=50;
  //song[3]=10;
  playSong(1, song);
}

float make_prediction(int x[]){
  float lprediction=0;
  for (int i=0;i< NN; i++) {
    float q=w[i] *x[i];
    lprediction += q;
  }
  return lprediction;
}
int data_size=0;
int *pkt_data;
int last_act=-1;

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

// just switch to GTD lambda
void agentUpdate(int obs, int act,int pktNum) { //post policy

  //printf("%d %d\n",pktNum,getWheelBump(pktNum));
    gettimeofday(&timeNow, NULL);
    int time_delta=timeNow.tv_sec*1.-timeBegin.tv_sec*1.;
    double netTime = time_delta+ (timeNow.tv_usec*1.-timeBegin.tv_usec)/1000000.;
    notes(netTime,prediction);
    if (startingLights >0) {
      setLEDs(255,255, 1 , 1);
      startingLights--;
    } else if (hit_decay > .3 ) 
      setLEDs(0,255, 1 , 1);
    else if (beh_mode == RAND_LEFT)  
      setLEDs(0,0, 1 , 0);
    else if (beh_mode == RAND_RIGHT)  
      setLEDs(0,0, 0 , 1);
    else
      setLEDs(0,0, 0 , 0);
    //printf("nettime: %f %ld %ld\n",netTime, timeNow.tv_sec, timeBegin.tv_sec);
    if (beh_mode==EXPLORE_MODE ) {
      //setLEDs(255,255, 0 , 0);
      setExtLEDs( ((iteration/3) %2)==1 ,  ((iteration/5) %2)==0 ,  ((iteration/5) %2)==0);
    } else {
      //setLEDs( 0 ,255, obs !=0 , prediction > .4);
      setExtLEDs(prediction > .358 ,  prediction > .505 ,prediction > .18 );
    }
    //int vc=vid_count%2;

    //unsafe off-policy learning.
  //learn via TD lambda
  float r = obs > 0 ? 1 : 0;
  if (r > 0)
    gamma =0;
  else 
    gamma =0.95;
  rho= last_act==FORWARD ? 1: 0;
  if (rho==0) {
    for (int i=0;i<NN;i++) {
      et[i] =0;
    }
    delta=0;
  }
  //GTD(lambda) with rho
  if (learn_prediction) {
    // set variable gamma going to zero on r.
    delta = r;
    for (int i=0;i< NN; i++)
      delta += gamma * w[i] * xplus[i]  - w[i] * xnow[i];
    //eligibility trace
    for (int i=0;i<NN;i++) {
      et[i] =rho *(xnow[i] + et[i]);
    }
    float ew=dot(et,ww , NN);
    float xw=idot(xnow,ww,NN);
    float glew=gamma * (1-lambda) * ew;
    //float ad=alpha*delta;
    //float a2d = alpha2*delta;
    //float aglew=alpha*glew;
    //float a2tw=alpha2*tw;
    // delta done
    for (int i=0;i< NN; i++) {
      w[i] += alpha * ( delta * et[i] - glew * xplus[i]);
      ww[i] += alpha2 * (delta * et[i] - xw * xnow[i]);
      //w[i] += ad * et[i] - aglew * xplus[i];
      //ww[i] += a2d * et[i] - a2tw * xnow[i];
    }
    for (int i=0;i< NN; i++)
      xnow[i]=xplus[i];
    
    //float lg=lambda*gamma;
    //eligibility trace
    for (int i=0;i<NN;i++) {
      et[i] *= lambda*gamma;
    }
    //printf ("delta: %4.3f\n", delta);
  }



  getPktValues(pktNum,pkt_data);
  //printf ("pred: %4.3f\n",pred);
  int dist= getDistanceReward();
  //int rotation=getRotationReward();
  gettimeofday(&timeUpdate, NULL);
  double updateTime = (timeUpdate.tv_sec*1.-timeNow.tv_sec*1.)
		      + (timeUpdate.tv_usec*1.-timeNow.tv_usec)/1000000.;
  //printf("update time: %f\n",updateTime);
  {//logging
    int c=sprintf(logstring,"%9.3f %9.3f %d %4.3f %4.3f %d %d %d %d %d %d %d %4.3f",netTime, updateTime, iteration, r, gamma, obs,act, dist, pktNum, beh_mode, mode_duration,vid_count, prediction);
    if (beh_mode!= -1)
      iteration++;
    c+=sprintf(logstring+c," %d %d %d",era, learn_prediction, use_prediction);
    for (int k=0;k<data_size;k++) {
      c+=sprintf(logstring+c," %d",pkt_data[k]);
    }
    c+=sprintf(logstring+c," ");
    for (int k=0;k<MM;k++) {
      //sprintf(logstring+c,"%c",'0'+selected_pixels[my_vc][k]);
      *(logstring+c)='0'+selected_pixels[my_vc][k];
      c+=1;
    }
    sprintf(logstring+c,"\n");
    writelog(logstring);  
    strncpy(logstring2,logstring,LOGSTR_SZ);
  }

  flip_era_if_needed();
  if (lastRemoteByte()==REMOTE_PAUSE) {
    endProgram(0);
  }
  if ((lastRemoteByte()==REMOTE_RIGHT) && (!want_snapshot)) {
    //play_switch();
    //want_snapshot=true;
    save_weights("./my_weights");
  }
  if (lastRemoteByte()==REMOTE_POWER) {
    load_weights("./my_weights");
  }
  
  last_act=act;
}

void flip_era_if_needed() {
  //if (0==iteration%(8000)) { //  4 minutes 
  //     era++; 
  //   init_learning(); 
  //   use_prediction = !use_prediction; 
  //   play_switch();
  //   //learn_prediction = !learn_prediction;
  //  }
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
  sprintf(logfilename,"mkdir %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src",
	  logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  system(logfilename);
  sprintf(logfilename,"cp Makefile *.c *.h %s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.src", 	  logfile_prefix,
          (tB->tm_year+1900),(1+tB->tm_mon),tB->tm_mday,
           tB->tm_hour,tB->tm_min,tB->tm_sec);
  system(logfilename);
  sprintf(logfilename,"%s-y%d-m%02d-d%02d-h%02d-m%02d-s%02d.logfile",
	  logfileprefix,
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
  loc+=sprintf(start_comment+loc,"#Global_constants: alpha=%f alpha2=%g lambda=%f prediction_threshold=%f use_prediction=%d learn_prediction=%d explore_on=%d exit_turn_on=%d use_random=%d\n",alpha,alpha2,lambda,prediction_threshold,use_prediction,learn_prediction,explore_on,exit_turn_on, use_random);
  loc += sprintf(start_comment+loc,"#Labels netTime updateTime iteration r gamma obs act dist pktNum beh_mode  mode_duration vid_count prediction era learn_prediction use_prediction");
  const char **names=getPacketNames();
  for (int i=0;names[i]!=NULL; i++,data_size=i) {
    loc += sprintf(start_comment+loc," %s",names[i]);
    
  }
  pkt_data=new int[data_size];
  //for (int i=0;i< NN;i++)
  //  loc += sprintf(start_comment+loc," im%03d",i);
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
	agentUpdate(0, 0,0);
    }
    agentFuneral();
}

int main(int argc, char **argv) {
  global_argc=argc;
  global_argv=argv;
  gettimeofday(&timeBegin, NULL);
  init_video();
  init_logging();
  // grab out behaviour policy one of -r(eflex) -p(avlov) or -f(ixed)
  if (argc > 1 && 0==strcmp("-r",argv[1])) {
    norobotmain(argc,argv);
    return 0;
  }else {
  return bmain(argc,argv);
  }
}
