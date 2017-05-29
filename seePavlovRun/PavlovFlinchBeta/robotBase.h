

// 
// core functions
//


// The agent code must define the following four functions 
void agentInitialize();
void agentFuneral();
//returns the action for this timestep
int agentSelectAction(int obs);
//updates agent state as needed
void agentUpdateAfterAction(int obs, int act,int pktNum);

// 
// setup functions
//

// return bmain from main, it won't necessarily return
int bmain(int argc, char **argv);
//call endProgram for a program-controlled exit 
void endProgram(int);

// 
//additional input/output helper functions
//

// get the last byte from the remote, valid if not -1
int lastRemoteByte();
int lastButtonByte();
int getWheelBump(int k);

//get the distance, rotation travelled as reported by odometry
int getDistanceReward();
int getRotationReward();

//extra thread-safe outputs
void setLEDs(int color, int intensity, bool play, bool ff);
void setExtLEDs(bool one, bool two, bool three);

void setBackupOnBump(bool);

void beep();
//the length of the array is two times numNotes
void playSong(int numNotes,int notes_64ths[]);

const char **getPacketNames();
void getPktValues(int pktNum, int data[]);

#define REMOTE_LEFT          129
#define REMOTE_FORWARD       130
#define REMOTE_RIGHT         131

#define REMOTE_SPOT          132
#define REMOTE_MAX          133
#define REMOTE_CLEAN          136
#define REMOTE_PAUSE         137

#define REMOTE_POWER         138
