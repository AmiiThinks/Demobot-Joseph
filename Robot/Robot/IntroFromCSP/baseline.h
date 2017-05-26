

// The agent code must define the following four functions 
void agentInitialize();
void agentFuneral();
//returns the action for this timestep
int agentPolicy(int obs);
//updates agent state as needed
void agentUpdate(int obs, int act,int pktNum);

// return bmain from main, it won't necessarily return
int bmain(int argc, char **argv);
//call endProgram for a program-controlled exit 
void endProgram(int);

// get the last byte from the remote, valid if not -1
int lastRemoteByte();
//get the distance travelled as reported by odometry
int getDistanceReward();
//extra thread-safe outputs
void setLEDs(int color, int intensity, bool play, bool ff);
void beep();
