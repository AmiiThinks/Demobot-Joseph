#include "robotBase.h"
#include "sounds.h"
#include <stdio.h>


bool make_noise=true;
int lastSongTime=0;
int song[15];

void set_make_noise(bool val){
  make_noise=val;
}

void notes(int netTime, float prediction) {    
    if (lastRemoteByte()==REMOTE_LEFT)
      make_noise=!make_noise;
    if (netTime>lastSongTime + 3) {
      int q=prediction*70 + 40;
      song[0]= q > 100 ? 100 : q < 40 ? 40 : q;
      song[1]=10;
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
  song[0]=70;
  song[1]=7;
  song[2]=30;
  song[3]=2;
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
