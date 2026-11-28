#include "../firmware/nusu_leak2/core.h"
#include <cstdio>
#include <cmath>
int main(){ SigDetector sd; double Ps=3.5,C=0.1,p=Ps; bool valve=false; 
 for(long ms=0; ms<20000; ms++){ double ts=ms/1000.0; double tap= ts>5 && ts<15 ? 9*std::min(1.0,(ts-5)*1000/400):0;
  if(!valve){ double frac=std::sqrt(std::max(0.0,p)/Ps); p-= (tap*frac/60/1000)/C; if(p<0)p=0; if(ts>5.6) valve=true; }
  else { double target=Ps-0.0015*tap*tap; p+=(target-p)*0.2; }
  sd.feed(ms,p); for(int i=0;i<sd.nout;i++) printf("t=%.3f type=%d mag=%.2f dur=%d p=%.2f\n",ts,sd.out[i].type,sd.out[i].mag,sd.out[i].durMs,p); sd.nout=0; }
}
