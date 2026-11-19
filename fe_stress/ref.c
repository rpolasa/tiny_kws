#include <stdio.h>
#include <stdlib.h>
#include "../model/kws_fe.c"
/* prints per-sample y of every channel and features; mirrors fe_sample */
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"r"); int hop=atoi(argv[2]); fe_state_t s; fe_reset(&s); int v; uint8_t feat[FB_NCH];
  FILE*fy=fopen("ref_y.txt","w"), *ff=fopen("ref_f.txt","w");
  while(fscanf(f,"%x",&v)==1){ int16_t x=(int16_t)v; int r=fe_sample(&s,x,hop,feat);
    for(int k=0;k<FB_NCH;k++) fprintf(fy,"%05x\n", s.y1[k]&0x1ffff);
    if(r) for(int k=0;k<FB_NCH;k++) fprintf(ff,"%02x\n",feat[k]); }
  return 0; }
