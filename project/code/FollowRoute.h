#ifndef __FOLLOWROUTE_H
#define __FOLLOWROUTE_H

#include "zf_common_headfile.h"

extern uint8 stat1, stat2, stat3, stat4;
extern int flag_FollowRoute;  
extern float yaw_target;
extern uint8_t flag_round; 


void Follow_Route(void);
void Follow_Route_Tweak(void);

#endif