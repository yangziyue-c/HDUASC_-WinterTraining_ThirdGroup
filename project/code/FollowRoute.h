#ifndef __FOLLOWROUTE_H
#define __FOLLOWROUTE_H

#include "zf_common_headfile.h"

extern uint8 Mode;                                  //mode状态位：0-暂停；

extern uint8 RunFlag; 	                               //电机运行标志位
extern uint8 flag2;                                  //flag2状态位：是否重置循迹环PID
extern uint8 Recorder_Flag;                         //路径记录标志位
extern uint8 stat1, stat2, stat3, stat4;
extern float yaw;
extern int flag_FollowRoute;  
extern uint8 RunFlag;
extern float yaw_target;
extern uint8_t flag_round; 


void Follow_Route(void);
void Control5ms(void);
void Control1ms(void);

#endif