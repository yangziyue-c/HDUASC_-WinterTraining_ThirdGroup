#ifndef __PATH_RECORDER_H
#define __PATH_RECORDER_H

#include "zf_common_headfile.h"

// 路径点结构
typedef struct {
    float left_speed;      	// 左轮速度值
    float right_speed;     	// 右轮速度值
} PathPoint_t;

// 路径记录结构
typedef struct {
    PathPoint_t points[3000];   // 最大记录3000个点
    uint16_t count;             // 当前路径点数
    uint16_t current_index;     // 当前复现索引
} PathManager_t;

extern PathManager_t path_manager;
extern float Navigation_Speed, Navigation_Turn;

void PathTracking_Init(void);
void Path_SaveToFlash(void);
void Path_LoadFromFlash(void);
void Record_PathPoint(void);
void Navigation_Calculate(void);

#endif
