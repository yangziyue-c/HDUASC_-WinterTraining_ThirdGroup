#include "zf_common_headfile.h"
#include "pit.h"
#include "key_handler.h"
#include "sensor.h"
#include "mpu6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "bluetooth.h"
#include "SoundLight.h"
#include "menu.h"
#include "flash.h"
#include "pid.h"
#include "FollowRoute.h"
#include "Path_Recorder.h"

uint8 RunFlag = 0;
uint8 Recorder_Flag = 0, Tracking_Flag = 0;
uint8 flag1 = 0, flag2 = 0;    // flag1: 模式4完成标志, flag2: 模式2/3退出标志
uint8 Mode = 1;

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    system_delay_ms(300);
    
    key_init(10);
    key_handler_init();
    SoundLight_Init();
    Sensor_Init();
    flash_init();
    Bluetooth_Init();
    Menu_Init();
    mpu6050_init();
    Kalman_Init(&KF, 0.001f, 0.003f, 0.05f);
    Encoder_Init();
    Motor_Init();
    Pit_Init();
    interrupt_global_enable(0);
    
    PID_Init(&GyroPID);
    PID_Init(&AnglePID);
    PID_Init(&SpeedPID);
    PID_Init(&TurnPID);
    PID_Init(&TracePID);
    PID_Init(&YAWPID);
		
		SoundLight_On();

    while(1)
    {
        Sensor_Check();
        key_event_scan();
        Menu_Update();
        BlueTooth_Update();
        
        ips200_show_string(0, 128, "Mode:");
        ips200_show_uint(64, 128, Mode, 1);
        ips200_show_string(0, 144, "RunFlag:");
        ips200_show_uint(64, 144, RunFlag, 1);
        
        ips200_show_uint(0, 160, path_manager.count, 4);
        ips200_show_uint(0, 176, path_manager.current_index, 4);
        
        ips200_show_float(0, 232, pitch, 2, 2);
        ips200_show_int(0, 280, flag_FollowRoute, 1);
        
        BlueSerial_Printf("[plot,%d,%d,%d,%d]", flag_FollowRoute, cnt_slowdown, cnt_speedup,cnt_FollowLine);
		
		// 在主循环显示调试信息
ips200_show_string(0, 192, "Yaw:");
ips200_show_float(40, 192, yaw, 3, 1);
ips200_show_string(0, 208, "Offset:");
ips200_show_float(56, 208, Yaw_offset, 3, 1);
    }
}

void pit_handler(void)
{
    static uint16 count5ms;
    count5ms++;
    
    /* 5ms定时中断 */
    if(count5ms >= 5)
    {
        count5ms = 0;
        
        Speed_Get();
        AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
        DifSpeed = LeftSpeed - RightSpeed;
        
        // 模式4：记录
        if(Mode == 4 && Recorder_Flag)
        {
            Record_PathPoint();
        }
        
        // 模式4：回放
        if(Mode == 4 && Tracking_Flag)
        {
            Navigation_Calculate();
        }
        
        // 模式2、3：循迹
        if(Mode == 2 || Mode == 3)
        {
            Follow_Route();
            Follow_Route_Tweak();
        }
        
        // 速度环
        Speed_Tweak();
        
        // 转向环（记录时不执行）
        if(Mode != 4 || !Recorder_Flag)
        {
            Turn_Tweak();
        }
    }
    
    /* 1ms定时中断 */
    if(pitch > 50 || pitch < -50)
    {
        RunFlag = 0;
    }
    
    Calculate_Attitude();
    Angle_Tweak();
    Gyro_Tweak();
    
    if(RunFlag)
    {
        LeftPWM = AvePWM + DifPWM / 2;
        RightPWM = AvePWM - DifPWM / 2;
        
        if(LeftPWM > 10000) LeftPWM = 10000;
        else if(LeftPWM < -10000) LeftPWM = -10000;
        if(RightPWM > 10000) RightPWM = 10000;
        else if(RightPWM < -10000) RightPWM = -10000;
        
        Set_Motor1(LeftPWM);
        Set_Motor2(RightPWM);
    }
    else
    {
        Set_Motor1(0);
        Set_Motor2(0);
    }
}