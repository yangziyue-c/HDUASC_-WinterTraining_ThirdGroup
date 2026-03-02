#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完
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
uint8 RunFlag = 0; 	//电机运行标志位
uint8 Mode = 2; 		//发车模式
uint8 Recorder_Flag=0, Tracking_Flag=0;
uint8 flag2=0;

int main(void)
{
	/*初始化部分*/
	clock_init(SYSTEM_CLOCK_600M);	// 不可删除
	debug_init();										// 调试端口初始化
	system_delay_ms(300);	
	
	key_init(10);										//按键初始化
	key_handler_init();							
	SoundLight_Init();							//声光板初始化
	Sensor_Init();									//循迹传感器初始化
	flash_init();                   //Flash初始化
	Bluetooth_Init();								//蓝牙初始化
	Menu_Init();										//菜单初始化
	mpu6050_init();									//陀螺仪初始化
	Kalman_Init(&KF, 0.0001f, 0.003f, 0.1f);		//卡尔曼姿态解算初始化
	Encoder_Init();        					//编码器初始化
	Motor_Init();										//电机初始化
	Pit_Init();											//定时中断初始化
	interrupt_global_enable(0);
	
	PID_Init(&GyroPID);				
	PID_Init(&AnglePID);
	PID_Init(&SpeedPID);
	PID_Init(&TurnPID);
	PID_Init(&TracePID);
	PID_Init(&YAWPID);
	
	/*测试使用*/
//	Set_Motor1(-50);
//	Set_Motor2(-50);
//	SoundLight_On();

	/*主循环*/
	while(1)
	{
		Sensor_Check();			//循迹检测
		key_event_scan();		//按键检测
		Menu_Update();			//菜单刷新
		BlueTooth_Update();	//蓝牙接收发送
		
		ips200_show_string(0, 128, "Mode:");
		ips200_show_uint(64, 128, Mode, 1);
		ips200_show_string(0, 144, "RunFlag:");
		ips200_show_uint(64, 144, RunFlag, 1);
		
		ips200_show_uint(0, 160, path_manager.count, 4);
		ips200_show_uint(0, 176, path_manager.current_index, 4);
		
//		ips200_show_float(0, 200, SpeedPID.Out,4,4);
		ips200_show_float(0, 232, pitch,2,2);
//		ips200_show_uint(0, 264, stat1,1);
//		ips200_show_uint(15, 264, stat2,1);	
//		ips200_show_uint(30, 264, stat3,1);	
//		ips200_show_uint(45, 264, stat4,1);
//		ips200_show_int(0, 280, error,2);		
		ips200_show_int(0, 280, flag_FollowRoute,1);
		/*测试使用*/
//		ips200_show_float(0,144,yaw_offset,4,4);	
//		BlueSerial_Printf("[plot,%f,%f]", TracePID.Actual, TracePID.Out);
//		BlueSerial_Printf("[plot,%d]", DifPWM);
		BlueSerial_Printf("[plot,%d,%d,%d]", flag_FollowRoute, flag_round, error);
	}
}

void pit_handler(void)  //1ms定时中断
{	
	static uint16 count5ms;
	count5ms++;				
	/*5ms定时中断*/
	if(count5ms>=5)
	{
		count5ms=0;
	
		//获得左右轮速度
		Speed_Get();		
		//计算速度
		AveSpeed = (LeftSpeed + RightSpeed) / 2.0;
		DifSpeed = LeftSpeed - RightSpeed;
		
		if(Mode==4 && Recorder_Flag==1){
			Record_PathPoint();
		}
		if(Mode==4 && Tracking_Flag==1){
			Navigation_Calculate();
			SpeedPID.Target=Navigation_Speed;
			TurnPID.Target=Navigation_Turn;
		}
		if(Mode!=4){
			PathTracking_Init();
		}
		


		if(Mode == 2 || Mode == 3)
		{
			Follow_Route();
		}

		Control5ms();



	}

	
	//俯仰角过大自动停机
	if (pitch > 50 || pitch < -50) 
	{
		RunFlag = 0;
	}
	

	Control1ms();
	
}
