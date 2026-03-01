#include "pid.h"
#include "mpu6050.h"
#include "Encoder.h"
#include "sensor.h"
#include "menu.h"

int32 LeftPWM = 0, RightPWM = 0;	//左PWM 右PWM
int32 AvePWM = 0, DifPWM = 0;			//平均PWM 差PWM

/*配置各个PID*/
PID_t GyroPID = {
	.OutMax = 10000,
	.OutMin = -10000,
};
PID_t AnglePID = {
	.OutMax = 400,
	.OutMin = -400,
};
PID_t SpeedPID = {
	.OutMax = 20,
	.OutMin = -20,
	
	.ErrorIntMax = 30,
	.ErrorIntMin = -30,
};
PID_t TurnPID = {
	.OutMax = 10,
	.OutMin = -10,
};
PID_t TracePID = {
	.OutMax = 3,
	.OutMin = -3,
};
PID_t YAWPID = {
	.OutMax = 2,
	.OutMin = -2,
	.ErrorIntMax = 20,
	.ErrorIntMin = -20,
};


/*PID参数清除*/
void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Actual1 = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
}

/*PID计算函数*/
void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		
		//积分限幅
		if (p->ErrorInt > p->ErrorIntMax) {p->ErrorInt = p->ErrorIntMax;}
		if (p->ErrorInt < p->ErrorIntMin) {p->ErrorInt = p->ErrorIntMin;}
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   - p->Kd * (p->Actual - p->Actual1);	//微分先行
	
	//输出偏移
	if (p->Out > 0) {p->Out += p->OutOffset;}
	if (p->Out < 0) {p->Out -= p->OutOffset;}
	//输出限幅
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
	
	p->Actual1 = p->Actual;
}


/*角速度环PID（结果输出为平均速度）*/
void Gyro_Tweak (void)
{
	GyroPID.Kp = parameter[1][0];
	GyroPID.Ki = parameter[1][1];
	GyroPID.Kd = parameter[1][2];
	
	GyroPID.Actual = gyro_pitch;
	PID_Update(&GyroPID);
	AvePWM = -GyroPID.Out;
}

/*角度环PID（结果输出给角速度环）*/
void Angle_Tweak (void)
{
	AnglePID.Kp = parameter[2][0];
	AnglePID.Ki = parameter[2][1];
	AnglePID.Kd = parameter[2][2];
	
	AnglePID.Actual = pitch;
	PID_Update(&AnglePID);
	GyroPID.Target = AnglePID.Out;
}

/*速度环PID（结果输出给角度环）*/
void Speed_Tweak (void)
{
	SpeedPID.Kp = parameter[3][0];
	SpeedPID.Ki = parameter[3][1];
	SpeedPID.Kd = parameter[3][2];
	
	SpeedPID.Actual = AveSpeed;
	PID_Update(&SpeedPID);
	AnglePID.Target = SpeedPID.Out;
}




/*转向环PID（结果输出为差速度）*/
void Turn_Tweak(void)
{
	TurnPID.Kp = parameter[4][0];
	TurnPID.Ki = parameter[4][1];
	TurnPID.Kd = parameter[4][2];

	TurnPID.Actual = DifSpeed;
	PID_Update(&TurnPID);
	DifPWM = TurnPID.Out * 1000;
}

/*循迹环PID（结果输出给转向环）*/
void Trace_Tweak(void)
{
	TracePID.Kp = parameter[5][0];
	TracePID.Ki = parameter[5][1];
	TracePID.Kd = parameter[5][2];
	
	TracePID.Actual = -error;
	PID_Update(&TracePID);
	TurnPID.Target = TracePID.Out;
}


/*定yaw角PID（结果输出给转向环），类似循迹环PID*/
void YAW_Tweak(void)
{
	YAWPID.Kp = 3.0;
	YAWPID.Ki = 0.2;
	YAWPID.Kd = 0.1;
	
	YAWPID.Actual = yaw;
	PID_Update(&YAWPID);
	TurnPID.Target = -YAWPID.Out;
}


