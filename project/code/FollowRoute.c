#include "FollowRoute.h"
#include "pid.h"
#include "sensor.h"
#include "SoundLight.h"
#include "menu.h"

extern uint8 Mode, RunFlag, flag2;
extern float yaw;

int flag_FollowRoute = 1;                                  //flag状态位：0-暂停；1-AB；2-BC；3-CD；4-DA；
float yaw_target = 0;
uint8_t flag_round = 0;                                    //圈数标志位


/*根据循线检测更新任务状态*/
void Follow_Route(void)
{
   

  stat1 = gpio_get_level(SenSor1);
	stat2 = gpio_get_level(SenSor2);
	stat3 = gpio_get_level(SenSor3);
	stat4 = gpio_get_level(SenSor4);

   /* 模式二 */
   if(Mode == 2)
   {
     static int cnt2 = 0;
	   if (RunFlag == 0)
	   {
			yaw_target = yaw;
	   }
       /* B点判定 */
       if(flag_FollowRoute == 1)
       {
           if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
           {
               cnt2++;
               if(cnt2 > 5)             //检测到黑线持续5次
               {
                   SoundLight_On();              //鸣笛并闪灯
                   flag_FollowRoute = 2;               //进入状态二
                   cnt2 = 0;
               }
           }
           else   cnt2 = 0;
       }
       /* C点判定 */
       else if(flag_FollowRoute == 2)
       {
           if(!(stat1 || stat2 || stat3 || stat4))                  //所有点检测到白线
           {
               cnt2 ++;
               if(cnt2 > 5)               //检测到白线持续5次
               {
                   SoundLight_On();              //鸣笛并闪灯
                   flag_FollowRoute = 3;
                   cnt2 = 0;
				   yaw_target = yaw_target - 180;
               }
           }
           else   cnt2 = 0;
       }
       /* D点判定 */
       else if(flag_FollowRoute == 3)
       {
           if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
           {
               cnt2++;
               if(cnt2 > 5)                //检测到黑线持续Black_CNT2次
               {
                   SoundLight_On();              //鸣笛并闪灯 
                   flag_FollowRoute = 4;
                   cnt2 = 0;
               }
           }
           else   cnt2 = 0;
       }
       /* A点（终点）判定 */
       else if(flag_FollowRoute == 4)
       {
           if(!(stat1 || stat2 || stat3 || stat4))                  //所有点检测到白线
           {
               cnt2++;
               if(cnt2 > 5 )                  //检测到白线持续White_CNT4次
               {
                   SoundLight_On();              //鸣笛并闪灯
                   flag_FollowRoute = 0;           //一圈结束
                   cnt2 = 0;
               }
           }
           else   cnt2 = 0;
       }
   }





   /* 模式三 */
   else if(Mode == 3)
   {
		if (RunFlag == 0)
	   {
			yaw_target = yaw - 40;
			flag_round = 0;    //圈数标志位
	   }
        static int cnt3 = 0;
        
        /* C点判定 */
        if(flag_FollowRoute == 1)
        {
            if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
            {
                cnt3++;
                if(cnt3 > 10)               //检测到黑线持续5次
                {
                    SoundLight_On();              //鸣笛并闪灯
                    flag_FollowRoute = 2;               //进入下一状态
                    cnt3 = 0;
                }
            }
            else   cnt3 = 0;
        }
    
        /* B点判定 */
        else if(flag_FollowRoute == 2)
        {
            if(!(stat1 || stat2 || stat3 || stat4))              //所有点检测到白线
            {
                cnt3++;
                if(cnt3 > 60)                             //检测到白线持续60次（避免车在C点冲出去检测不到黑线）
                {
                    SoundLight_On();              //鸣笛并闪灯
                    flag_FollowRoute = 3;
                    cnt3 = 0;
                    yaw_target = yaw_target + 258;
                }
            }
        }
    
        /* D点判定 */
        else if(flag_FollowRoute == 3)
        {
            if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
            {
                cnt3++;
                if(cnt3 > 10)                      //检测到黑线持续5次
                {
                    SoundLight_On();              //鸣笛并闪灯
                    flag_FollowRoute = 4;
                    cnt3 = 0;
                }
            }
        }
    
        /* A点判定 */
        else if(flag_FollowRoute == 4)
        {
            if(!(stat1 || stat2 || stat3 || stat4))             //所有点检测到白线
            {
                cnt3++;
                if(cnt3 > 60)                      //检测到白线持续50次
                {
                    SoundLight_On();              //鸣笛并闪灯
                    flag_FollowRoute = 1;                     //下一次循环
                    cnt3 = 0;
                    yaw_target = yaw_target - 258;
                    flag_round = flag_round + 1;       //第一圈结束
                    if(flag_round >= 4) 
                    {
                        flag_round = 0;                   //第四圈结束
                        flag_FollowRoute = 0;                      //停止运行
                    }
                }
            }
        }
       

   }

}





/*根据任务状态使用循迹PID与定yaw角PID*/
void Follow_Route_Tweak(void)
{        
	if(Mode == 2 || Mode == 3)
	{
		flag2 = 1;
		//循迹环PID
		if(flag_FollowRoute == 1)
		{
			SpeedPID.Target = 0.40;
//    TurnPID.Target = 0;
			YAWPID.Target = yaw_target;
			YAW_Tweak();
		}

		else if(flag_FollowRoute == 2)
		{
			SpeedPID.Target = 0.40;
			//循迹环PID
			Trace_Tweak();
		}

		else if(flag_FollowRoute == 3)
		{
			SpeedPID.Target = 0.40;
//    TurnPID.Target = 0;
			YAWPID.Target = yaw_target;
			YAW_Tweak();   
		}
		else if(flag_FollowRoute == 4)
		{
			SpeedPID.Target = 0.40;
			//循迹环PID
			Trace_Tweak();
		}
		else if(flag_FollowRoute == 0)
		{
			RunFlag = 0;
		}
	}
    else if (flag2)
    {
			flag2=0;
			TurnPID.Target =0 ;
    }
		
}
