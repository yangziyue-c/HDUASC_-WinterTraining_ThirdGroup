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

uint16_t cnt_FollowLine = 0;                               //巡线时避免过早检测B点和A点






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
		   if (cnt_FollowLine > 100)
		   {
			   if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
			   {
				   cnt2++;
				   if(cnt2 > 10)             //检测到黑线持续5次
				   {
					   SoundLight_On();              //鸣笛并闪灯
					   flag_FollowRoute = 2;               //进入状态二
					   cnt_FollowLine = 0;
					   cnt2 = 0;
				   }
			   }
			   else   cnt2 = 0;
		   }
		   else
		   {
			   cnt_FollowLine += 1;
		   }
       }
       /* C点判定 */
       else if(flag_FollowRoute == 2)
       {
		   if (cnt_FollowLine > 100)
		   {
			   if(!(stat1 || stat2 || stat3 || stat4))                  //所有点检测到白线
			   {
				   cnt2 ++;
				   if(cnt2 > 50)               //检测到白线持续5次
				   {
					   SoundLight_On();              //鸣笛并闪灯
					   flag_FollowRoute = 3;
					   cnt_FollowLine = 0;
					   cnt2 = 0;
					   yaw_target = yaw_target - 180;
				   }
			   }
			   else   cnt2 = 0;
		   }
		   else
		   {
			   cnt_FollowLine += 1;
		   }

       }
       /* D点判定 */
       else if(flag_FollowRoute == 3)
       {
		   if (cnt_FollowLine > 100)
		   {
			   if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
			   {
				   cnt2++;
				   if(cnt2 > 10)                //检测到黑线持续Black_CNT2次
				   {
					   SoundLight_On();              //鸣笛并闪灯 
					   flag_FollowRoute = 4;
					   cnt_FollowLine = 0;
					   cnt2 = 0;
				   }
			   }
			   else   cnt2 = 0;
		   }
		   else
		   {
			   cnt_FollowLine += 1;
		   }
       }
       /* A点（终点）判定 */
       else if(flag_FollowRoute == 4)
       {
		   
		   if (cnt_FollowLine > 100)
		   {
			   if(!(stat1 || stat2 || stat3 || stat4))                  //所有点检测到白线
			   {
				   cnt2++;
				   if(cnt2 > 50 )                  //检测到白线持续White_CNT4次
				   {
					   SoundLight_On();              //鸣笛并闪灯
					   flag_FollowRoute = 0;           //一圈结束
					   cnt_FollowLine = 0;
					   cnt2 = 0;
				   }
			   }
			   else   cnt2 = 0;
		   }
		   else
		   {
			   cnt_FollowLine += 1;
		   }
		   

       }
   }





   /* 模式三 */
   else if(Mode == 3)
   {
		if (RunFlag == 0)
	   {
			yaw_target = yaw - 38;
			flag_round = 0;    //圈数标志位
	   }
        static int cnt3 = 0;
        
        /* C点判定 */
        if(flag_FollowRoute == 1)
        {
		   if (cnt_FollowLine > 200)
		   {
				if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
				{
					cnt3++;
					if(cnt3 > 15)               //检测到黑线持续15次
					{
						SoundLight_On();              //鸣笛并闪灯
						flag_FollowRoute = 2;               //进入下一状态
						cnt_FollowLine = 0;
						cnt3 = 0;
					}
				}
				else   cnt3 = 0;
			}
			else 
			{
				cnt_FollowLine += 1;
			}
		   
        }
    
        /* B点判定 */
        else if(flag_FollowRoute == 2)
        {
			
			if (cnt_FollowLine > 200)
			{
				if(!(stat1 || stat2 || stat3 || stat4))              //所有点检测到白线
				{
					cnt3++;
					if(cnt3 > 60)                             //检测到白线持续30次（避免车在C点冲出去检测不到黑线）
					{
						SoundLight_On();              //鸣笛并闪灯
						flag_FollowRoute = 3;
						cnt_FollowLine = 0;
						cnt3 = 0;
						yaw_target = yaw_target + 258;
					}
				}
			}
			else 
			{
				cnt_FollowLine += 1;
			}

        }
    
        /* D点判定 */
        else if(flag_FollowRoute == 3)
        {
		   if (cnt_FollowLine > 200)
		   {
				if(stat1 || stat2 || stat3 || stat4)        //任一点检测到黑线
				{
					cnt3++;
					if(cnt3 > 15)                      //检测到黑线持续15次
					{
						SoundLight_On();              //鸣笛并闪灯
						flag_FollowRoute = 4;
						cnt_FollowLine = 0;
						cnt3 = 0;
					}
				}
			}
			else 
			{
				cnt_FollowLine += 1;
			}
        }
    
        /* A点判定 */
        else if(flag_FollowRoute == 4)
        {
		   if (cnt_FollowLine > 200)
		   {
				if(!(stat1 || stat2 || stat3 || stat4))             //所有点检测到白线
				{
					cnt3++;
					if(cnt3 > 60)                      //检测到白线持续50次
					{
						SoundLight_On();              //鸣笛并闪灯
						flag_FollowRoute = 1;                     //下一次循环
						cnt_FollowLine = 0;
						cnt3 = 0;
						yaw_target = yaw_target - 258;
						flag_round = flag_round + 1;       //第一圈结束
						if(flag_round >= 4) 
						{
							flag_round = 0;                   //第四圈结束
							flag_FollowRoute = 0;                      //停止运行
							cnt_FollowLine = 0;
							
						}
					}
				}
		   }
		   else 
		   {
			   cnt_FollowLine += 1;
		   }

        }
       

   }

}






uint16 cnt_slowdown = 0;           //循迹前提前减速，避免冲出黑线导致错误
uint16 cnt_speedup = 0;            //循迹后慢慢加速，避免刚循迹时角度太大冲出黑线


/*根据任务状态使用循迹PID与定yaw角PID*/
void Follow_Route_Tweak(void)
{        
	if(RunFlag == 1 && (Mode == 2 || Mode == 3))
	{
		flag2 = 1;
		//yaw环PID
		if(flag_FollowRoute == 1)
		{
			cnt_speedup = 0;          //不用加速，重置cnt_speedup
			if (cnt_slowdown < 400)
			{
				SpeedPID.Target = 0.50;
				cnt_slowdown += 1;
			}
			else
			{
				SpeedPID.Target = 0.35;      //提前减速
			}
			
			YAWPID.Target = yaw_target;
			YAW_Tweak();
		}

		else if(flag_FollowRoute == 2)
		{
			cnt_slowdown = 0;          //不用减速，重置cnt_slowdown
			if (cnt_speedup < 100)
			{
				SpeedPID.Target = 0.35;
				cnt_speedup += 1;
			}
			else
			{
				SpeedPID.Target = 0.50;
			}
			//循迹环PID
			Trace_Tweak();
		}
		//yaw环PID
		else if(flag_FollowRoute == 3)
		{
			cnt_speedup = 0;
			if (cnt_slowdown < 400)
			{
				SpeedPID.Target = 0.50;
				cnt_slowdown += 1;
			}
			else
			{
				SpeedPID.Target = 0.35;      //提前减速
			}
			YAWPID.Target = yaw_target;
			YAW_Tweak();   
		}
		else if(flag_FollowRoute == 4)
		{
			cnt_slowdown = 0;          //不用减速，重置cnt_slowdown
			if (cnt_speedup < 100)
			{
				SpeedPID.Target = 0.35;
				cnt_speedup += 1;
			}
			else
			{
				SpeedPID.Target = 0.50;
			}
			//循迹环PID
			Trace_Tweak();
		}
		else if(flag_FollowRoute == 0)
		{
			cnt_slowdown = 0;          //不用减速，重置cnt_slowdown
			cnt_speedup = 0;
			SpeedPID.Target = 0;
			TurnPID.Target = 0;
		}
	}
    else if (flag2)
    {
			flag2=0;
			TurnPID.Target =0 ;
    }
		
}
