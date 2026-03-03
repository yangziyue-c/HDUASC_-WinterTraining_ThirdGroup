#include "bluetooth.h"
#include "pid.h"
#include "menu.h"

extern uint8 RunFlag, Mode, Recorder_Flag, Tracking_Flag;

char BlueSerial_RxPacket[64]; 	//接收数组
uint8_t BlueSerial_RxFlag;			//接收标志位

/*初始化蓝牙*/
void Bluetooth_Init (void)
{
  uart_init(UART_4, 9600, UART4_TX_C16, UART4_RX_C17);
  uart_rx_interrupt(UART_4, 1);
}

/*发送字节*/
void BlueSerial_SendByte(uint8_t Byte)
{
	uart_write_byte(UART_4, Byte);
}

/*发送数组*/
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uart_write_buffer (UART_4, Array, Length);
}

/*发送字符串*/
void BlueSerial_SendString(char *String)
{
	uart_write_string (UART_4, String);
}

/*发送数字辅助函数*/
uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/*发送数字*/
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/*格式化发送数据*/
void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

/*接受中断*/
void uart_rx_interrupt_handler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
	{
		uint8_t RxData = uart_read_byte (UART_4);
		
		if (RxState == 0)
		{
			if (RxData == '[' && BlueSerial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1)
		{
			if (RxData == ']')
			{
				RxState = 0;
				BlueSerial_RxPacket[pRxPacket] = '\0';
				BlueSerial_RxFlag = 1;
			}
			else
			{
				BlueSerial_RxPacket[pRxPacket] = RxData;
				pRxPacket ++;
			}
		}
	
	}
	LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // 不允许删除
}
/*蓝牙更新*/
void BlueTooth_Update (void)
{
		if (BlueSerial_RxFlag == 1)								//有接收
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			
			
			if (strcmp(Tag, "key") == 0)								//按键(开关设置)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				if (strcmp(Name, "1") == 0 && strcmp(Action, "down") == 0)
				{
						PID_Init(&GyroPID);				
						PID_Init(&AnglePID);
						PID_Init(&SpeedPID);
						PID_Init(&TurnPID);
						PID_Init(&TracePID);
						RunFlag=1;
				}	
				if (strcmp(Name, "1") == 0 && strcmp(Action, "up") == 0)
				{
						RunFlag=0;
				}
				if (strcmp(Name, "2") == 0 && strcmp(Action, "down") == 0)
				{
						Mode++;
						if(Mode==6)Mode=1;
				}
				if (strcmp(Name, "3") == 0 && strcmp(Action, "down") == 0)
				{
						Recorder_Flag=1;
				}	
				if (strcmp(Name, "3") == 0 && strcmp(Action, "up") == 0)
				{
						Recorder_Flag=0;
				}
				if (strcmp(Name, "4") == 0 && strcmp(Action, "down") == 0)
				{
						Tracking_Flag=1;
				}	
				if (strcmp(Name, "4") == 0 && strcmp(Action, "up") == 0)
				{
						Tracking_Flag=0;
				}					
			}
			
			else if (strcmp(Tag, "slider") == 0)				//滑块（调参功能）
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				if (strcmp(Name, "GyroKp") == 0)
				{
					parameter[1][0] = atof(Value);
				}
				else if (strcmp(Name, "GyroKi") == 0)
				{
					parameter[1][1] = atof(Value);
				}
				else if (strcmp(Name, "GyroKd") == 0)
				{
					parameter[1][2] = atof(Value);
				}
				if (strcmp(Name, "AngleKp") == 0)
				{
					parameter[2][0] = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					parameter[2][1] = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					parameter[2][2] = atof(Value);
				}
				else if (strcmp(Name, "SpeedKp") == 0)
				{
					parameter[3][0] = atof(Value);
				}
				else if (strcmp(Name, "SpeedKi") == 0)
				{
					parameter[3][1] = atof(Value);
				}
				else if (strcmp(Name, "SpeedKd") == 0)
				{
					parameter[3][2] = atof(Value);
				}
				else if (strcmp(Name, "TurnKp") == 0)
				{
					parameter[4][0] = atof(Value);
				}
				else if (strcmp(Name, "TurnKi") == 0)
				{
					parameter[4][1] = atof(Value);
				}
				else if (strcmp(Name, "TurnKd") == 0)
				{
					parameter[4][2] = atof(Value);
				}
				else if (strcmp(Name, "TraceKp") == 0)
				{
					parameter[5][0] = atof(Value);
				}
				else if (strcmp(Name, "TraceKi") == 0)
				{
					parameter[5][1] = atof(Value);
				}
				else if (strcmp(Name, "TraceKd") == 0)
				{
					parameter[5][2] = atof(Value);
				}
				else if (strcmp(Name, "Speed") == 0)
				{
					SpeedPID.Target = atof(Value);
				}
			}
			
			else if (strcmp(Tag, "joystick") == 0)			//摇杆（遥控功能）
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				/*模式五功能*/
				if(Mode==5)
				{
					SpeedPID.Target = LV / 250.0;				//控制前后加速
					TurnPID.Target = RH / 100.0;				//控制左右转向
				}
				
			}
			
			BlueSerial_RxFlag = 0;
		}
}
