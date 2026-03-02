#include "sensor.h"

int8 error=0;
uint8 flag = 0, prev_stat = 0, curr_stat = 0;
uint8 stat1 = 0, stat2 = 0, stat3 = 0, stat4 = 0;
static uint16 cnt = 0;

/*³õÊ¼»¯Ñ­¼£Ä£¿éÒý½Å*/
void Sensor_Init (void)
{
	gpio_init(SenSor1,GPI,0,GPI_PULL_UP);
	gpio_init(SenSor2,GPI,0,GPI_PULL_UP);
	gpio_init(SenSor3,GPI,0,GPI_PULL_UP);
	gpio_init(SenSor4,GPI,0,GPI_PULL_UP);
}

/*¼ì²âÑ­¼£×´Ì¬*/
int8 Sensor_Check (void)
{
	stat1 = gpio_get_level(SenSor1);
	stat2 = gpio_get_level(SenSor2);
	stat3 = gpio_get_level(SenSor3);
	stat4 = gpio_get_level(SenSor4);
	
	prev_stat = curr_stat;
	
	if(stat1||stat2||stat3||stat4)curr_stat = 1;
	else curr_stat = 0;
	
	if(curr_stat != prev_stat){flag = 1;cnt = Flag_Hold;}
	else if (cnt)cnt--;
	else flag = 0;
	
	if( !stat1 && stat2 && !stat3 && !stat4) { /* 0 1 0 0 */
		error = -1;
	}
	else if(stat1 && stat2 && !stat3 && !stat4) { /* 1 1 0 0 */
		error = -2;
	} 
	else if(stat1 && !stat2 && !stat3 && !stat4) { /* 1 0 0 0 */
		error = -6;
	} 
	else if(!stat1 && !stat2 && stat3 && !stat4) { /* 0 0 1 0 */
		error = 1;
	} 
	else if(!stat1 && !stat2 && stat3 && stat4) { /* 0 0 1 1 */
		error = 2;
	} 
	else if(!stat1 && !stat2 && !stat3 && stat4) { /* 0 0 0 1 */
		error = 6;
	}
	else if(!stat1 && !stat2 && !stat3 && !stat4) { /* 0 0 0 0 */
		error = error;
	}
	else{
    error = 0;
  }
	return error;
}
