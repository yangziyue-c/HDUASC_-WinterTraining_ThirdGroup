#include "sensor.h"

int8 Sensor_Error=0;
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
void Sensor_Check (void)
{
	stat1 = gpio_get_level(SenSor1);
	stat2 = gpio_get_level(SenSor2);
	stat3 = gpio_get_level(SenSor3);
	stat4 = gpio_get_level(SenSor4);
	
	if( !stat1 && stat2 && !stat3 && !stat4) { /* 0 1 0 0 */
		Sensor_Error = -1;
	}
	else if(stat1 && stat2 && !stat3 && !stat4) { /* 1 1 0 0 */
		Sensor_Error = -2;
	} 
	else if(stat1 && !stat2 && !stat3 && !stat4) { /* 1 0 0 0 */
		Sensor_Error = -6;
	} 
	else if(!stat1 && !stat2 && stat3 && !stat4) { /* 0 0 1 0 */
		Sensor_Error = 1;
	} 
	else if(!stat1 && !stat2 && stat3 && stat4) { /* 0 0 1 1 */
		Sensor_Error = 2;
	} 
	else if(!stat1 && !stat2 && !stat3 && stat4) { /* 0 0 0 1 */
		Sensor_Error = 6;
	}
	else if(!stat1 && !stat2 && !stat3 && !stat4) { /* 0 0 0 0 */
		Sensor_Error = Sensor_Error;
	}
	else{
    Sensor_Error = 0;
  }
}
