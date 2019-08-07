#include "I2C_SCANER.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;


///////////////////////////////////////////////////////////////////////////////////////////////
void I2C_scaner(void)
{
	/*Description function
	This function search devise connected to I2C in this case -hi2c1.
	After thet function print in console information about what to connect to I2C. 
	*/
	
		#define DEVICE_FOUND  0
		uint8_t addres_devise=0x00;;      //ADRESS_MPU6050=0x68; -> return 0  ,   DRESS_MS5611=0x77;-> return 0 		 
		uint8_t addr=0;
		uint16_t sizebuf=1;								// size how many data we receive from devise			
		uint8_t buff=0;										// data for receive
		uint32_t timeout=1000;						// timeout for receive
		uint16_t STATUS=0;								// Status connect to device (if STATUS==0 - device if found, if STATUS==1 - device if not found)
		uint8_t number_of_device=0;				// How many device controller is found

		uint8_t size=0;
		char str3[35]={0};
		uint8_t size_mas=sizeof(str3);
		uint8_t i=0;
		sprintf(str3,"SEARCH DEVISES... \r\n");      										// convert   in  str 
		size=sizeof(str3);
		HAL_UART_Transmit(&huart1 , (uint8_t *)str3, size, 0xFFFF);
		HAL_Delay(500);

	for(addres_devise=0x00;addres_devise<0x7F;addres_devise++)
	{
			HAL_Delay(10);
			STATUS=HAL_I2C_Mem_Read(&hi2c1, (uint16_t)addres_devise<<1,(uint16_t)addr, (uint16_t) sizebuf, &buff, (uint16_t) sizebuf,(uint32_t) timeout);
			if(STATUS==DEVICE_FOUND)																		// if devsice is found
			{	
				  for(i=0;i<size_mas;i++)    															// Delete data in str3[]
					{
						str3[i]=0;
					}
					sprintf(str3,"Device address-0x%x - found \r\n",addres_devise);      // convert   in  str 
					size=sizeof(str3);
					HAL_UART_Transmit(&huart1 , (uint8_t *)str3, size, 0xFFFF);
					number_of_device++;
			}
	}
	if(number_of_device==0)  																				// If devices nofound
	{	
			for(i=0;i<size_mas;i++)				 															// Delete data in str3[]
			{
				str3[i]=0;
			}
			sprintf(str3,"Devices no found!!!\r\n");      							// convert   in  str 
			size=sizeof(str3);
			HAL_UART_Transmit(&huart1 , (uint8_t *)str3, size, 0xFFFF);
	    }
	HAL_Delay(500);
	for(i=0;i<size_mas;i++)																					// Delete data in str3[]
	{
		str3[i]=0;
	}
	sprintf(str3,"DONE\r\n");      																	// convert   in  str 
	size=sizeof(str3);
	HAL_UART_Transmit(&huart1 , (uint8_t *)str3, size, 0xFFFF);	
}
///////////////////////////////////////////////////////////////////////////////////////////////