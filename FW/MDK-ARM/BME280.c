
#include "BME280.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "main.h"
#include <string.h> 

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

extern int16_t T_indoor;
extern uint32_t P;
extern uint16_t H_indoor;


BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T);
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P);
BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H);


void init_BME280(void)
{
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_ID_ADDRES, (uint16_t) 1, &buff_id, (uint16_t)1, 1000);
		if((STATUS_BME280!=HAL_OK)|(buff_id!=0x60))
		{
					while(1)
					{
						    if(STATUS_BME280!=HAL_OK)
								{
											#if DEBUG
													sprintf(str_BME280,"ERROR CONNECT TO BME280 !!! STATUS!=HAL_OK\r\n");      // convert   in  str 
													size=sizeof(str_BME280);
													HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
											#endif
											HAL_Delay(1000);		
								}	
								if(buff_id!=0x60)
								{
											#if DEBUG
													sprintf(str_BME280,"BME280 not found !!!\r\n");      // convert   in  str 
													size=sizeof(str_BME280);
													HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
									    #endif
											HAL_Delay(1000);		
								}									
					}
		}
    #if DEBUG
				sprintf(str_BME280,"CONNECT TO BME280 OK\r\n");      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 		
		#endif
			
		

		
		// Read all registers like massive
		memset(ALL_REGISTERS, 0, sizeof(ALL_REGISTERS));
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0x00, (uint16_t) 1, ALL_REGISTERS, (uint16_t)255, 1000);
		  	
	
 
    dig_T1 = ((uint16_t)ALL_REGISTERS[0x88]) | (((uint16_t)ALL_REGISTERS[0x89]) << 8);
    dig_T2 = ((int16_t)ALL_REGISTERS[0x8a]) | (((int16_t)ALL_REGISTERS[0x8b]) << 8);
    dig_T3 = ((int16_t)ALL_REGISTERS[0x8c]) | (((int16_t)ALL_REGISTERS[0x8d]) << 8);
 
    dig_P1 = ((uint16_t)ALL_REGISTERS[0x8e]) | (((uint16_t)ALL_REGISTERS[0x8f]) << 8);
    dig_P2 = ((int16_t)ALL_REGISTERS[0x90]) | (((int16_t)ALL_REGISTERS[0x91]) << 8);
    dig_P3 = ((int16_t)ALL_REGISTERS[0x92]) | (((int16_t)ALL_REGISTERS[0x93]) << 8);
    dig_P4 = ((int16_t)ALL_REGISTERS[0x94]) | (((int16_t)ALL_REGISTERS[0x95]) << 8);
    dig_P5 = ((int16_t)ALL_REGISTERS[0x96]) | (((int16_t)ALL_REGISTERS[0x97]) << 8);
    dig_P6 = ((int16_t)ALL_REGISTERS[0x98]) | (((int16_t)ALL_REGISTERS[0x99]) << 8);
    dig_P7 = ((int16_t)ALL_REGISTERS[0x9a]) | (((int16_t)ALL_REGISTERS[0x9b]) << 8);
    dig_P8 = ((int16_t)ALL_REGISTERS[0x9c]) | (((int16_t)ALL_REGISTERS[0x9d]) << 8);
    dig_P9 = ((int16_t)ALL_REGISTERS[0x9e]) | (((int16_t)ALL_REGISTERS[0x9f]) << 8);
 
 
    dig_H1 = ALL_REGISTERS[0xa1];
    dig_H2 = ((int16_t)ALL_REGISTERS[0xe1]) | (((int16_t)ALL_REGISTERS[0xe2]) << 8);
    dig_H3 = ALL_REGISTERS[0xe3];
    dig_H4 = (((int16_t)ALL_REGISTERS[0xe4]) << 4) | (((int16_t)ALL_REGISTERS[0xe5]) & 0x0f);
    dig_H5 = ((((int16_t)ALL_REGISTERS[0xe5]) & 0xf0 ) >> 4) | (((int16_t)ALL_REGISTERS[0xe6]) << 4);
    dig_H6 = (int8_t)ALL_REGISTERS[0xe7];
 	
		// 4. Configure BME280 
		// 1. Configure BME280_CONFIG register
		uint8_t CONFIG_DATA=0x20;  //0x20;  // 0x88;
		uint8_t BME280_CONFIG_DATA_FROM=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CONFIG, (uint16_t) 1, &BME280_CONFIG_DATA_FROM, (uint16_t)1, 1000);
		BME280_CONFIG_DATA_FROM=BME280_CONFIG_DATA_FROM&0x03;
		BME280_CONFIG_DATA_FROM=BME280_CONFIG_DATA_FROM|CONFIG_DATA;
		STATUS_BME280=HAL_I2C_Mem_Write(&hi2c1, BME280_ID<<1, (uint16_t)BME280_CONFIG,1, &BME280_CONFIG_DATA_FROM, 1, 1000);
		
		// Read CONFIG register
		uint8_t BME280_CONFIG_read=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CONFIG, (uint16_t) 1, &BME280_CONFIG_read, (uint16_t)1, 1000);
		
		// 2. Configure BME280_CTRL_MEAS register
		uint8_t BME280_CTRL_MEAS_read=0;
		uint8_t CTRL_MEAS_DATA=0x27; 	//0x4A;  //0x48
		uint8_t BME280_CTRL_MEAS_DATA_FROM=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CTRL_MEAS, (uint16_t) 1, &BME280_CTRL_MEAS_DATA_FROM, (uint16_t)1, 1000);
		BME280_CTRL_MEAS_DATA_FROM=BME280_CTRL_MEAS_DATA_FROM|CTRL_MEAS_DATA;
		STATUS_BME280=HAL_I2C_Mem_Write(&hi2c1, BME280_ID<<1, (uint16_t)BME280_CTRL_MEAS,1, &BME280_CTRL_MEAS_DATA_FROM, 1, 1000);
		
		// Read BME280_CTRL_MEAS register
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CTRL_MEAS, (uint16_t) 1, &BME280_CTRL_MEAS_read, (uint16_t)1, 1000);	
		
		// 3. Configure BME280_CTRL_HUM register
		uint8_t CTRL_HUM=0x02;
		uint8_t BME280_CTRL_HUM_DATA_FROM=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CTRL_HUM, (uint16_t) 1, &BME280_CTRL_HUM_DATA_FROM, (uint16_t)1, 1000);
		BME280_CTRL_HUM_DATA_FROM=BME280_CTRL_HUM_DATA_FROM&0x07;
		BME280_CTRL_HUM_DATA_FROM=BME280_CTRL_HUM_DATA_FROM|CTRL_HUM;
		STATUS_BME280=HAL_I2C_Mem_Write(&hi2c1, BME280_ID<<1, (uint16_t)BME280_CTRL_HUM,1, &BME280_CTRL_HUM_DATA_FROM, 1, 1000);
		
		// Print All register
		uint8_t BME280_CTRL_HUM_read=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CTRL_HUM, (uint16_t) 1, &BME280_CTRL_HUM_read, (uint16_t)1, 1000);
		
		#if DEBUG
				sprintf(str_BME280,"CONFIG=0x%x,  CTRL_MEAS=0x%x, CTRL_HUM=0x%x\r\n",BME280_CONFIG_read,  BME280_CTRL_MEAS_read, BME280_CTRL_HUM_read );      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 
		#endif

		// 
		if(STATUS_BME280!=HAL_OK)
		{
					#if DEBUG
							sprintf(str_BME280,"ERROR CONNECT TO BME280 !!! STATUS!=HAL_OK\r\n");      // convert   in  str 
							size=sizeof(str_BME280);
							HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 		
							HAL_Delay(1000);	
			    #endif
		}
}


void read_data_BME280(void)
{
		// Read all registers like massive
		memset(ALL_REGISTERS, 0, sizeof(ALL_REGISTERS));
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0x00, (uint16_t) 1, ALL_REGISTERS, (uint16_t)255, 1000);
	
		adc_P = ((((int32_t)ALL_REGISTERS[0xf7]) << 16) | (((int32_t)ALL_REGISTERS[0xf8]) << 8) | ((int32_t)ALL_REGISTERS[0xf9])) >> 4;
    adc_T = ((((int32_t)ALL_REGISTERS[0xfa]) << 16) | (((int32_t)ALL_REGISTERS[0xfb]) << 8) | ((int32_t)ALL_REGISTERS[0xfc])) >> 4;
    adc_H = (((int32_t)ALL_REGISTERS[0xfd]) << 8) | ((int32_t)ALL_REGISTERS[0xfe]);
     
		t = BME280_compensate_T_int32(adc_T);
    p = BME280_compensate_P_int64(adc_P);
    h = bme280_compensate_H_int32(adc_H);
	
		// Save data in global variabble
    T_indoor = (0.01f * t)-2;    //
    P =p/10000;
    P=P*0.750063;     // Convert from hPa to mmHg
    H_indoor = 1.0f / 1024.0f * h;
	
		#if DEBUG
				memset(str_BME280,0,sizeof(str_BME280));
				sprintf(str_BME280,"t:%d, p:%d, h:%d <<<<\\r\n", T_indoor , P, H_indoor);      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF);
		#endif
	
}

BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, T;
    var1 = ((((adc_T >> 3) -((BME280_S32_t)dig_T1 << 1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) -((BME280_S32_t)dig_T1)) * ((adc_T >> 4) -((BME280_S32_t)dig_T1))) >> 12) *
        ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8; 
	
    return T;
}

BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;
 
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) * 131072);
    var2 = var2 + (((int64_t)dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)dig_P3) / 256) + ((var1 * ((int64_t)dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)dig_P1) / 8589934592;
 
    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - adc_P;
        var4 = (((var4 * ((int64_t)(2147483648))) - var2) * 3125) / var1;
        var1 = (((int64_t)dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)dig_P7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }
 
    return pressure;
}

BME280_U32_t bme280_compensate_H_int32(BME280_S32_t adc_H)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;
 
    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)dig_H4) * 1048576);
    var4 = ((int32_t)dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);
    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }
     
    return humidity;
}
