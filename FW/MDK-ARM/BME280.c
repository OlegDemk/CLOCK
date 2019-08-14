
#include "BME280.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "main.h"

extern float T_indoor;
extern uint16_t P;
extern float H_indoor;
float calulete_temperature(uint32_t adc_T);
uint32_t calulete_preasure(uint32_t adc_P);
float calulete_humidity(uint32_t adc_H);


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
			
		// 3. Read calibrate registers
		uint8_t buf_calibration_data[24]={0};
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CALIB_00, (uint16_t) 1, buf_calibration_data, (uint16_t)24, 1000);
			
		// Print All callibration data T and P like one arrey
		#if DEBUG
		
		    // Read all registers like massive
				uint8_t ALL_REGISTERS[255];
				for (uint16_t f=0; f<=255; f++)
				{
						ALL_REGISTERS[f]=0;
				}
		    STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CALIB_00, (uint16_t) 1, ALL_REGISTERS, (uint16_t)255, 1000);
		    char str_reg[40]="0";
				uint16_t i=0;
				for (i=0; i<=255; i++)
				{
					  sprintf(str_reg,"REG:[%d]: %x  \r\n", i , ALL_REGISTERS[i]);      // convert   in  str 
						size=sizeof(str_reg);
						HAL_UART_Transmit(&huart1 , (uint8_t *)str_reg, size, 0xFFF); 
				}
				HAL_Delay(50000);
				/////////
				
				i=0;
				for (i=0; i<=24; i++)
				{
						char str_adc[40]="0";
					  if(i<=5)
						{
								sprintf(str_adc,"callibration data_T[%d]: %x  \r\n", i , buf_calibration_data[i]);      // convert   in  str 
								size=sizeof(str_adc);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_adc, size, 0xFFF); 
						}
						else
						{	
								sprintf(str_adc,"callibration data_P[%d]: %x  \r\n", i , buf_calibration_data[i]);      // convert   in  str 
								size=sizeof(str_adc);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_adc, size, 0xFFF); 
						}
				}
				HAL_Delay(2000);
		#endif		
		
		dig_T1_LSB=(unsigned short)buf_calibration_data[0];
		dig_T1_MSB=(unsigned short)buf_calibration_data[1];
		dig_T1=((unsigned short)dig_T1_MSB<<8)|((unsigned short)dig_T1_LSB);
		
		dig_T2_LSB=(signed short)buf_calibration_data[2];
		dig_T2_MSB=(signed short)buf_calibration_data[3];
		dig_T2=((signed short)dig_T2_MSB<<8)|((signed short)dig_T2_LSB);
		
		dig_T3_LSB=(signed short)buf_calibration_data[4];
	  dig_T3_MSB=(signed short)buf_calibration_data[5];
		dig_T3=((signed short)dig_T3_MSB<<8)|((signed short)dig_T3_LSB);
		
		dig_P1=(unsigned short)buf_calibration_data[7]<<8|(unsigned short)buf_calibration_data[6];
		dig_P2=(signed short)buf_calibration_data[9]<<8|(signed short)buf_calibration_data[8];
		dig_P3=(signed short)buf_calibration_data[11]<<8|(signed short)buf_calibration_data[10];
		dig_P4=(signed short)buf_calibration_data[13]<<8|(signed short)buf_calibration_data[12];
		dig_P5=(signed short)buf_calibration_data[15]<<8|(signed short)buf_calibration_data[14];
		dig_P6=(signed short)buf_calibration_data[17]<<8|(signed short)buf_calibration_data[16];
		dig_P7=(signed short)buf_calibration_data[19]<<8|(signed short)buf_calibration_data[18];
		dig_P8=(signed short)buf_calibration_data[21]<<8|(signed short)buf_calibration_data[20];
		dig_P9=(signed short)buf_calibration_data[23]<<8|(signed short)buf_calibration_data[22];
		
		
		
		
		// Print All callibration data H like one arrey
		#if DEBUG
				// Read H1 from 0xA1
				uint8_t buf_calibration_H1=0;
				STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xA1, (uint16_t) 1, &buf_calibration_H1, (uint16_t)1, 1000);
				char str_adc_H1[40]="0";
				sprintf(str_adc_H1,"callibration data_H 0xA1: %x \r\n", buf_calibration_H1);      // convert   in  str 
				size=sizeof(str_adc_H1);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_adc_H1, size, 0xFFF); 
		
		   
				// Read callibration data h from 0xE1 to 0xE7 like arrey
				uint8_t buf_calibration_data_H[7];
				STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE1, (uint16_t) 1, buf_calibration_data_H, (uint16_t)7, 1000);
				
				i=0;
				for (i=1; i<=8; i++)
				{
						char str_adc[30]="0";
				  	sprintf(str_adc,"callibration data_H:%d: %x  \r\n", i , buf_calibration_data_H[i]);      // convert   in  str 
						size=sizeof(str_adc);
						HAL_UART_Transmit(&huart1 , (uint8_t *)str_adc, size, 0xFFF); 
					
				}
				HAL_Delay(2000);
		#endif		
		
		
		
		// Read H1 from cilibrate register
		uint8_t buf_calibration_data_H1=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xA1, (uint16_t) 1, &buf_calibration_data_H1, (uint16_t)1, 1000);
		dig_H1=(unsigned char)buf_calibration_data_H1;    
		
		// Read H2 calibration register
		uint8_t buf_calibration_data_H2[2]={0};
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE1, (uint16_t) 1, buf_calibration_data_H2, (uint16_t)2, 1000);	
		dig_H2=(signed short)buf_calibration_data_H2[1]<<8|(signed short)buf_calibration_data_H2[0];
		
		// Read H3 calibration register
		uint8_t buf_calibration_data_H3=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE3, (uint16_t) 1, &buf_calibration_data_H3, (uint16_t)1, 1000);	
		dig_H3=(unsigned char)buf_calibration_data_H3;
		
		// Read H4 calibration register
		uint8_t buf_calibration_data_H4[2]={0};
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE4, (uint16_t) 1, buf_calibration_data_H4, (uint16_t)2, 1000);	
		dig_H4=(signed short)buf_calibration_data_H4[0]<<4|(0x0F&(signed short)buf_calibration_data_H4[1]);
		
		// Read H5 calibration register
		uint8_t buf_calibration_data_H5[2]={0};
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE5, (uint16_t) 1,buf_calibration_data_H5, (uint16_t)2, 1000);	
		dig_H5=(signed short)buf_calibration_data[1]<<4|((signed short)buf_calibration_data[0]>>4);
		
		// Read H6 calibration register
		uint8_t buf_calibration_data_H6=0;
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)0xE7, (uint16_t) 1, &buf_calibration_data_H6, (uint16_t)1, 1000);	
		dig_H6=(signed char)buf_calibration_data_H6;
		
	
		
		// Print All callibration data
		#if DEBUG
		
				sprintf(str_BME280,"CALIBRATE DATA====================\r\n");      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 
				
				char str1[60]="";
				sprintf(str1,"dig_T1:%d, dig_T2:%d, dig_T3:%d\r\n", dig_T1, dig_T2, dig_T3);      // convert   in  str 
				size=sizeof(str1);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str1, size, 0xFFF); 
				
				char str2[60]="";
				sprintf(str2,"dig_P1:%d, dig_P2:%d, dig_P3:%d\r\n", dig_P1, dig_P2, dig_P3);      // convert   in  str 
				size=sizeof(str2);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str2, size, 0xFFF); 
				
				char str3[60]="";
				sprintf(str3,"dig_P4:%d, dig_P5:%d, dig_P6:%d\r\n", dig_P4, dig_P5, dig_P6);      // convert   in  str 
				size=sizeof(str3);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str3, size, 0xFFF); 
				
				char str4[60]="";
				sprintf(str4,"dig_P7:%d, dig_P8:%d, dig_P9:%d\r\n", dig_P7, dig_P8, dig_P9);      // convert   in  str 
				size=sizeof(str4);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str4, size, 0xFFF); 
				
				char str5[60]="";
				sprintf(str5,"dig_H1:%d, dig_H2:%d, dig_H3:%d\r\n", dig_H1, dig_H2, dig_H3);      // convert   in  str 
				size=sizeof(str5);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str5, size, 0xFFF); 
						
				char str6[60]="";		
				sprintf(str6,"dig_H4:%d, dig_H5:%d, dig_H6:%d\r\n", dig_H4, dig_H5, dig_H6);      // convert   in  str 
				size=sizeof(str6);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str6, size, 0xFFF); 
				
				char str7[60]="";
				sprintf(str7,"END CALIBRATE DATA====================\r\n\r\n");      // convert   in  str 
				size=sizeof(str7);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str7, size, 0xFFF); 
	
				
				HAL_Delay(1000);
		#endif		
		
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


////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_data_BME280(void)
{
	 // HAL_Delay(1000);
		// 4. Read measure data from BME280
		uint8_t buf_data_from_BME280[8]={0};      
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_PRESS_MSB, (uint16_t) 1, buf_data_from_BME280, (uint16_t)8, 1000);
	  
		#if DEBUG  // Print all adc registers
				uint8_t i=0;
				for (i=0; i<=7; i++)
				{
					  char str_adc[15]="0";
						sprintf(str_adc,"adc[%d]:%x  \r\n", i ,buf_data_from_BME280[i]);      // convert   in  str 
						size=sizeof(str_adc);
						HAL_UART_Transmit(&huart1 , (uint8_t *)str_adc, size, 0xFFF); 
				}
		#endif
		
		uint32_t adc_T=(((uint32_t)buf_data_from_BME280[3]) << 12) | (((uint32_t)buf_data_from_BME280[4]) << 4) |(((uint32_t)buf_data_from_BME280[5]) >> 4);
    uint32_t adc_P=(((uint32_t)buf_data_from_BME280[0])<<12)|(((uint32_t)buf_data_from_BME280[1])<<4)|(((uint32_t)buf_data_from_BME280[2])>>4);		
		uint32_t adc_H=(((uint32_t)buf_data_from_BME280[6])<<8)|((uint32_t)buf_data_from_BME280[7]);
	
		#if DEBUG
				for(uint8_t i=0; i<=60; i++)
				{
						str_BME280[i]=0;
				}
				sprintf(str_BME280,"adc_T= %d  adc_P= %d, adc_H= %d \r\n", adc_T, adc_P, adc_H);      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
		#endif
		
		
    T_indoor=calulete_temperature(adc_T); 
    P=calulete_preasure(adc_P);		
		H_indoor=calulete_humidity(adc_H);

		// Print 
		for(uint8_t i=0; i<=60; i++)
		{
				str_BME280[i]=0;
		}
		#if DEBUG
				sprintf(str_BME280,"Temperature=%.2f Preasure=%d Humidity=%.1f\r\n", T_indoor, P, H_indoor);      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
		#endif
			
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
float calulete_temperature(uint32_t adc_T)
{
//	// 5. Convert TEMPERATURE/
//		// From datasheet
//		int32_t var1=((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
//		int32_t var2=(((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
//	  t_fine=var1+var2;
//		int32_t T=(t_fine*5+128)>>8;
//		T_indoor = (((float)T)/100.0f);
//	
//	  return  T_indoor;
		
		// NEW METHOD
	  double var1;
    double var2;
    double temperature;
    double temperature_min = -40;
    double temperature_max = 85;

    var1 = ((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0;
    var1 = var1 * ((double)dig_T2);
    var2 = (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0);
    var2 = (var2 * var2) * ((double)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temperature = (var1 + var2) / 5120.0;
    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

uint32_t calulete_preasure(uint32_t adc_P)
{	   
//		double var1, var2, p;
//		var1 = ((double)t_fine/2.0)-64000.0;
//		var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
//		var2 = var2 + var1 * ((double)dig_P5) * 2.0;
//		var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
//		var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
//		var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
//		if (var1 == 0.0)
//		{
//				return 0; // avoid exception caused by division by zero
//		}
//		p = 1048576.0-(double)adc_P;
//		p = (p-(var2 / 4096.0)) * 6250.0 / var1;
//		var1 = ((double)dig_P9) * p * p / 2147483648.0;
//		var2 = p * ((double)dig_P8) / 32768.0;
//		p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
//		
//		p=p/100;
//		p=p*0.750063;  	// Convert from hPa to mmHg
//	
//		return p;
    	
		double var1;
    double var2;
    double var3;
    double pressure;
    double pressure_min = 30000.0;
    double pressure_max = 110000.0;
		

    var1 = ((double)t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var3 = ((double)dig_P3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);

    /* avoid exception caused by division by zero */
    if (var1)
    {
        pressure = 1048576.0 - (double) adc_P;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)dig_P9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)dig_P8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)dig_P7)) / 16.0;
        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else /* Invalid case */
    {
        pressure = pressure_min;
    }
     
		pressure=pressure/100;
		pressure=pressure*0.750063;  	// Convert from hPa to mmHg
    return pressure;
}

float calulete_humidity(uint32_t adc_H)
{	
//		// print transformed calibration data for HUMIDITY
//		char str_print_uart[70]="";
//		sprintf(str_print_uart,"H1: %d, H2: %d, H3: %d\r\n", dig_H1, dig_H2, dig_H3 );      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 

//    uint8_t i=0;
//		for (i=0; i<=size; i++)
//		{
//			str_print_uart[i]=0;
//		}
//		
//		sprintf(str_print_uart,"H4: %d, H5: %d, H6: %d\r\n", dig_H4, dig_H5, dig_H6 );      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 		
//		

//		// Calculate cumidity
//		double var_H;
//		var_H = (((double)t_fine)-76800.0);
//		// Print var_H
//		for (i=0; i<=size; i++)
//		{
//			str_print_uart[i]=0;
//		}
//		sprintf(str_print_uart,"var_H: %d\r\n", (int)var_H);      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 		
//		
//		var_H = (adc_H-(((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
//		(((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
//		(1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
//		
//		// Print var_H
//		for (i=0; i<=size; i++)
//		{
//			str_print_uart[i]=0;
//		}
//		sprintf(str_print_uart,"var_H: %d\r\n", (int)var_H);      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 	
//		
//		
//		var_H = var_H * (1.0-((double)dig_H1) * var_H / 524288.0);
//		
//		// Print var_H
//		for (i=0; i<=size; i++)
//		{
//			str_print_uart[i]=0;
//		}
//		sprintf(str_print_uart,"var_H: %d\r\n", (int)var_H);      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 	
//		
//		if (var_H > 100.0)
//			var_H = 100.0;
//		else if (var_H < 0.0)
//			var_H = 0.0;
//		
//		sprintf(str_BME280,"HUMIDITY: %d\r\n", (int)var_H);      // convert   in  str 
//		size=sizeof(str_BME280);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
//		
//		// Print t_fine
//		for (i=0; i<=size; i++)
//		{
//			str_print_uart[i]=0;
//		}
//		sprintf(str_print_uart,"t_fine: %d\r\n", (int)t_fine);      // convert   in  str 
//		size=sizeof(str_print_uart);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 	
//		
//	
//		return var_H;

//		t_fine=27;

////////		char str_print_uart[70]="";
////////		uint8_t i=0;
////////    for (i=0; i<=size; i++)
////////		{
////////			str_print_uart[i]=0;
////////		}
////////		sprintf(str_print_uart,"t_fine: %d\r\n", (int)t_fine);      // convert   in  str 
////////		size=sizeof(str_print_uart);
////////		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 	

////////		double humidity;
////////    double humidity_min = 0.0;
////////    double humidity_max = 100.0;
////////    double var1;
////////    double var2;
////////    double var3;
////////    double var4;
////////    double var5;
////////    double var6;

////////    var1 = ((double)t_fine) - 76800.0;
////////    var2 = (((double)dig_H4) * 64.0 + (((double)dig_H5) / 16384.0) * var1);
////////    var3 = adc_H - var2;
////////    var4 = ((double)dig_H2) / 65536.0;
////////    var5 = (1.0 + (((double)dig_H3) / 67108864.0) * var1);
////////    var6 = 1.0 + (((double)dig_H6) / 67108864.0) * var1 * var5;
////////    var6 = var3 * var4 * (var5 * var6);
////////    humidity = var6 * (1.0 - ((double)dig_H1) * var6 / 524288.0);
////////    if (humidity > humidity_max)
////////    {
////////        humidity = humidity_max;
////////    }
////////    else if (humidity < humidity_min)
////////    {
////////        humidity = humidity_min;
////////    }

////////    return humidity;



/////////////////////////////////////////////////////////////////////\\
 Second method
   
	  //t_fine=27;
 
    char str_print_uart[70]="";
		uint8_t i=0;
    for (i=0; i<=size; i++)
		{
			str_print_uart[i]=0;
		}
		sprintf(str_print_uart,"t_fine: %d\r\n", (int)t_fine);      // convert   in  str 
		size=sizeof(str_print_uart);
		HAL_UART_Transmit(&huart1 , (uint8_t *)str_print_uart, size, 0xFFF); 


    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
		
		sprintf(str_BME280,"HUMIDITY: %.1f________\r\n", h / 1024.0);      // convert   in  str 
		size=sizeof(str_BME280);
		HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
		
    return  (float)h / 1024.0f;
/////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//		int v_x1_u32r;
//		v_x1_u32r = (t_fine - ((int)76800));
//		v_x1_u32r = (((((adc_H << 14) - (((int)dig_H4) << 20) - (((int)dig_H5) * v_x1_u32r)) +
//		((int)16384)) >> 15) * (((((((v_x1_u32r * ((int)dig_H6)) >> 10) * (((v_x1_u32r * 
//		((int)dig_H3)) >> 11) + ((int)32768))) >> 10) + ((int)2097152)) *
//		((int)dig_H2) + 8192) >> 14));
//		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int)dig_H1)) >> 4));
//		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
//		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
//	  
//		uint32_t buf=v_x1_u32r>>12;
//		buf=buf/1024;
//			return buf;
//////////////////////////////////////////////////////////////////		
		
}



