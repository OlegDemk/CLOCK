
#include "BME280.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "main.h"

extern float T_indoor;
extern uint16_t P;
extern uint16_t H_indoor;
float calulete_temperature(uint32_t adc_T);
uint32_t calulete_preasure(uint32_t adc_P);
uint32_t calulete_humidity(uint32_t adc_H);

// Wariabble for temperature
int32_t t_fine=0;

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
	
		// New method
		dig_T1_LSB=buf_calibration_data[0];
		dig_T1_MSB=(unsigned short)buf_calibration_data[1];
		dig_T1=dig_T1_MSB<<8|dig_T1_LSB;
		
		dig_T2_LSB=buf_calibration_data[2];
		dig_T2_MSB=(short)buf_calibration_data[3];
		dig_T2=dig_T2_MSB<<8|dig_T2_LSB;
		
		dig_T3_LSB=buf_calibration_data[4];
	  dig_T3_MSB=(short)buf_calibration_data[5];
		dig_T3=dig_T3_MSB<<8|dig_T3_LSB;
		
		dig_P1=(unsigned short)buf_calibration_data[7]<<8|(unsigned short)buf_calibration_data[6];
		dig_P2=(signed short)buf_calibration_data[9]<<8|(signed short)buf_calibration_data[8];
		dig_P3=(signed short)buf_calibration_data[11]<<8|(signed short)buf_calibration_data[10];
		dig_P4=(signed short)buf_calibration_data[13]<<8|(signed short)buf_calibration_data[12];
		dig_P5=(signed short)buf_calibration_data[15]<<8|(signed short)buf_calibration_data[14];
		dig_P6=(signed short)buf_calibration_data[17]<<8|(signed short)buf_calibration_data[16];
		dig_P7=(signed short)buf_calibration_data[19]<<8|(signed short)buf_calibration_data[18];
		dig_P8=(signed short)buf_calibration_data[21]<<8|(signed short)buf_calibration_data[20];
		dig_P9=(signed short)buf_calibration_data[23]<<8|(signed short)buf_calibration_data[22];
		
//		dig_H1=(unsigned short)buf_calibration_data[24];    ////  25   ?????????
		
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CALIB_26, (uint16_t) 1, buf_calibration_data, (uint16_t)10, 1000);
		
//		dig_H2=(signed short)buf_calibration_data[1]<<8|(signed short)buf_calibration_data[0];
//		dig_H3=(unsigned short)buf_calibration_data[2];
//		dig_H4=(signed short)buf_calibration_data[4]<<3|(signed short)buf_calibration_data[5];
//		dig_H5=(signed short)buf_calibration_data[6]<<11|(signed short)buf_calibration_data[7];
//		dig_H6=(signed short)buf_calibration_data[8];
		
	
		
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
	  
		uint32_t adc_T=(uint32_t)buf_data_from_BME280[3]<<12|buf_data_from_BME280[4]<<4|buf_data_from_BME280[5]>>4;
    uint32_t adc_P=(uint32_t)buf_data_from_BME280[0]<<12|(uint32_t)buf_data_from_BME280[1]<<4|(uint32_t)buf_data_from_BME280[2]>>4;		
		uint32_t adc_H=(uint32_t)buf_data_from_BME280[6]<<8|(uint32_t)buf_data_from_BME280[7];
	
		#if DEBUG
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
				sprintf(str_BME280,"T=%.2f P=%d H=%d\r\n", T_indoor, P, H_indoor);      // convert   in  str 
				size=sizeof(str_BME280);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
		#endif
			
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
float calulete_temperature(uint32_t adc_T)
{
	// 5. Convert TEMPERATURE/
		// From datasheet
		int32_t var1=((((adc_T>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
		int32_t var2=(((((adc_T>>4)-((int32_t)dig_T1))*((adc_T>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;

	  t_fine=var1+var2;
		int32_t T=(t_fine*5+128)>>8;
		T_indoor = (((float)T)/100.0f);
	
	  return  T_indoor;
}

uint32_t calulete_preasure(uint32_t adc_P)
{	   
		double var1, var2, p;
		var1 = ((double)t_fine/2.0)-64000.0;
		var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double)dig_P5) * 2.0;
		var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
		var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
		if (var1 == 0.0)
		{
				return 0; // avoid exception caused by division by zero
		}
		p = 1048576.0-(double)adc_P;
		p = (p-(var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)dig_P9) * p * p / 2147483648.0;
		var2 = p * ((double)dig_P8) / 32768.0;
		p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
		
		p=p/100;
		p=p*0.750063;  	// Convert from hPa to mmHg
	
		return p;
}

uint32_t calulete_humidity(uint32_t adc_H)
{	

//    int32_t v_x1_u32r;

//    v_x1_u32r = (t_fine - ((int32_t)76800));

//    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
//                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
//                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
//                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
//                    ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

//    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
//                               ((int32_t)dig_H1)) >> 4));

//    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
//    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
//    float h = (v_x1_u32r>>12);
//		
//		sprintf(str_BME280,"HUMIDITY: %f________\r\n", h / 1024.0);      // convert   in  str 
//		size=sizeof(str_BME280);
//		HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
//		
//    return  h / 1024.0;
	
	
//////////////////////////////////////////////////////////////////
////////		double humidity;
////////		double humidity_min = 0.0;
////////		double humidity_max = 100.0;
////////		double var1;
////////		double var2;
////////		double var3;
////////		double var4;
////////		double var5;
////////		double var6;

////////		var1 = ((double)t_fine) - 76800.0;
////////		var2 = (((double)dig_H4) * 64.0 + (((double)dig_H5) / 16384.0) * var1);
////////		var3 = humidity - var2;
////////		var4 = ((double)dig_H2) / 65536.0;
////////		var5 = (1.0 + (((double)dig_H3) / 67108864.0) * var1);
////////		var6 = 1.0 + (((double)dig_H6) / 67108864.0) * var1 * var5;
////////		var6 = var3 * var4 * (var5 * var6);
////////		humidity = var6 * (1.0 - ((double)dig_H1) * var6 / 524288.0);

////////		if (humidity > humidity_max)
////////			humidity = humidity_max;
////////		else if (humidity < humidity_min)
////////			humidity = humidity_min;

////////	  sprintf(str_BME280,"HUMIDITY: %d________\r\n", (int)humidity);      // convert   in  str 
////////		size=sizeof(str_BME280);
////////		HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
////////	
////////	return humidity;
//////////////////////////////////////////////////////////////////		
    //adc_H  29760
		
		//adc_H=297600;
		
				
		unsigned char dig_H1=0;
		signed short dig_H2=0;
		unsigned char dig_H3=0;
		signed short dig_H4=0;
		signed short dig_H5=0;
		signed char dig_H6=0;
		
		uint8_t buf_calibration_data[24]={0};
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CALIB_00, (uint16_t) 1, buf_calibration_data, (uint16_t)24, 1000);
		dig_H1=(unsigned short)buf_calibration_data[25];    ////  25   ?????????
		
		STATUS_BME280=HAL_I2C_Mem_Read(&hi2c1, BME280_ID<<1,(uint16_t)BME280_CALIB_26, (uint16_t) 1, buf_calibration_data, (uint16_t)10, 1000);
		
		dig_H2=(signed short)buf_calibration_data[1]<<8|(signed short)buf_calibration_data[0];
		dig_H3=(unsigned char)buf_calibration_data[2];
		signed short dig_H4_msb=(int16_t)(int8_t)buf_calibration_data[4]*16;
		signed short dig_H4_lsb=(int16_t)(buf_calibration_data[4] & 0x0F);
		dig_H4 = dig_H4_msb | dig_H4_lsb;
		signed short dig_H5_msb = (int16_t)(int8_t)buf_calibration_data[6] * 16;
	  signed short dig_H5_lsb = (int16_t)(buf_calibration_data[7] >> 4);
		dig_H5 = dig_H5_msb | dig_H5_lsb;
		dig_H6=(int8_t)buf_calibration_data[8];

		double var_H;
		var_H = (((double)t_fine)-76800.0);
		var_H = (adc_H-(((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
		(((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
		(1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
		var_H = var_H * (1.0-((double)dig_H1) * var_H / 524288.0);
		if (var_H > 100.0)
			var_H = 100.0;
		else if (var_H < 0.0)
			var_H = 0.0;
		
		sprintf(str_BME280,"HUMIDITY: %d________\r\n", (int)var_H);      // convert   in  str 
		size=sizeof(str_BME280);
		HAL_UART_Transmit(&huart1 , (uint8_t *)str_BME280, size, 0xFFF); 	
		
		return var_H;





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
























