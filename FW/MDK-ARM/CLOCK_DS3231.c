
#include "CLOCK_DS3231.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "main.h"



extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

extern float T_outdoor;
extern uint16_t H_outdoor;
extern float T_indoor;
extern uint16_t H_indoor;
extern uint32_t P;

/*//////////////////////////////////////////////////////////////////////////////
This function init DS3231
*///////////////////////////////////////////////////////////////////////////////
void start_init_DS3231(void)
{
	  // init 24 hour mode 
	   uint8_t size=0;
	   // Read data from  hour  register;
	   uint8_t buffer_hour=0;
	   HAL_I2C_Mem_Read(&hi2c1, ADRESS_DEVCE_DS3231<<1,ADDRES_HOUR_REG, (uint16_t) sizebuf, &buffer_hour, 1, timeout);
	   // Write  24 hour mode 
	   buffer_hour=buffer_hour&0x1F;  // 0x1F=0b0001 1111;
	   data_to_DS3231=buffer_hour|0x00;
		 HAL_I2C_Mem_Write(&hi2c1, ADRESS_DEVCE_DS3231<<1, ADDRES_HOUR_REG, sizebuf, &data_to_DS3231, return_one_byre, timeout);
}

/*//////////////////////////////////////////////////////////////////////////////
This function read all data from DS3231 in one massive and write it
in global variabble.
*///////////////////////////////////////////////////////////////////////////////
// Read data from clock module
void read_data_from_DS3231(void)
{
			STATUS=HAL_I2C_Mem_Read(&hi2c1, ADRESS_DEVCE_DS3231<<1,Read_sec, (uint16_t) sizebuf, buff, sizebuf_data_return, timeout);
	    if(STATUS==HAL_OK)
			{
				     // SEC
				     sec=RTC_ConvertFromDec(buff[0]);
				     // MIN
				     min=RTC_ConvertFromDec(buff[1]);
				     // HOUR
				     //uint8_t H=buff[2]&0x1F;
				     hour=RTC_ConvertFromDec(buff[2]);
				     // DAY
				     day=RTC_ConvertFromDec(buff[3]);
				     // DATE
				     date=RTC_ConvertFromDec(buff[4]);
				     // MONTH
				     uint8_t H=buff[5]&0x1F;
				     month=RTC_ConvertFromDec(H);
				     // YEAR
				     year=RTC_ConvertFromDec(buff[6]);   
			}
      else								// If ettor STATE
			{
				    while(1)
						{
								//Error hendler
						}
						
			}
			#if DEBUG	
					// Print data in COMPORT
					uint8_t size=0;
					sprintf(str,"SEC:%d MIN:%d HOUR:%d\r\n", sec, min, hour);      // convert   in  str 
					size=sizeof(str);
					HAL_UART_Transmit(&huart1 , (uint8_t *)str, size, 0xFFF); 
					sprintf(str,"DAY:%d DATE:%d MOUNT:%d YEAR:%d \r\n",day, date, month, year);      // convert   in  str 
					size=sizeof(str);
					HAL_UART_Transmit(&huart1 , (uint8_t *)str, size, 0xFFF); 
					
					

					// TEST HOUR REG
					uint8_t buffer_hour=0;
					HAL_I2C_Mem_Read(&hi2c1, ADRESS_DEVCE_DS3231<<1,ADDRES_HOUR_REG, (uint16_t) sizebuf, &buffer_hour, 1, timeout);
					
					sprintf(str,"HOUR REG: 0x%x <<<<<<<\r\n",buffer_hour);      // convert   in  str 
					size=sizeof(str);
					HAL_UART_Transmit(&huart1 , (uint8_t *)str, size, 0xFFF); 
			#endif
  
}

/*//////////////////////////////////////////////////////////////////////////////
This function convert data from bin in dec for print it in LCD and UART.
*///////////////////////////////////////////////////////////////////////////////
uint8_t RTC_ConvertFromDec(uint8_t c)
{
        uint8_t ch = ((c>>4)*10+(0x0F&c));
        return ch;
}

/*//////////////////////////////////////////////////////////////////////////////
This function convert data from dec in bin for read it data in clock.
*///////////////////////////////////////////////////////////////////////////////
uint8_t RTC_ConvertFromBinDec(uint8_t c)
{
        uint8_t ch = ((c/10)<<4)|(c%10);
        return ch;
}

/*//////////////////////////////////////////////////////////////////////////////
This function write data in clock.
param: reg_DS3231 - number of register for write data (like hour, second and so on..)
			 data_to_DS3231 - data will be write in clock.
*///////////////////////////////////////////////////////////////////////////////
void WRITE_DATA(uint16_t reg_DS3231, uint8_t data_to_DS3231)
{
			// This function write some data in DS3231;
		  uint8_t size=0;
	    uint8_t buffer_write=0;
	
	    /// CONVERT INPUT DATA IN 
	    buffer_write=RTC_ConvertFromBinDec(data_to_DS3231);
	
			STATUS=HAL_I2C_Mem_Write(&hi2c1, ADRESS_DEVCE_DS3231<<1, reg_DS3231, sizebuf, &buffer_write, return_one_byre, timeout);
	    if(STATUS==HAL_OK)
			{
						#if DEBUG
								sprintf(str,"SET TIME \r\n");      // convert   in  str 
								size=sizeof(str);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str, size, 0xFFF);   
						#endif				
				    HAL_Delay(1000);
			}
      else
			{
				    while(1)
						{
							    #if DEBUG
											sprintf(str,"ERROR \r\n");      // convert   in  str 
											size=sizeof(str);
											HAL_UART_Transmit(&huart1 , (uint8_t *)str, size, 0xFFF);  
									#endif
									HAL_Delay(1000);
						}
						//Error hendler
			}
}


/*//////////////////////////////////////////////////////////////////////////////
This function called when we select second configuration.
This function illuminates seconds position on LCD. After pres button_2 we can
set seconds using button_2 for add seconds or button_3 for down seconds.
If we selected necessary second value we can seve it press button_4 neat one second.
*///////////////////////////////////////////////////////////////////////////////
void select_seconds(void)
{
	  // SET SEC
	  #if DEBUG
				// Prit in COM Port
				size=0;
				sprintf(str_UART,"SET SEC\r\n");      										// convert   in  str 
				size=sizeof(str_UART);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
	  #endif

		// Select blinking second in LCD
		sprintf(str_LCD,"%d",sec);
		ILI9341_Draw_Text(str_LCD, 230, 80, GREEN,3, RED);
		HAL_Delay(blinking_LCD);                                           // Period blincink selected LCD zone
		ILI9341_Draw_Text(str_LCD, 230, 80, GREEN,3, BLACK);

		if(button_2==STATUS_BUTTON_PRESS)       													// Select change seconds
		{
				uint8_t SEC=0;																								// Default start value seconds
			  STATUS=1;

				while(STATUS)
				{
						#if DEBUG
								sprintf(str_UART,"SEC: %d\r\n",SEC);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
					  #endif
						
						// Print sec in LCD
						sprintf(str_LCD,"%d",SEC);
						if(SEC<10)
						{
								sprintf(str_LCD_CLAER,"  ");
								ILI9341_Draw_Text(str_LCD_CLAER, 230, 80, GREEN,3, BLACK);  
								ILI9341_Draw_Text(str_LCD, 230, 80, GREEN,3, RED);
						}
						else
						{
								ILI9341_Draw_Text(str_LCD, 230, 80, GREEN,3, RED);
						}
						//
						HAL_Delay(blinking_LCD);	
						if(button_2==STATUS_BUTTON_PRESS)
						{
								SEC++;
								if(SEC>max_value_SEC)
								{
										SEC=min_value_MINUTE;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								SEC--;
								if(SEC>max_value_SEC)
								{
										SEC=min_value_MINUTE;	
								}
						}
						
						if(button_4==STATUS_BUTTON_PRESS)	   				// EXIT FROM SETTINGS	AND SAWE DATA IN KLOCK
						{
									STATUS=0;																// Exit from while loop
									WRITE_DATA(ADDRES_SEC_REG, SEC);
						}			  
				}
		}			
}


/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_minutes(void)
{
	   #if DEBUG
				 // SET MIN
				 sprintf(str_UART,"SET MIN\r\n");      										// convert   in  str 
				 size=sizeof(str_UART);
				 HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);		
     #endif	
		
		 // Select blinking min in LCD			
		 if(min==0)
		 {
				sprintf(str_LCD,"00");
				ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
				HAL_Delay(blinking_LCD);
				ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
		 }
		 else
		 {
				if(min<10)
				{
						sprintf(str_LCD,"0%d",min);
						ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
						HAL_Delay(blinking_LCD);
						ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
				}
				else
				{
						sprintf(str_LCD,"%d",min);
						ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
						HAL_Delay(blinking_LCD);
						ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
				 }
		  }
		
		 if(button_2==STATUS_BUTTON_PRESS)       // Select change hour
		 {
				uint8_t MIN=0;
				uint8_t STATUS=1;
				while(STATUS)
				{
					  #if DEBUG
								sprintf(str_UART,"MIN: %d\r\n",MIN);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
					  #endif
					
						// Print minute in LCD
						if(MIN==0)
						{
								sprintf(str_LCD,"00");
								ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
						}
						else
						{
								if(MIN<10)
								{
										sprintf(str_LCD,"0%d",MIN);
										ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
								}
								else
								{
										sprintf(str_LCD,"%d",MIN);
										ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, RED);
								}
						}
						//
						
						HAL_Delay(blinking_LCD);

						if(button_2==STATUS_BUTTON_PRESS)
						{
								MIN++;
								if(MIN>max_value_MINUTE)
								{
										MIN=min_value_MINUTE;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								MIN--;
								if(MIN>max_value_MINUTE)
								{
										MIN=min_value_MINUTE;	
								}
						}
					
						if(button_4==STATUS_BUTTON_PRESS)	   // EXIT FROM SETTINGS AND SAWE DATA IN KLOCK
						{
									STATUS=0;
									WRITE_DATA(ADDRES_MIN_REG, MIN);
						}			  
				}
		} 
}

/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_hours(void)
{
	  #if DEBUG
				sprintf(str_UART,"SET HOUR\r\n");      										// convert   in  str 
				size=sizeof(str_UART);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
		#endif

		// Select hour in LCD			
		if(hour==0)
		{
				sprintf(str_LCD,"00");
				ILI9341_Draw_Text(str_LCD, 0, 0, GREEN,10, RED);
				HAL_Delay(blinking_LCD);
				ILI9341_Draw_Text(str_LCD, 0, 0, GREEN,10, BLACK);
		}
		else
		{
				sprintf(str_LCD_CLAER,"  ");
				ILI9341_Draw_Text(str_LCD_CLAER, 0, 0, GREEN,10, BLACK);

				if(hour>9)
				{
						sprintf(str_LCD,"%d",hour);
						ILI9341_Draw_Text(str_LCD, 10, 0, GREEN,10, RED);
						HAL_Delay(blinking_LCD);
						ILI9341_Draw_Text(str_LCD, 10, 0, GREEN,10, BLACK);
				}
				else
				{
						sprintf(str_LCD_CLAER,"  ");
						ILI9341_Draw_Text(str_LCD_CLAER, 0, 0, GREEN,10, BLACK);
					
						sprintf(str_LCD,"%d",hour);
						ILI9341_Draw_Text(str_LCD, 50, 0, GREEN,10, RED);
						HAL_Delay(blinking_LCD);
						ILI9341_Draw_Text(str_LCD, 50, 0, GREEN,10, BLACK);
				}
		}

		if(button_2==STATUS_BUTTON_PRESS)       // Select change hour
		{
				uint8_t HOUR=0;
				uint8_t STATUS=1;
				while(STATUS)
				{			
						#if DEBUG
								sprintf(str_UART,"HOUR: %d\r\n",HOUR);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
						#endif
						
						// Print hours in LCD
						 if(HOUR==0)
						 {
								sprintf(str_LCD,"00");
								ILI9341_Draw_Text(str_LCD, 0, 0, GREEN,10, RED);
						 }
						 else
						 {
							
								if(HOUR>9)
								{
									  sprintf(str_LCD_CLAER,"  ");
								    ILI9341_Draw_Text(str_LCD_CLAER, 0, 0, GREEN,10, RED);
										sprintf(str_LCD,"%d",HOUR);
										ILI9341_Draw_Text(str_LCD, 10, 0, GREEN,10, RED);
								}
								else
								{
										sprintf(str_LCD_CLAER,"   ");
										ILI9341_Draw_Text(str_LCD_CLAER, 0, 0, GREEN,10, BLACK);	  
									
										char str3[4]="";
										sprintf(str3,":");
										ILI9341_Draw_Text(str3, 140, 0, GREEN,10, BLACK);
									
										
										sprintf(str_LCD,"%d",HOUR);
										ILI9341_Draw_Text(str_LCD, 50, 0, GREEN,10, RED);
									  
									  // Print Minutes
										if(min==0)
										{
												sprintf(str_LCD,"00");
												ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
										}
										else
										{
												if(min<10)
												{
														sprintf(str_LCD,"0%d",min);
														ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
												}
												else
												{
														sprintf(str_LCD,"%d",min);
														ILI9341_Draw_Text(str_LCD, 190, 0, GREEN,10, BLACK);
												}
										}
								}
						 }
						//
						HAL_Delay(blinking_LCD);
						 
						if(button_2==STATUS_BUTTON_PRESS)
						{
								HOUR++;
								if(HOUR>max_value_HOURS)
								{
										HOUR=min_value_HOURS;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								HOUR--;
								if(HOUR>max_value_HOURS)
								{
										HOUR=min_value_HOURS;	
								}
						}
					
						if(button_4==STATUS_BUTTON_PRESS)	   // EXIT
						{
									STATUS=0;
									WRITE_DATA(ADDRES_HOUR_REG, HOUR);
						}			  
				}
		}
}

/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_days(void)
{ 
	 #if DEBUG
			 // SET DAY
			 sprintf(str_UART,"SET DAY\r\n");      										// convert   in  str 
			 size=sizeof(str_UART);
			 HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
	 #endif	
	
	 // Select day in LCD	
		switch (day)
		{
		case 1:
			sprintf(str_LCD,"MONDAY");
		break;

		case 2:
			sprintf(str_LCD,"TUESDAY");
		break;

		case 3:
			sprintf(str_LCD,"WEDNESDAY");
		break;

		case 4:
			sprintf(str_LCD,"THUSDAY");
		break;

		case 5:
			sprintf(str_LCD,"FRIDAY");
		break;

		case 6:
			sprintf(str_LCD,"SATURDAY");
		break;

		case 7:
			sprintf(str_LCD,"SUNDAY");
		break;

		}
		ILI9341_Draw_Text(str_LCD, 0, 80, YELLOW,3, RED);
		HAL_Delay(blinking_LCD);
		ILI9341_Draw_Text(str_LCD, 0, 80, YELLOW,3, BLACK);
		
	 if(button_2==STATUS_BUTTON_PRESS)       // Select change hour
	 {
				uint8_t DAY=1;
				uint8_t STATUS=1;
				while(STATUS)
				{		
					  #if DEBUG
								sprintf(str_UART,"DAY: %d\r\n",DAY);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
					  #endif	

						switch (DAY)
						{
						case 1:
							sprintf(str_LCD,"MONDAY");
						break;

						case 2:
							sprintf(str_LCD,"TUESDAY");
						break;

						case 3:
							sprintf(str_LCD,"WEDNESDAY");
						break;

						case 4:
							sprintf(str_LCD,"THUSDAY");
						break;

						case 5:
							sprintf(str_LCD,"FRIDAY");
						break;

						case 6:
							sprintf(str_LCD,"SATURDAY");
						break;

						case 7:
							sprintf(str_LCD,"SUNDAY");
						break;
						}
						
						sprintf(str_LCD_CLAER,"            ");
						ILI9341_Draw_Text(str_LCD_CLAER, 0, 80, YELLOW,3, BLACK);
						ILI9341_Draw_Text(str_LCD, 0, 80, YELLOW,3, RED);
						//					
						HAL_Delay(blinking_LCD);
						if(button_2==STATUS_BUTTON_PRESS)
						{
								DAY++;
								if(DAY>max_value_DAYS)
								{
										DAY=min_value_DAYS;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								DAY--;
								if(DAY<min_value_DAYS)
								{
										DAY=min_value_DAYS;	
								}
						}
						// EXIT FROM SETTINGS		
						if(button_4==STATUS_BUTTON_PRESS)	   // EXIT
						{
									STATUS=0;
									WRITE_DATA(ADDRES_DAY_REG, DAY);
						}			  
		  }
	 }
}

/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_dates(void)
{ 
	  #if DEBUG
				sprintf(str_UART,"SET DATE\r\n");      										// convert   in  str 
				size=sizeof(str_UART);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
	  #endif

		// Select date in LCD	
		if(date<10)
		{
			sprintf(str_LCD,"%d",date);
			ILI9341_Draw_Text(str_LCD, 20, 100, BLUE,6, RED);
			HAL_Delay(blinking_LCD);
			ILI9341_Draw_Text(str_LCD, 20, 100, BLUE,6, BLACK);
		}
		else
		{
			sprintf(str_LCD,"%d",date);
			ILI9341_Draw_Text(str_LCD, 0, 100, BLUE,6, RED);
			HAL_Delay(blinking_LCD);
			ILI9341_Draw_Text(str_LCD, 0, 100, BLUE,6, BLACK);
		}

		if(button_2==STATUS_BUTTON_PRESS)       // Select change dates
		{
				uint8_t DATA=1;
				uint8_t STATUS=1;
				while(STATUS)
				{
					  #if DEBUG
								sprintf(str_UART,"DATE: %d\r\n",DATA);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
						#endif
					 
						// Print DATE in LCD
						sprintf(str_LCD_CLAER,"  ");
						if(DATA<10)
						{
								ILI9341_Draw_Text( str_LCD_CLAER, 0, 100, BLUE,6, BLACK);
								sprintf(str_LCD,"%d",DATA);
								ILI9341_Draw_Text(str_LCD, 20, 100, BLUE,6, RED);
						}
						else
						{
								sprintf(str_LCD,"%d",DATA);
								ILI9341_Draw_Text(str_LCD, 0, 100, BLUE,6, RED);
						}
						//
						 
						HAL_Delay(blinking_LCD);
						if(button_2==STATUS_BUTTON_PRESS)
						{
								DATA++;
								if(DATA>max_value_DATA)
								{
										DATA=min_value_DATA;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								DATA--;
								if(DATA<min_value_DATA)
								{
										DATA=min_value_DATA;	
								}
						}

						// EXIT FROM SETTINGS		
						if(button_4==STATUS_BUTTON_PRESS)	   // EXIT
						{
									STATUS=0;
									WRITE_DATA(ADDRES_DATE_REG, DATA);
						}			  
				}
		}
}
									 
									 							 
/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_month(void)
{ 
		#if DEBUG
				sprintf(str_UART,"SET MONTH\r\n");      										// convert   in  str 
				size=sizeof(str_UART);
				HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
	  #endif

		// Select mounth in LCD	
		if(month<10)
		{
				sprintf(str_LCD,"%d",month);
				ILI9341_Draw_Text(str_LCD, 120, 100, BLUE,6, RED);
				HAL_Delay(blinking_LCD);
				ILI9341_Draw_Text(str_LCD, 120, 100, BLUE,6, BLACK);
		}
		else
		{
				sprintf(str_LCD,"%d",month);
				ILI9341_Draw_Text(str_LCD, 100, 100, BLUE,6, RED);
				HAL_Delay(blinking_LCD);
				ILI9341_Draw_Text(str_LCD, 100, 100, BLUE,6, BLACK);
		}

		if(button_2==STATUS_BUTTON_PRESS)       // Select change hour
		{
				uint8_t MONTH=1;
				uint8_t STATUS=1;
				while(STATUS)
				{
					  #if DEBUG
								sprintf(str_UART,"MONTH: %d\r\n",MONTH);      										// convert   in  str 
								size=sizeof(str_UART);
								HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
					  #endif

						// Print MONTH in LCD
						if(MONTH<10)
						{
								sprintf(str_LCD,"%d",MONTH);
							  sprintf(str_LCD_CLAER,"  ");
								ILI9341_Draw_Text(str_LCD_CLAER, 100, 100, BLUE,6, BLACK);
								ILI9341_Draw_Text(str_LCD, 120, 100, BLUE,6, RED);
						}
						else
						{
								sprintf(str_LCD,"%d",MONTH);
								ILI9341_Draw_Text(str_LCD, 100, 100, BLUE,6, RED);
						}
						//

						HAL_Delay(blinking_LCD);
						if(button_2==STATUS_BUTTON_PRESS)
						{
								MONTH++;
								if(MONTH>max_value_MOUNTH)
								{
										MONTH=min_value_MOUNTH;	
								}
						}
						if(button_3==STATUS_BUTTON_PRESS)
						{
								MONTH--;
								if(MONTH<min_value_MOUNTH)
								{
										MONTH=min_value_MOUNTH;	
								}
						}

						// EXIT FROM SETTINGS		
						if(button_4==STATUS_BUTTON_PRESS)	   // EXIT
						{
									STATUS=0;
									WRITE_DATA(ADDRES_MOUNTH_REG, MONTH);
						}			  
				}
		}	
}


/*//////////////////////////////////////////////////////////////////////////////
similar to like select_seconds
*///////////////////////////////////////////////////////////////////////////////
void select_year(void)
{ 
	    #if DEBUG
					sprintf(str_UART,"SET YEAR\r\n");      										// convert   in  str 
					size=sizeof(str_UART);
					HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFF);
	    #endif

			// Select year in LCD		 
			if(year<10)
			{
					sprintf(str_LCD,"%d",year);
					ILI9341_Draw_Text(str_LCD, 220, 100, BLUE,6, RED);
					HAL_Delay(blinking_LCD);
					ILI9341_Draw_Text(str_LCD, 220, 100, BLUE,6, BLACK);
			}
			else
			{
					sprintf(str_LCD,"%d",year);
					ILI9341_Draw_Text(str_LCD, 200, 100, BLUE,6, RED);
					HAL_Delay(blinking_LCD);
					ILI9341_Draw_Text(str_LCD, 200, 100, BLUE,6, BLACK);
			}	 
			 
			if(button_2==STATUS_BUTTON_PRESS)       // Select change hour
			{
					uint8_t YEAR=1;
					uint8_t STATUS=1;
					while(STATUS)
					{
						  #if DEBUG
									sprintf(str_UART,"YEAR: %d\r\n",YEAR);      										// convert   in  str 
									size=sizeof(str_UART);
									HAL_UART_Transmit(&huart1 , (uint8_t *)str_UART, size, 0xFFFF);
							#endif

							// Print MONTH in LCD
							if(YEAR<10)
							{
									sprintf(str_LCD_CLAER,"  ");
									sprintf(str_LCD,"%d",YEAR);
									ILI9341_Draw_Text(str_LCD_CLAER, 200, 100, BLUE,6, BLACK);
									ILI9341_Draw_Text(str_LCD, 220, 100, BLUE,6, RED);
							}
							else
							{
									sprintf(str_LCD,"%d",YEAR);
									ILI9341_Draw_Text(str_LCD, 200, 100, BLUE,6, RED);
							}	
							// 				 
							HAL_Delay(200);

							if(button_2==STATUS_BUTTON_PRESS)
							{
									YEAR++;
									if(YEAR>max_value_YEAR)
									{
											YEAR=min_value_YEAR;	
									}
							}
							if(button_3==STATUS_BUTTON_PRESS)
							{
									YEAR--;
									if(YEAR<min_value_YEAR)
									{
											YEAR=min_value_YEAR;	
									}
							}

							// EXIT FROM SETTINGS		
							if(button_4==STATUS_BUTTON_PRESS)	   // EXIT
							{
									STATUS=0;
									WRITE_DATA(ADDRES_YEAR_REG, YEAR);
							}			  
					}
		}
}


/*//////////////////////////////////////////////////////////////////////////////
This function  print on LCD static object like lines and some words.
*///////////////////////////////////////////////////////////////////////////////
void init_LCD_ILI9341(void)
{
		ILI9341_Init();    // Init LCD.
	  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);				
		ILI9341_Fill_Screen(BLACK);

	  sprintf(str_LCD,"OUTDOOR");
		ILI9341_Draw_Text(str_LCD, 160, 160, BLUE,2, BLACK);
	  
	  sprintf(str_LCD,"T:");
		ILI9341_Draw_Text(str_LCD, 7, 183, BLUE,3, BLACK);
	
	  sprintf(str_LCD,"H:");
		ILI9341_Draw_Text(str_LCD, 7, 213, BLUE,3, BLACK);
	  
	  sprintf(str_LCD,"P:");
		ILI9341_Draw_Text(str_LCD, 7, 154, BLUE,3, BLACK);
	  
	  // Draw grafity
		ILI9341_Draw_Rectangle(0, 151, 143, 3, BLUE);  // First line  
	  ILI9341_Draw_Rectangle(0, 180, 248, 3, BLUE);   //  Second line
	  ILI9341_Draw_Rectangle(0, 210, 248, 3, BLUE);   //  Third line
	  ILI9341_Draw_Rectangle(0, 237, 248, 3, BLUE);   // 
		
		// Draw line
		ILI9341_Draw_Rectangle(0, 151, 3, 90, BLUE);
		ILI9341_Draw_Rectangle(35, 151, 3, 90, BLUE);
		ILI9341_Draw_Rectangle(140, 151, 3, 90, BLUE);
		ILI9341_Draw_Rectangle(245, 180, 3, 60, BLUE);
}

void print_data_on_LCD(int optimization)
{  
	  if(optimization==optimization_ON)
		{
				print_seconds_in_LCD();  		// If second =0 rewrite all  data in LCD.
			  if(sec==0)
				{
					  print_colon_in_LCD();
						print_minutes_in_LCD();
						print_hours_in_LCD();		
						print_days_in_LCD();
						print_dates_in_LCD();
						print_months_in_LCD();
						print_year_in_LCD();
						print_MEMS_in_LCD();
				}					
		}
		if(optimization==optimization_OFF)
		{
			  print_colon_in_LCD();
				print_seconds_in_LCD();
				print_minutes_in_LCD();
				print_hours_in_LCD();		
				print_days_in_LCD();
				print_dates_in_LCD();
				print_months_in_LCD();
				print_year_in_LCD();
				print_MEMS_in_LCD();						
		}
}

void print_colon_in_LCD(void)
{	  
	  char str3[1]="";
	  sprintf(str3,":");
		ILI9341_Draw_Text(str3, 140, 0, GREEN,10, BLACK);
}

void print_seconds_in_LCD(void)
{
		char str[10]="";
		if(sec==0)													// Clear seconds position
		{
			  sprintf(str,"  ");
		    ILI9341_Draw_Text(str, 230, 80, GREEN,3, BLACK);
		}
		sprintf(str,"%d",sec);
		ILI9341_Draw_Text(str, 230, 80, GREEN,3, BLACK);
}


void print_minutes_in_LCD(void)
{
	  char str[10]="";
		if(min==0)
		{
				sprintf(str,"00");							
				ILI9341_Draw_Text(str, 190, 0, GREEN,10, BLACK);
		}
		else
		{
				if(min<10)
				{
						sprintf(str,"0%d",min);
						ILI9341_Draw_Text(str, 190, 0, GREEN,10, BLACK);
				}
				else
				{
						sprintf(str,"%d",min);
						ILI9341_Draw_Text(str, 190, 0, GREEN,10, BLACK);
				}
		}
	
}

void print_hours_in_LCD(void)
{
	  char str[10]="";
		if(hour==0)
		{
				sprintf(str,"00");
				ILI9341_Draw_Text(str, 0, 0, GREEN,10, BLACK);
		}
		else
		{
				sprintf(str,"  ");
				ILI9341_Draw_Text(str, 0, 0, GREEN,10, BLACK);
			
				if(hour>9)
				{
						sprintf(str,"%d",hour);
						ILI9341_Draw_Text(str, 10, 0, GREEN,10, BLACK);
				}
				else
				{
						sprintf(str,"%d",hour);
						ILI9341_Draw_Text(str, 50, 0, GREEN,10, BLACK);
				}
		}
}

void print_days_in_LCD(void)
{
		char str_day[10]="";
		switch (day)
		{
			case 1:
				sprintf(str_day,"MONDAY");
			break;
			
			case 2:
				sprintf(str_day,"TUESDAY");
			break;
			
			case 3:
				sprintf(str_day,"WEDNESDAY");
			break;
			
			case 4:
				sprintf(str_day,"THUSDAY");
			break;
			
			case 5:
				sprintf(str_day,"FRIDAY");
			break;
			
			case 6:
				sprintf(str_day,"SATURDAY");
			break;
			
			case 7:
				sprintf(str_day,"SUNDAY");
			break;
		}
		
		ILI9341_Draw_Text(str_day, 0, 80, YELLOW,3, BLACK);
}


void print_dates_in_LCD(void)
{
		char str[10]="";

		if(date<10)
		{
				sprintf(str,"%d",date);
				ILI9341_Draw_Text(str, 20, 100, BLUE,6, BLACK);
		}
		else
		{
				sprintf(str,"%d",date);
				ILI9341_Draw_Text(str, 0, 100, BLUE,6, BLACK);
		}

		sprintf(str,"/");
		ILI9341_Draw_Text(str, 75, 100, BLUE,6, BLACK);
		if(date<10)
		{
				sprintf(str,"%d",date);
				ILI9341_Draw_Text(str, 20, 100, BLUE,6, BLACK);
		}
		else
		{
				sprintf(str,"%d",date);
				ILI9341_Draw_Text(str, 0, 100, BLUE,6, BLACK);
		}

		sprintf(str,"/");
		ILI9341_Draw_Text(str, 75, 100, BLUE,6, BLACK);
}

void print_months_in_LCD(void)
{
		char str[10]="";
	
		if(month<10)
		{
				sprintf(str,"%d",month);
				ILI9341_Draw_Text(str, 120, 100, BLUE,6, BLACK);
		}
		else
		{
				sprintf(str,"%d",month);
				ILI9341_Draw_Text(str, 100, 100, BLUE,6, BLACK);
		}
		
		sprintf(str,"/");
		ILI9341_Draw_Text(str, 175, 100, BLUE,6, BLACK);
}

void print_year_in_LCD(void)
{
		char str[10]="";
		
		if(year<10)
		{
				sprintf(str,"%d",year);
				ILI9341_Draw_Text(str, 220, 100, BLUE,6, BLACK);
		}
		else
		{
				sprintf(str,"%d",year);
				ILI9341_Draw_Text(str, 200, 100, BLUE,6, BLACK);
		}
}


void print_MEMS_in_LCD(void)
{
		char str[10]="";
		// Print indoor temperature //--------------------------------------------
	  //T_indoor=-99;
		sprintf(str,"%.1f",T_indoor);
		ILI9341_Draw_Text(str, 60, 183, BLUE,3, BLACK);
	  // Print indoor preasure
	  sprintf(str,"%d",H_indoor);
		ILI9341_Draw_Text(str,60, 213, BLUE,3, BLACK);
	  // Draw preasure plase
		sprintf(str_LCD,"%d",P);
		ILI9341_Draw_Text(str_LCD, 60, 154, BLUE,3, BLACK); 

		// Print outdor temperature  //--------------------------------------------
	  if(STATUS_CONNECTION_TO_OUTDOR==0) // If no connection
		{   
			  // Nead clear Clear 
				sprintf(str,"    ");
			  ILI9341_Draw_Text(str, 155, 184, BLUE,3, BLACK);
				
				// Print outdor humidyty 
				sprintf(str,"    ");
			  ILI9341_Draw_Text(str, 165, 213, BLUE,3, BLACK);
				
		}
		else{
				//T_outdoor=-99;
				sprintf(str,"%.1f",T_outdoor);
				ILI9341_Draw_Text(str, 155, 184, BLUE,3, BLACK);
				// Print outdor humidyty 
				sprintf(str,"%d%%",H_outdoor);
				ILI9341_Draw_Text(str, 165, 213, BLUE,3, BLACK);
		}
	  
		
}

























