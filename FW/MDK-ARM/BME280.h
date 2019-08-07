#include "main.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

	/////// Variable fir BME280
// 1. Memory map
#define BME280_HUM_LSB				0xFE	    //7:0
#define BME280_HUM_MSB				0xFD			//7:0
#define BME280_TEMP_XLSB			0xFC 			//7:4
#define BME280_TEMP_LSB				0xFB			//7:0
#define BME280_TEMP_MSB				0xFA			//7:0
#define BME280_PRESS_XLSB			0xF9			//7:4		
#define BME280_PRESS_LSB			0xF8			//7:0
#define BME280_PRESS_MSB			0xF7			//7:4

#define BME280_CONFIG					0xF5     
#define BME280_CTRL_MEAS			0xF4
#define BME280_CTRL_HUM				0xF2

// Calibrate registers
#define BME280_CALIB_26				0xE1
#define BME280_CALIB_41				0xF0

#define BME280_RESET 					0xE0

#define BME280_ID_ADDRES			0xD0

// Calibrate registers
#define BME280_CALIB_00				0x88
#define BME280_CALIB_25				0xA1

////////////////////////////////////////////////////////

// 2. Read ID BME280
uint8_t STATUS_BME280=HAL_ERROR;
#define BME280_ID             0x76
uint8_t buff_id=0;
char str_BME280[60]="";
extern uint8_t size;
//////

// Variable for read T, H and P DATA
unsigned short dig_T1_LSB=0, dig_T1_MSB=0, dig_T1=0;
short dig_T2_LSB=0, dig_T2_MSB=0, dig_T2=0;
short dig_T3_LSB=0, dig_T3_MSB=0, dig_T3=0;

// Variable for read calibrate data
unsigned short dig_P1=0;
signed short dig_P2=0;
signed short dig_P3=0;
signed short dig_P4=0;
signed short dig_P5=0;
signed short dig_P6=0;
signed short dig_P7=0;
signed short dig_P8=0;
signed short dig_P9=0;

//unsigned short dig_H1=0;
//signed short dig_H2=0;
//unsigned short dig_H3=0;
//signed short dig_H4=0;
//signed short dig_H5=0;
//signed short dig_H6=0;



//float cTemp=0;
