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
#define BME280_CALIB_dig_H1   0xA1

////////////////////////////////////////////////////////

// 2. Read ID BME280
uint8_t STATUS_BME280=HAL_ERROR;
#define BME280_ID             0x76
uint8_t buff_id=0;
char str_BME280[60]="";
extern uint8_t size;
//////

// Variable for read T, H and P DATA
typedef int32_t BME280_S32_t;
typedef uint32_t BME280_U32_t;
typedef int64_t BME280_S64_t;
 
// dig T
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
 
// dig P
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
 
// dig H1
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

// Wariabble for temperature
BME280_S32_t t_fine;

int t=0;
int p=0;
int h=0;

BME280_S32_t adc_T, adc_P, adc_H;
uint8_t ALL_REGISTERS[255];


//float cTemp=0;
