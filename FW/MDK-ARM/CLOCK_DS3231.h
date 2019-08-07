#include "main.h"

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

// Variabble for botton
extern int STATUS_BUTTON_PRESS;
extern int STATUS_BUTTON_NOT_PRESS;

// Init buttons.
extern int button_1;
extern int button_2;
extern int button_3;
extern int button_4;

extern char str_uatr[10];					
extern uint8_t size;

extern float cTemp;
extern uint16_t humidity;
extern uint16_t preasure;

extern int optimization_ON;					
extern int optimization_OFF;

extern char str_UART[10];						// Value for print all data in COM Port
extern char str_LCD[20];			 				// Value for print all data in LCD
extern char str_LCD_CLAER[2];				// Value for clear two item in LCD

extern int blinking_LCD;

uint8_t STATUS_CONNECTION_TO_OUTDOR=0; // 0- no connection 1- connection

// Value for change clock
extern uint8_t max_value_SEC;
extern uint8_t min_value_SEC;
extern uint8_t max_value_MINUTE;
extern uint8_t min_value_MINUTE;
extern uint8_t max_value_HOURS;
extern uint8_t min_value_HOURS;
extern uint8_t max_value_DAYS;
extern uint8_t min_value_DAYS;
extern uint8_t max_value_DATA;
extern uint8_t min_value_DATA;
extern uint8_t max_value_MOUNTH;
extern uint8_t min_value_MOUNTH;
extern uint8_t max_value_YEAR;
extern uint8_t min_value_YEAR;

char str[35]={0};															// Buffer for transmit data in comport.
// Registers for DS3231
uint16_t ADRESS_DEVCE_DS3231=0x68;						// Devaice addres.
uint16_t ADDRES_SEC_REG=0x00;                 // Addres second.
uint16_t ADDRES_MIN_REG=0x01;									// Addres ninutes.
uint16_t ADDRES_HOUR_REG=0x02;								// Addres hour.
uint16_t ADDRES_DAY_REG=0x03;									// Addres day.
uint16_t ADDRES_DATE_REG=0x04;								// Addres date.
uint16_t ADDRES_MOUNTH_REG=0x05;							// Addres mounth.
uint16_t ADDRES_YEAR_REG=0x06;								// Addres year.
uint16_t Read_sec=0x00;												// Start reading register.

uint8_t reg_DS3231=0;													// Register
uint8_t data_to_DS3231=0;											// Data setd to clock like hout/sec/..
// Register for I2C communicate with DS3231.
uint8_t STATUS=1;															// Status error  transmit /receive data.
uint8_t buff[7]={0};													// Buffer for read data.
uint16_t sizebuf_data_return=7;								// How money bytes must return from devaice.
uint16_t return_one_byre=1;           				// How money bytes must return from devaice.
uint16_t sizebuf=1;														// Sendet data in devaice.
uint32_t timeout=1000;

// Variabble for ready data
uint8_t sec=0;
uint8_t min=0;
uint8_t hour=0;
uint8_t day=0;
uint8_t date=0;
uint8_t month=0;
uint8_t year=0;

// Prototipe function.
void read_data_from_DS3231(void);
uint8_t RTC_ConvertFromDec(uint8_t c);
uint8_t RTC_ConvertFromBinDec(uint8_t c);
void WRITE_DATA(uint16_t reg_DS3231, uint8_t data_to_DS3231);
void start_init_DS3231(void);
void print_clear_colon_in_LCD(void);
void print_colon_in_LCD(void);
void print_seconds_in_LCD(void);
void print_minutes_in_LCD(void);
void print_hours_in_LCD(void);
void print_days_in_LCD(void);
void print_dates_in_LCD(void);
void print_months_in_LCD(void);
void print_year_in_LCD(void);
void print_MEMS_in_LCD(void);

