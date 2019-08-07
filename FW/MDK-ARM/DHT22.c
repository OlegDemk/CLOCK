#include "DHT22.h"

extern UART_HandleTypeDef huart1;

void port_init(void);
uint8_t DHD22_init(void);
uint8_t DHD22_GetData(uint8_t *data);

//--------------------------------------------------
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 1000000) / 9;
	/* Wait till done */
	while (micros--) ;
}
//--------------------------------------------------
void port_init(void)
{
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
  GPIOA->CRL |= GPIO_CRL_MODE4;
  GPIOA->CRL |= GPIO_CRL_CNF4_0;
  GPIOA->CRL &= ~GPIO_CRL_CNF4_1;
}
//--------------------------------------------------
uint8_t DHD22_init(void)
{
		HAL_Delay(2000);
		GPIOA->ODR |= GPIO_ODR_ODR4;   // High level
	
//  uint16_t status;
//	GPIOA->ODR &= ~GPIO_ODR_ODR4;
//	//DelayMicro(485);
//	HAL_Delay(18);
//	GPIOA->ODR |= GPIO_ODR_ODR4;
//	DelayMicro(800);
//	status = GPIOA->IDR & GPIO_IDR_IDR4;
//	DelayMicro(500);
return 0;	
}
//--------------------------------------------------
uint8_t DHD22_GetData(uint8_t *data)
{
	uint8_t i, j=0;
	GPIOA->ODR &= ~GPIO_ODR_ODR4;			// Low level
	GPIOA->ODR |= GPIO_ODR_ODR4;   // High level
	DelayMicro(100000);
	GPIOA->ODR &= ~GPIO_ODR_ODR4;			// Low level
	
	// Send START signal
	DelayMicro(18000);
	GPIOA->ODR |= GPIO_ODR_ODR4;   // High level
	DelayMicro(39);
	
	// Response from DHT22
	if(GPIOA->IDR & GPIO_IDR_IDR4)
	{
		  char str1[15];
		  sprintf(str1,"EROR: sensor not response \n\r");
  	  HAL_UART_Transmit(&huart1,(uint8_t*)str1,strlen(str1),0x1000);
	    HAL_Delay(1000);
		  return 0; 
	}
	
	 
	
}
//--------------------------------------------------