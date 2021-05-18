#ifndef  INC_SHAKTIVAJRA_GPIO_DRIVER_H_
#define  INC_SHAKTIVAJRA_GPIO_DRIVER_H_

#include "stdint.h"

/*!General Purpose Input / Output */
#define GPIO_START 0x00040100 //GPIO Start Address */
#define GPIO_OFFSET 0x08 /*!Generic offset used to access GPIO registers*/
#define GPIO_DIRECTION_CNTRL_REG (uint32_t*) (GPIO_START  + (0 * GPIO_OFFSET ))
#define GPIO_DATA_REG (uint32_t*) (GPIO_START + (1 * GPIO_OFFSET ))
/*
 * General Purpose IOs supported
 */

#define GPIO0  (1 <<  0)
#define GPIO1  (1 <<  1)
#define GPIO2  (1 <<  2)
#define GPIO3  (1 <<  3)
#define GPIO4  (1 <<  4)
#define GPIO5  (1 <<  5)
#define GPIO6  (1 <<  6)
#define GPIO7  (1 <<  7)
#define GPIO8  (1 <<  8)
#define GPIO9  (1 <<  9)
#define GPIO10 (1 << 10)
#define GPIO11 (1 << 11)
#define GPIO12 (1 << 12)
#define GPIO13 (1 << 13)
#define GPIO14 (1 << 14)
#define GPIO15 (1 << 15)
#define GPIO16 (1 << 16)
#define GPIO17 (1 << 17)
#define GPIO18 (1 << 18)
#define GPIO19 (1 << 19)
#define GPIO20 (1 << 20)
#define GPIO21 (1 << 21)
#define GPIO22 (1 << 22)
#define GPIO23 (1 << 23)
#define GPIO24 (1 << 24)
#define GPIO25 (1 << 25)
#define GPIO26 (1 << 26)
#define GPIO27 (1 << 27)
#define GPIO28 (1 << 28)
#define GPIO29 (1 << 29)
#define GPIO30 (1 << 30)
#define GPIO31 (1 << 31)

//some macros
#define ENABLE 	 1	
#define DISABLE  0
#define SET	ENABLE
#define RESET	DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#define GPIO ((GPIO_RegDef_t *)GPIO_START);

typedef struct{
	volatile uint32_t GPIO_DIRECTION_CNTRL_REG;
	volatile uint32_t GPIO_DATA_REG;
}GPIO_RegDef_t;


typedef struct{
  volatile uint32_t GPIO_DIRECTION_CNTRL_REG;
  volatile uint32_t GPIO_DATA_REG;  
}GPIO_PinConfig_t;

typedef struct{
  GPIO_RegDef_t *pGPIO;
  GPIO_PinConfig_t GPIO_PinConfig;  //for configuration settings
}GPIO_Handle_t;

/*
* Peripheral Clock setup
*/
//void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIO, unit8_t EnorDi);

/*
* Init and DeInit
*/
//void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
//void GPIO_DeInit(GPIO_RegDef_t *pGPIO);

/*
* Data Read and Write
*/
uint32_t GPIO_ReadFromInputPin((GPIO_RegDef_t *pGPIO); //No pin number required as there are no ports
//uint32_t GPIO_ReadFromInputPort((GPIO_RegDef_t *pGPIO);
void GPIO_WriteToOutputPin((GPIO_RegDef_t *pGPIO, uint32_t Value);
//void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO);

/*
* IRQ Configuration and ISR Handling
*/
//void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
//void GPIO_IRQHandling(GPIO_RegDef_t *pGPIO);
	
/*
 * delay
 */
void delay_loop(uint32_t cntr1, uint32_t cntr2);

#endif
