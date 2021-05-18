#include "GPIO_driver.h"

int main(void){
GPIO_Handle_t GPIO_LED;
GPIO_LED.GPIO_PinConfig.GPIO_DIRECTION_CNTRL_REG = 0x00000FFF;
  
while(1){
  GPIO_WriteToOutputPin(GPIO_LED.GPIO_PinConfig.GPIO_DATA_REG, GPIO0);
  delay_loop(1000, 1000);
  GPIO_WriteToOutputPin(GPIO_LED.GPIO_PinConfig.GPIO_DATA_REG, 0x00);
  delay_loop(1000,1000);
}  
return 0;
}
