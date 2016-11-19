
#ifndef __SysTickDelay_H
#define __SysTickDelay_H

#include "stm32f4xx.h" 

void SysTick_Initaize(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_s(uint16_t t);

#endif /* __EVAL_H */
/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */     

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
