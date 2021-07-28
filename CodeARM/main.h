#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx.h"
__IO uint32_t uwTimingDelay;
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

								

/* FUNCTION------------------------------------------------------------------*/
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}


void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}


#endif /* __MAIN_H */


