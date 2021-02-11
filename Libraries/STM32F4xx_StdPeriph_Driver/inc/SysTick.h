#ifndef __INC_SYSTICK_H_
#define __INC_SYSTICK_H_

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include "stm32f4xx.h"
void SysTick_IncrementTicks_cb(void);


uint32_t SysTick_GetCurrentTick(void);


void SysTick_Delay(uint32_t wait_time_ms);


void SysTick_Init(void);



/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /*__INC_SYSTICK_H_ */
