#include "SysTick.h"


volatile uint32_t SysTickCounter = 0;

void SysTick_IncrementTicks_cb(void)
{
  ++SysTickCounter;
}
uint32_t SysTick_GetCurrentTick(void)
{
  return(SysTickCounter);
}
void SysTick_Delay(uint32_t wait_time_ms)
{
  /* Store start tick */
  uint32_t startTick = SysTickCounter;

  /* Loop until timeout */
  while((SysTickCounter - startTick) < wait_time_ms)
  {

  }
}
void SysTick_Init(void)
{
  uint32_t returnCode;

  /* Update clock configuration */
  SystemCoreClockUpdate();

  /* Check clock configuration */
  if(SystemCoreClock != (uint32_t) 180000000)
  {
    /* Clock configuration is not OK */
    while(1)
    {

    }
  }
  else
  {
    /* Clock configuration is OK */
  }

  /* Configure SysTick to generate an interrupt every millisecond */
  returnCode = SysTick_Config(SystemCoreClock / 1000);

  /* Check return code for errors */
  if (returnCode != 0)
  {
    /* SysTick configuration failed */
    while(1)
    {

    }
  }
  else
  {
    /* Do nothing, SysTick configuration OK */
  }
}
