/**
  ******************************************************************************
  * @file    stm32f4xx_flash.c
  * @author  MCD Application Team
  * @version V1.2.0RC3
  * @date    21-June-2013
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the FLASH peripheral:
  *            + FLASH Interface configuration
  *            + FLASH Memory Programming
  *            + Option Bytes Programming
  *            + Interrupts and flags management
  *  
 @verbatim    
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]                             
      This driver provides functions to configure and program the FLASH memory 
      of all STM32F4xx devices. These functions are split in 4 groups:
   
      (#) FLASH Interface configuration functions: this group includes the
          management of the following features:
        (++) Set the latency
        (++) Enable/Disable the prefetch buffer
        (++) Enable/Disable the Instruction cache and the Data cache
        (++) Reset the Instruction cache and the Data cache
    
      (#) FLASH Memory Programming functions: this group includes all needed
          functions to erase and program the main memory:
        (++) Lock and Unlock the FLASH interface
        (++) Erase function: Erase sector, erase all sectors
        (++) Program functions: byte, half word, word and double word
    
      (#) Option Bytes Programming functions: this group includes all needed
          functions to manage the Option Bytes:
        (++) Set/Reset the write protection
        (++) Set the Read protection Level
        (++) Set the BOR level
        (++) Program the user Option Bytes
        (++) Launch the Option Bytes loader
    
      (#) Interrupts and flags management functions: this group 
          includes all needed functions to:
        (++) Enable/Disable the FLASH interrupt sources
        (++) Get flags status
        (++) Clear flags
        (++) Get FLASH operation status
        (++) Wait for last FLASH operation   
 @endverbatim                      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_flash.h"
#include <stddef.h>

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH 
  * @brief FLASH driver modules
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 
#define SECTOR_MASK               ((uint32_t)0xFFFFFF07)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup FLASH_Private_Functions
  * @{
  */ 

/** @defgroup FLASH_Group1 FLASH Interface configuration functions
  *  @brief   FLASH Interface configuration functions 
 *

@verbatim   
 ===============================================================================
              ##### FLASH Interface configuration functions #####
 ===============================================================================
    [..]
      This group includes the following functions:
      (+) void FLASH_SetLatency(uint32_t FLASH_Latency)
          To correctly read data from FLASH memory, the number of wait states (LATENCY) 
          must be correctly programmed according to the frequency of the CPU clock 
          (HCLK) and the supply voltage of the device.
    [..]      
      For STM32F405xx/07xx and STM32F415xx/17xx devices
 +-------------------------------------------------------------------------------------+     
 | Latency       |                HCLK clock frequency (MHz)                           |
 |               |---------------------------------------------------------------------|     
 |               | voltage range  | voltage range  | voltage range   | voltage range   |
 |               | 2.7 V - 3.6 V  | 2.4 V - 2.7 V  | 2.1 V - 2.4 V   | 1.8 V - 2.1 V   |
 |---------------|----------------|----------------|-----------------|-----------------|              
 |0WS(1CPU cycle)|0 < HCLK <= 30  |0 < HCLK <= 24  |0 < HCLK <= 22   |0 < HCLK <= 20   |
 |---------------|----------------|----------------|-----------------|-----------------|   
 |1WS(2CPU cycle)|30 < HCLK <= 60 |24 < HCLK <= 48 |22 < HCLK <= 44  |20 < HCLK <= 40  | 
 |---------------|----------------|----------------|-----------------|-----------------|   
 |2WS(3CPU cycle)|60 < HCLK <= 90 |48 < HCLK <= 72 |44 < HCLK <= 66  |40 < HCLK <= 60  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |3WS(4CPU cycle)|90 < HCLK <= 120|72 < HCLK <= 96 |66 < HCLK <= 88  |60 < HCLK <= 80  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |4WS(5CPU cycle)|120< HCLK <= 150|96 < HCLK <= 120|88 < HCLK <= 110 |80 < HCLK <= 100 |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |5WS(6CPU cycle)|150< HCLK <= 168|120< HCLK <= 144|110 < HCLK <= 132|100 < HCLK <= 120| 
 |---------------|----------------|----------------|-----------------|-----------------| 
 |6WS(7CPU cycle)|      NA        |144< HCLK <= 168|132 < HCLK <= 154|120 < HCLK <= 140| 
 |---------------|----------------|----------------|-----------------|-----------------| 
 |7WS(8CPU cycle)|      NA        |      NA        |154 < HCLK <= 168|140 < HCLK <= 160|
 +---------------|----------------|----------------|-----------------|-----------------+ 

    [..]      
      For STM32F42xxx/43xxx devices
 +-------------------------------------------------------------------------------------+     
 | Latency       |                HCLK clock frequency (MHz)                           |
 |               |---------------------------------------------------------------------|     
 |               | voltage range  | voltage range  | voltage range   | voltage range   |
 |               | 2.7 V - 3.6 V  | 2.4 V - 2.7 V  | 2.1 V - 2.4 V   | 1.8 V - 2.1 V   |
 |---------------|----------------|----------------|-----------------|-----------------|              
 |0WS(1CPU cycle)|0 < HCLK <= 30  |0 < HCLK <= 24  |0 < HCLK <= 22   |0 < HCLK <= 20   |
 |---------------|----------------|----------------|-----------------|-----------------|   
 |1WS(2CPU cycle)|30 < HCLK <= 60 |24 < HCLK <= 48 |22 < HCLK <= 44  |20 < HCLK <= 40  | 
 |---------------|----------------|----------------|-----------------|-----------------|   
 |2WS(3CPU cycle)|60 < HCLK <= 90 |48 < HCLK <= 72 |44 < HCLK <= 66  |40 < HCLK <= 60  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |3WS(4CPU cycle)|90 < HCLK <= 120|72 < HCLK <= 96 |66 < HCLK <= 88  |60 < HCLK <= 80  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |4WS(5CPU cycle)|120< HCLK <= 150|96 < HCLK <= 120|88 < HCLK <= 110 |80 < HCLK <= 100 |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |5WS(6CPU cycle)|120< HCLK <= 180|120< HCLK <= 144|110 < HCLK <= 132|100 < HCLK <= 120| 
 |---------------|----------------|----------------|-----------------|-----------------| 
 |6WS(7CPU cycle)|      NA        |144< HCLK <= 168|132 < HCLK <= 154|120 < HCLK <= 140| 
 |---------------|----------------|----------------|-----------------|-----------------| 
 |7WS(8CPU cycle)|      NA        |168< HCLK <= 180|154 < HCLK <= 176|140 < HCLK <= 160|
 |---------------|----------------|----------------|-----------------|-----------------| 
 |8WS(9CPU cycle)|      NA        |      NA        |176 < HCLK <= 180|160 < HCLK <= 168|
 +-------------------------------------------------------------------------------------+
   
    [..]
    For STM32F401x devices
 +-------------------------------------------------------------------------------------+     
 | Latency       |                HCLK clock frequency (MHz)                           |
 |               |---------------------------------------------------------------------|     
 |               | voltage range  | voltage range  | voltage range   | voltage range   |
 |               | 2.7 V - 3.6 V  | 2.4 V - 2.7 V  | 2.1 V - 2.4 V   | 1.8 V - 2.1 V   |
 |---------------|----------------|----------------|-----------------|-----------------|              
 |0WS(1CPU cycle)|0 < HCLK <= 30  |0 < HCLK <= 24  |0 < HCLK <= 22   |0 < HCLK <= 20   |
 |---------------|----------------|----------------|-----------------|-----------------|   
 |1WS(2CPU cycle)|30 < HCLK <= 60 |24 < HCLK <= 48 |22 < HCLK <= 44  |20 < HCLK <= 40  | 
 |---------------|----------------|----------------|-----------------|-----------------|   
 |2WS(3CPU cycle)|60 < HCLK <= 84 |48 < HCLK <= 72 |44 < HCLK <= 66  |40 < HCLK <= 60  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |3WS(4CPU cycle)|      NA        |72 < HCLK <= 84 |66 < HCLK <= 84  |60 < HCLK <= 80  |
 |---------------|----------------|----------------|-----------------|-----------------| 
 |4WS(5CPU cycle)|      NA        |      NA        |      NA         |80 < HCLK <= 84  | 
 +-------------------------------------------------------------------------------------+ 
 
 [..]
 +-------------------------------------------------------------------------------------------------------------------+
 |               | voltage range  | voltage range  | voltage range   | voltage range   | voltage range 2.7 V - 3.6 V |
 |               | 2.7 V - 3.6 V  | 2.4 V - 2.7 V  | 2.1 V - 2.4 V   | 1.8 V - 2.1 V   | with External Vpp = 9V      |
 |---------------|----------------|----------------|-----------------|-----------------|-----------------------------| 
 |Max Parallelism|      x32       |               x16                |       x8        |          x64                |              
 |---------------|----------------|----------------|-----------------|-----------------|-----------------------------|   
 |PSIZE[1:0]     |      10        |               01                 |       00        |           11                |
 +-------------------------------------------------------------------------------------------------------------------+  

      -@- On STM32F405xx/407xx and STM32F415xx/417xx devices: 
           (++) when VOS = '0', the maximum value of fHCLK = 144MHz. 
           (++) when VOS = '1', the maximum value of fHCLK = 168MHz. 
          [..] 
          On STM32F42xxx/43xxx devices:
           (++) when VOS[1:0] = '0x01', the maximum value of fHCLK is 120MHz.
           (++) when VOS[1:0] = '0x10', the maximum value of fHCLK is 144MHz.
           (++) when VOS[1:0] = '0x11', the maximum value of f  is 168MHz 
          [..]  
          On STM32F401x devices:
           (++) when VOS[1:0] = '0x01', the maximum value of fHCLK is 64MHz.
           (++) when VOS[1:0] = '0x10', the maximum value of fHCLK is 84MHz.
           You can use PWR_MainRegulatorModeConfig() function to control VOS bits.
                 
      (+) void FLASH_PrefetchBufferCmd(FunctionalState NewState)
      (+) void FLASH_InstructionCacheCmd(FunctionalState NewState)
      (+) void FLASH_DataCacheCmd(FunctionalState NewState)
      (+) void FLASH_InstructionCacheReset(void)
      (+) void FLASH_DataCacheReset(void)
      
    [..]   
      The unlock sequence is not needed for these functions.
 
@endverbatim
  * @{
  */
 
/**
  * @brief  Sets the code latency value.  
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *          This parameter can be one of the following values:
  *            @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *            @arg FLASH_Latency_1: FLASH One Latency cycle
  *            @arg FLASH_Latency_2: FLASH Two Latency cycles
  *            @arg FLASH_Latency_3: FLASH Three Latency cycles
  *            @arg FLASH_Latency_4: FLASH Four Latency cycles 
  *            @arg FLASH_Latency_5: FLASH Five Latency cycles 
  *            @arg FLASH_Latency_6: FLASH Six Latency cycles
  *            @arg FLASH_Latency_7: FLASH Seven Latency cycles 
  *            @arg FLASH_Latency_8: FLASH Eight Latency cycles
  *            @arg FLASH_Latency_9: FLASH Nine Latency cycles
  *            @arg FLASH_Latency_10: FLASH Teen Latency cycles 
  *            @arg FLASH_Latency_11: FLASH Eleven Latency cycles 
  *            @arg FLASH_Latency_12: FLASH Twelve Latency cycles
  *            @arg FLASH_Latency_13: FLASH Thirteen Latency cycles        
  *            @arg FLASH_Latency_14: FLASH Fourteen Latency cycles
  *            @arg FLASH_Latency_15: FLASH Fifteen Latency cycles 
  *              
  * @note For STM32F405xx/407xx, STM32F415xx/417xx and STM32F401xx devices this parameter    
  *       can be a value between FLASH_Latency_0 and FLASH_Latency_7.
  *         
  * @note For STM32F42xxx/43xxx devices this parameter can be a value between 
  *       FLASH_Latency_0 and FLASH_Latency_15. 
  *         
  * @retval None
  */
//////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

typedef enum
{
  FLASH_IDLE,
  FLASH_WAIT_FOR_CMD,
} FLASH_StateType;

/**
 * @}
 */

/**
 * @defgroup flash_private_defines
 * @{
 */

/**
 * @brief   AF7 PA9 - PA10 pin masks
 */
#define GPIO_AFRH_AFRH9                      ((uint32_t) 0x000000F0)
#define GPIO_AFRH_AFRH9_AF7                  ((uint32_t) 0x00000070)
#define GPIO_AFRH_AFRH10                     ((uint32_t) 0x00000F00)
#define GPIO_AFRH_AFRH10_AF7                 ((uint32_t) 0x00000700)

/**
 * @brief   Maximum USART reception buffer length
 */
#define MAX_BUFFER_LENGTH                     ((uint32_t) 128u)

/**
 * @}
 */

/**
 * @defgroup flash_private_macros
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup flash_private_constants
 * @{
 */

/**
 * @brief   USART1 messages to be transmitted
 */
static char hello[] = "Welcome to Flash Controller !";
static char select_cmd[] = "Select command from the following: ";
static char lock[] = "1: Flash Lock";
static char unlock[] = "2: Flash Unlock";
static char write_data[] = "3: Write Data";
static char read_data[] = "4: Read Data";
static char erase[] = "5: Sector Erase";
static char write_protect[] = "6: Write Protection";
static char remove_write_protect[] = "7: Remove Write Protection";
static char ob_unlock[] = "8: Option Bytes Unlock";
static char ob_lock[] = "9: Option Bytes Lock";
static char line[] = "**********************************************";

static char error_PGSERR[] = "Programming sequence error";
static char error_PGPERR[] = "Programming parallelism error";
static char error_PGAERR[] = "Programming alignment error";
static char error_WRPERR[] = "Write protection error";

static char unlocked[] = "Flash unlock sequence executed ";
static char locked[] = "Flash lock sequence executed ";
static char erased[] = "Sector erase sequence executed";
static char ob_unlocked[] = "Option Bytes unlock sequence executed ";
static char ob_locked[] = "Option Bytes lock sequence executed ";
static char protected[] = "Write protection sequence executed ";
static char not_protected[] = "Remove write sequence executed ";
static char programmed[] = "Flash programming sequence executed ";

/**
 * @}
 */

/**
 * @defgroup flash_private_variables
 * @{
 */

/**
 * @brief   Flash current state
 */
static FLASH_StateType currentState = FLASH_IDLE;

/**
 * @brief   USART1 RX message buffer
 */
static char RxBuffer[MAX_BUFFER_LENGTH];
static char RxDMABuffer[MAX_BUFFER_LENGTH];

/**
 * @brief   USART1 RX message length
 */
static uint8_t RxMessageLength = 0;

/**
 * @}
 */

/**
 * @defgroup flash_private_function_prototypes
 * @{
 */
static void strTransmit(const char * str, uint8_t size);
static void process_command();
static void check_errors();

/**
 * @}
 */

/**
 * @defgroup flash_private_functions
 * @{
 */

/**
 * @brief   Check flash errors
 * @param   none
 * @retval  none
 */
static void check_errors()
{
  if(FLASH_SR_PGSERR == (FLASH->SR & FLASH_SR_PGSERR))
  {
    /* Programming sequence error */
    strTransmit(error_PGSERR, sizeof(error_PGSERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGSERR;
  }
  else
  {
    /* Do nothing */
  }

  if(FLASH_SR_PGPERR == (FLASH->SR & FLASH_SR_PGPERR))
  {
    /* Programming parallelism error */
    strTransmit(error_PGPERR, sizeof(error_PGPERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGPERR;
  }
  else
  {
    /* Do nothing */
  }

  if(FLASH_SR_PGAERR == (FLASH->SR & FLASH_SR_PGAERR))
  {
    /* Programming alignment error */
    strTransmit(error_PGAERR, sizeof(error_PGAERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_PGAERR;
  }
  else
  {
    /* Do nothing */
  }

  if(FLASH_SR_WRPERR == (FLASH->SR & FLASH_SR_WRPERR))
  {
    /* Write protection error */
    strTransmit(error_WRPERR, sizeof(error_WRPERR));
    strTransmit(line, sizeof(line));

    /* Clear */
    FLASH->SR |= FLASH_SR_WRPERR;
  }
  else
  {
    /* Do nothing */
  }
}

/**
 * @brief   Process flash command
 * @param   none
 * @retval  none
 */
static void process_command()
{
  uint32_t address = 0;
  uint32_t data = 0;
  uint32_t val = 0;

  /* Process received command */
  switch (RxBuffer[0])
  {
    case 1: /* Flash Lock */
      /* Write lock bit */
      FLASH->CR |= FLASH_CR_LOCK;

      strTransmit(locked, sizeof(locked));
      strTransmit(line, sizeof(line));
      break;

    case 2: /* Flash Unlock */
      /* Write KEY1 */
      FLASH->KEYR = 0x45670123;

      /* Write KEY2 */
      FLASH->KEYR = 0xCDEF89AB;

      strTransmit(unlocked, sizeof(unlocked));
      strTransmit(line, sizeof(line));
      break;

    case 3: /* Write Data */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Enable flash programming */
      FLASH->CR |= FLASH_CR_PG;

      /* Write data into flash */
      address = * (uint32_t *) &RxBuffer[1];
      data = * (uint32_t *) &RxBuffer[5];

      *(volatile uint32_t*)(address) = data;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Disable flash programming */
      FLASH->CR &= ~FLASH_CR_PG;

      strTransmit(programmed, sizeof(programmed));
      strTransmit(line, sizeof(line));
      break;

    case 4: /* Read Data */
      /* 4 00 00 1E 08 */
      address = * (uint32_t *) &RxBuffer[1];
      val = * (uint32_t *) address;

      strTransmit((char *)&val, sizeof(val));
      break;

    case 5: /* Sector Erase */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Select sector erase */
      FLASH->CR |= FLASH_CR_SER;

      /* Check sector number for bank 2 */
      uint32_t sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num + 4;
      }
      else
      {

      }

      /* Write sector number */
      FLASH->CR |= (FLASH_CR_SNB & (sector_num << 3));

      /* Start sector erase */
      FLASH->CR |= FLASH_CR_STRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Disable sector erase */
      FLASH->CR &= ~FLASH_CR_SER;

      strTransmit(erased, sizeof(erased));
      strTransmit(line, sizeof(line));
      break;

    case 6: /* Write Protection */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Check sector number */
      sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num - 12;
        FLASH->OPTCR1 &= ~((1 << sector_num) << 16);
      }
      else
      {
        /* Bank 2 */
        FLASH->OPTCR &= ~((1 << sector_num) << 16);
      }

      /* Start sector write protection */
      FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      strTransmit(protected, sizeof(protected));
      strTransmit(line, sizeof(line));
      break;

    case 7: /* Remove Write Protection */
      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      /* Check sector number */
      sector_num = RxBuffer[1];
      if(12 <= sector_num)
      {
        /* Bank 2 */
        sector_num = sector_num - 12;
        FLASH->OPTCR1 |= ((1 << sector_num) << 16);
      }
      else
      {
        /* Bank 2 */
        FLASH->OPTCR |= ((1 << sector_num) << 16);
      }

      /* Start sector write protection */
      FLASH->OPTCR |= FLASH_OPTCR_OPTSTRT;

      /* Wait bsy flag */
      while(0 != (FLASH->SR & FLASH_SR_BSY))
      {
        /* Waiting */
      }

      strTransmit(not_protected, sizeof(not_protected));
      strTransmit(line, sizeof(line));
      break;

    case 8: /* Option Bytes Unlock */
      /* Write OPTKEY1 */
      FLASH->OPTKEYR = 0x08192A3B;

      /* Write OPTKEY2 */
      FLASH->OPTKEYR = 0x4C5D6E7F;

      strTransmit(ob_unlocked, sizeof(ob_unlocked));
      strTransmit(line, sizeof(line));
      break;

    case 9: /* Option Bytes Lock */
      /* Write lock bit */
      FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;

      strTransmit(ob_locked, sizeof(ob_locked));
      strTransmit(line, sizeof(line));
      break;

    default:
      break;
  }
}
/**
 * @brief   DMA string transmit
 * @note
 * @param   str, size
 * @retval  None
 */
static void strTransmit(const char * str, uint8_t size)
{
  /* Check null pointers */
  if(NULL != str)
  {
    /* Wait until DMA2 stream 7 is disabled */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
    {
      /* Do nothing, the enable flag shall reset
       * when DMA transfer complete */
    }

    /* Set memory address */
    DMA2_Stream7->M0AR = (uint32_t)str;

    /* Set number of data items */
    DMA2_Stream7->NDTR = size;

    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
        | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);

    /* Enable DMA2 stream 7 */
    DMA2_Stream7->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}

void FLASH_USART1_GPIO_Config(void)
{
  /* Enable port A clock */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

  /* Select alternate function mode */
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
  GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);

  /* Select output type push-pull */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;

  /* Select output speed medium */
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR9;
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_0;

  /* Select pull up */
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
  GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0);

  /* Select AF7 */
  GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH9 | GPIO_AFRH_AFRH10);
  GPIOA->AFR[1] |= 0x00000770/*(GPIO_AFRH_AFRH9_AF7 | GPIO_AFRH_AFRH10_AF7)*/;
}

void FLASH_USART1_TX_DMA_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2 stream 7 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
  {
    /* DMA 2 stream 7 is enabled, shall be disabled first */
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream7->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 7 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream7->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream7->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA2_Stream7->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction memory-to-peripheral */
  DMA2_Stream7->CR &= ~DMA_SxCR_DIR;
  DMA2_Stream7->CR |= DMA_SxCR_DIR_0;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream7->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream7->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream7->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream7->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream7->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream7->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
}

void FLASH_USART1_RX_DMA_Config(void)
{
  /* Enable DMA clock in RCC */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

  /* Make sure that the DMA2 stream 5 is disabled */
  if(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream5->CR))
  {
    /* DMA 2 stream 5 is enabled, shall be disabled first */
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;

    /* Wait until EN bit is cleared */
    while(DMA_SxCR_EN == (DMA_SxCR_EN & DMA2_Stream5->CR))
    {
      /* Do nothing until EN bit is cleared */
    }
  }
  else
  {
    /* Do nothing, stream 5 is not enabled */
  }

  /* Select the DMA channel 4 in CHSEL[2:0] in the DMA_SxCR */
  DMA2_Stream5->CR &= ~DMA_SxCR_CHSEL;
  DMA2_Stream5->CR |= DMA_SxCR_CHSEL_2;

  /* Select stream priority very high */
  DMA2_Stream5->CR |= DMA_SxCR_PL;

  /* Select the data transfer direction peripheral-to-memory */
  DMA2_Stream5->CR &= ~DMA_SxCR_DIR;

  /* Select memory and peripherals sizes byte (8-bit) */
  DMA2_Stream5->CR &= ~DMA_SxCR_MSIZE;
  DMA2_Stream5->CR &= ~DMA_SxCR_PSIZE;

  /* Select FIFO mode */
  DMA2_Stream5->FCR |= DMA_SxFCR_DMDIS;

  /* Select FIFO threshold level half FIFO */
  DMA2_Stream5->FCR |= (DMA_SxFCR_FTH_0 | DMA_SxFCR_FTH_1);

  /* Select memory incremented mode, peripheral shall has fixed address */
  DMA2_Stream5->CR |= DMA_SxCR_MINC;

  /* Enable DMA transfer complete interrupt */
  DMA2_Stream5->CR |= DMA_SxCR_TCIE;

  /* Set peripheral address */
  DMA2_Stream5->PAR = (uint32_t)&USART1->DR;

  /* Set memory address */
  DMA2_Stream5->M0AR = (uint32_t)RxDMABuffer;

  /* Set number of data items */
  DMA2_Stream5->NDTR = MAX_BUFFER_LENGTH;
}

/**
 * @brief   Configure USART1 for ST virtual COM port (VCP)
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_USART1_Init(void)
{
  /* Configure USART1 */
  /* Enable USART1 clock */
  RCC->APB2ENR = RCC_APB2ENR_USART1EN;

  /* Set baud rate = 115200 Bps
   * USARTDIV = Fck / (16 * baud_rate)
   *          = 90000000 / (16 * 115200) = 48.82
   *
   * DIV_Fraction = 16 * 0.82 = 13.12 = 13 = 0xD
   * DIV_Mantissa = 48 = 0x30
   *
   * BRR          = 0x30D */
  USART1->BRR = 0x30D;
}

/**
 * @brief   Enable USART1 transmitter and receiver
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_USART1_Enable(void)
{
  /* Enable USART1 */
  USART1->CR1 |= USART_CR1_UE;

  /* Enable transmitter */
  USART1->CR1 |= USART_CR1_TE;

  /* Enable receiver */
  USART1->CR1 |= USART_CR1_RE;

  /* Enable idle line detection interrupt */
  USART1->CR1 |= USART_CR1_IDLEIE;

  /* Enable DMA mode for transmitter and receiver */
  USART1->CR3 |= USART_CR3_DMAT;
  USART1->CR3 |= USART_CR3_DMAR;

  /* Clear all DMA interrupt flags for RX */
  DMA2->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5
      | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);

  /* Enable DMA2 stream 5 for RX */
  DMA2_Stream5->CR |= DMA_SxCR_EN;
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_USART1_IRQ_Callback(void)
{
  /* Check if idle line detected */
  if((USART1->SR & USART_SR_IDLE) == USART_SR_IDLE)
  {
    /* Read data register to clear idle line flag */
    (void)USART1->DR;

    /* Disable DMA stream for RX */
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
  }
  else
  {
    /* No new data received */
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_USART1_TX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF7 == (DMA_HISR_TCIF7 & DMA2->HISR))
  {
    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7
        | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7);
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   IRQ callback function
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_USART1_RX_DMA_IRQ_Callback(void)
{
  /* Check transfer complete flag */
  if(DMA_HISR_TCIF5 == (DMA_HISR_TCIF5 & DMA2->HISR))
  {
    /* Calculate amount of data received */
    RxMessageLength = MAX_BUFFER_LENGTH - DMA2_Stream5->NDTR;

    /* Copy data into RX buffer */
    for(int idx = 0; idx < RxMessageLength; idx++)
    {
      RxBuffer[idx] = RxDMABuffer[idx];
    }

    /* Clear all interrupt flags */
    DMA2->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CTEIF5
        | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5);

    /* Enable DMA 2 stream 5 */
    DMA2_Stream5->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Do nothing, this interrupt is not handled */
  }
}

/**
 * @brief   Flash Initialization function
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_Init(void)
{
  /* Write KEY1 */
  FLASH->KEYR = 0x45670123;

  /* Write KEY2 */
  FLASH->KEYR = 0xCDEF89AB;

  /* Select flash parallelism x32 */
  FLASH->CR &= ~FLASH_CR_PSIZE;
  FLASH->CR |= FLASH_CR_PSIZE_1;

  /* Write lock bit */
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
 * @brief   USART1 transmit and receive data
 * @note
 * @param   None
 * @retval  None
 */
void FLASH_Main(void)
{
  /* Check current USART state */
  switch (currentState)
  {
    case FLASH_IDLE:
      /* Transmit data */
      strTransmit(line, sizeof(line));
      strTransmit(hello, sizeof(hello));
      strTransmit(select_cmd, sizeof(select_cmd));
      strTransmit(line, sizeof(line));
      strTransmit(lock, sizeof(lock));
      strTransmit(unlock, sizeof(unlock));
      strTransmit(write_data, sizeof(write_data));
      strTransmit(read_data, sizeof(read_data));
      strTransmit(erase, sizeof(erase));
      strTransmit(write_protect, sizeof(write_protect));
      strTransmit(remove_write_protect, sizeof(remove_write_protect));
      strTransmit(ob_unlock, sizeof(ob_unlock));
      strTransmit(ob_lock, sizeof(ob_lock));
      strTransmit(line, sizeof(line));

      /* Go to next state */
      currentState = FLASH_WAIT_FOR_CMD;
      break;

    case FLASH_WAIT_FOR_CMD:
      /* Check if new message received */
      if(0 != RxMessageLength)
      {
        /* Process command */
        process_command();

        /* Check errors */
        check_errors();

        /* Reset message length */
        RxMessageLength = 0;
      }
      else
      {
        /* Nothing received yet */
      }

      /* Go to next state */
      currentState = FLASH_WAIT_FOR_CMD;
      break;

    default:
      break;
  }
}

/**
 * @}
 */
/**
 * @}
 */
/**
 * @}
 */
/////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
void FLASH_SetLatency(uint32_t FLASH_Latency)
{
  /* Check the parameters */
  assert_param(IS_FLASH_LATENCY(FLASH_Latency));
  
  /* Perform Byte access to FLASH_ACR[8:0] to set the Latency value */
  *(__IO uint8_t *)ACR_BYTE0_ADDRESS = (uint8_t)FLASH_Latency;
}

/**
  * @brief  Enables or disables the Prefetch Buffer.
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
void FLASH_PrefetchBufferCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  /* Enable or disable the Prefetch Buffer */
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_PRFTEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_PRFTEN);
  }
}

/**
  * @brief  Enables or disables the Instruction Cache feature.
  * @param  NewState: new state of the Instruction Cache.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
void FLASH_InstructionCacheCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_ICEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_ICEN);
  }
}

/**
  * @brief  Enables or disables the Data Cache feature.
  * @param  NewState: new state of the Data Cache.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
void FLASH_DataCacheCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_DCEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_DCEN);
  }
}

/**
  * @brief  Resets the Instruction Cache.
  * @note   This function must be used only when the Instruction Cache is disabled.  
  * @param  None
  * @retval None
  */
void FLASH_InstructionCacheReset(void)
{
  FLASH->ACR |= FLASH_ACR_ICRST;
}

/**
  * @brief  Resets the Data Cache.
  * @note   This function must be used only when the Data Cache is disabled.  
  * @param  None
  * @retval None
  */
void FLASH_DataCacheReset(void)
{
  FLASH->ACR |= FLASH_ACR_DCRST;
}

/**
  * @}
  */

/** @defgroup FLASH_Group2 FLASH Memory Programming functions
 *  @brief   FLASH Memory Programming functions
 *
@verbatim   
 ===============================================================================
                ##### FLASH Memory Programming functions #####
 ===============================================================================   
    [..]
      This group includes the following functions:
      (+) void FLASH_Unlock(void)
      (+) void FLASH_Lock(void)
      (+) FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange)
      (+) FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange)       
      (+) FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data)
      (+) FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
      (+) FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
      (+) FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data)
          The following functions can be used only for STM32F42xxx/43xxx devices. 
      (+) FLASH_Status FLASH_EraseAllBank1Sectors(uint8_t VoltageRange)
      (+) FLASH_Status FLASH_EraseAllBank2Sectors(uint8_t VoltageRange)    
    [..]   
      Any operation of erase or program should follow these steps:
      (#) Call the FLASH_Unlock() function to enable the FLASH control register access

      (#) Call the desired function to erase sector(s) or program data

      (#) Call the FLASH_Lock() function to disable the FLASH control register access
          (recommended to protect the FLASH memory against possible unwanted operation)
    
@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }  
}

/**
  * @brief  Locks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CR |= FLASH_CR_LOCK;
}

/**
  * @brief  Erases a specified FLASH Sector.
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  *   
  * @param  FLASH_Sector: The Sector number to be erased.
  *
  *  @note  For STM32F405xx/407xx and STM32F415xx/417xx devices this parameter can 
  *         be a value between FLASH_Sector_0 and FLASH_Sector_11.
  *           
  *         For STM32F42xxx/43xxx devices this parameter can be a value between 
  *         FLASH_Sector_0 and FLASH_Sector_23.
  *           
  *         For STM32F401xx devices this parameter can be a value between 
  *         FLASH_Sector_0 and FLASH_Sector_5.             
  *    
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  *       
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_SECTOR(FLASH_Sector));
  assert_param(IS_VOLTAGERANGE(VoltageRange));
  
  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  { 
    /* if the previous operation is completed, proceed to erase the sector */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR &= SECTOR_MASK;
    FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();
    
    /* if the erase operation is completed, disable the SER Bit */
    FLASH->CR &= (~FLASH_CR_SER);
    FLASH->CR &= SECTOR_MASK; 
  }
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Erases all FLASH Sectors.
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  *       
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseAllSectors(uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  assert_param(IS_VOLTAGERANGE(VoltageRange));
  
  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all sectors */
#if defined (STM32F42_43xxx)    
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR |= (FLASH_CR_MER1 | FLASH_CR_MER2);
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the erase operation is completed, disable the MER Bit */
    FLASH->CR &= ~(FLASH_CR_MER1 | FLASH_CR_MER2);
#endif /* STM32F42_43xxx */

#if defined (STM32F40_41xxx) || defined (STM32F401xx) 
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the erase operation is completed, disable the MER Bit */
    FLASH->CR &= (~FLASH_CR_MER);
#endif /* STM32F40_41xxx || STM32F401xx */

  }   
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Erases all FLASH Sectors in Bank 1.
  *
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *      
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one. 
  *  
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  *       
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseAllBank1Sectors(uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  assert_param(IS_VOLTAGERANGE(VoltageRange));
  
  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all sectors */
     FLASH->CR &= CR_PSIZE_MASK;
     FLASH->CR |= tmp_psize;
     FLASH->CR |= FLASH_CR_MER1;
     FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the erase operation is completed, disable the MER Bit */
    FLASH->CR &= (~FLASH_CR_MER1);

  }   
  /* Return the Erase Status */
  return status;
}


/**
  * @brief  Erases all FLASH Sectors in Bank 2.
  *
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *     
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *     
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  *       
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseAllBank2Sectors(uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  assert_param(IS_VOLTAGERANGE(VoltageRange));
  
  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all sectors */
     FLASH->CR &= CR_PSIZE_MASK;
     FLASH->CR |= tmp_psize;
     FLASH->CR |= FLASH_CR_MER2;
     FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the erase operation is completed, disable the MER Bit */
    FLASH->CR &= (~FLASH_CR_MER2);

  }   
  /* Return the Erase Status */
  return status;
}

/**
  * @brief  Programs a double word (64-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V and an External Vpp is present.
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_ProgramDoubleWord(uint32_t Address, uint64_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint64_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a word (32-bit) at a specified address.
  *
  * @note   This function must be used when the device voltage range is from 2.7V to 3.6V. 
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  *         This parameter can be any address in Program memory zone or in OTP zone.  
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint32_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a half word (16-bit) at a specified address. 
  * @note   This function must be used when the device voltage range is from 2.1V to 3.6V. 
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  * 
  * @param  Address: specifies the address to be programmed.
  *         This parameter can be any address in Program memory zone or in OTP zone.  
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_HALF_WORD;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint16_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 
  /* Return the Program Status */
  return status;
}

/**
  * @brief  Programs a byte (8-bit) at a specified address.
  * @note   This function can be used within all the device supply voltage ranges.  
  *
  * @note   If an erase and a program operations are requested simustaneously,    
  *         the erase operation is performed before the program one.
  * 
  * @param  Address: specifies the address to be programmed.
  *         This parameter can be any address in Program memory zone or in OTP zone.  
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_BYTE;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint8_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 

  /* Return the Program Status */
  return status;
}

/**
  * @}
  */

/** @defgroup FLASH_Group3 Option Bytes Programming functions
 *  @brief   Option Bytes Programming functions 
 *
@verbatim   
 ===============================================================================
                ##### Option Bytes Programming functions #####
 ===============================================================================  
    [..]
      This group includes the following functions:
      (+) void FLASH_OB_Unlock(void)
      (+) void FLASH_OB_Lock(void)
      (+) void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState)
      (+) void FLASH_OB_WRP1Config(uint32_t OB_WRP, FunctionalState NewState)  
      (+) void FLASH_OB_PCROPSelectionConfig(uint8_t OB_PCROPSelect)
      (+) void FLASH_OB_PCROPConfig(uint32_t OB_PCROP, FunctionalState NewState)
      (+) void FLASH_OB_PCROP1Config(uint32_t OB_PCROP, FunctionalState NewState) 
      (+) void FLASH_OB_RDPConfig(uint8_t OB_RDP)
      (+) void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY)
      (+) void FLASH_OB_BORConfig(uint8_t OB_BOR)
      (+) FLASH_Status FLASH_ProgramOTP(uint32_t Address, uint32_t Data)							
      (+) FLASH_Status FLASH_OB_Launch(void)
      (+) uint32_t FLASH_OB_GetUser(void)						
      (+) uint8_t FLASH_OB_GetWRP(void)
      (+) uint8_t FLASH_OB_GetWRP1(void)
      (+) uint8_t FLASH_OB_GetPCROP(void)
      (+) uint8_t FLASH_OB_GetPCROP1(void)    						
      (+) uint8_t FLASH_OB_GetRDP(void)							
      (+) uint8_t FLASH_OB_GetBOR(void)
    [..]  
      The following function can be used only for STM32F42xxx/43xxx devices. 
      (+) void FLASH_OB_BootConfig(uint8_t OB_BOOT)
    [..]   
     Any operation of erase or program should follow these steps:
      (#) Call the FLASH_OB_Unlock() function to enable the FLASH option control 
          register access

      (#) Call one or several functions to program the desired Option Bytes:
        (++) void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState) 
             => to Enable/Disable the desired sector write protection
        (++) void FLASH_OB_RDPConfig(uint8_t OB_RDP) => to set the desired read 
             Protection Level
        (++) void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY) 
             => to configure the user Option Bytes.
        (++) void FLASH_OB_BORConfig(uint8_t OB_BOR) => to set the BOR Level 			 

      (#) Once all needed Option Bytes to be programmed are correctly written, 
          call the FLASH_OB_Launch() function to launch the Option Bytes 
          programming process.
     
      -@- When changing the IWDG mode from HW to SW or from SW to HW, a system 
          reset is needed to make the change effective.  

      (#) Call the FLASH_OB_Lock() function to disable the FLASH option control 
          register access (recommended to protect the Option Bytes against 
          possible unwanted operations)
    
@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the FLASH Option Control Registers access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Unlock(void)
{
  if((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != RESET)
  {
    /* Authorizes the Option Byte register programming */
    FLASH->OPTKEYR = FLASH_OPT_KEY1;
    FLASH->OPTKEYR = FLASH_OPT_KEY2;
  }  
}

/**
  * @brief  Locks the FLASH Option Control Registers access.
  * @param  None
  * @retval None
  */
void FLASH_OB_Lock(void)
{
  /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
  FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
}

/**
  * @brief  Enables or disables the write protection of the desired sectors, for the first
  *         1 Mb of the Flash  
  *
  * @note   When the memory read protection level is selected (RDP level = 1), 
  *         it is not possible to program or erase the flash sector i if CortexM4  
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1 
  * @note   Active value of nWRPi bits is inverted when PCROP mode is active (SPRMOD =1).   
  * 
  * @param  OB_WRP: specifies the sector(s) to be write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_WRP: A value between OB_WRP_Sector0 and OB_WRP_Sector11                      
  *            @arg OB_WRP_Sector_All
  * @param  Newstate: new state of the Write Protection.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None  
  */
void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState)
{ 
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_OB_WRP(OB_WRP));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  status = FLASH_WaitForLastOperation();

  if(status == FLASH_COMPLETE)
  { 
    if(NewState != DISABLE)
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS &= (~OB_WRP);
    }
    else
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS |= (uint16_t)OB_WRP;
    }
  }
}

/**
  * @brief  Enables or disables the write protection of the desired sectors, for the second
  *         1 Mb of the Flash  
  *           
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *   
  * @note   When the memory read out protection is selected (RDP level = 1), 
  *         it is not possible to program or erase the flash sector i if CortexM4  
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1 
  * @note   Active value of nWRPi bits is inverted when PCROP mode is active (SPRMOD =1).      
  * 
  * @param  OB_WRP: specifies the sector(s) to be write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_WRP: A value between OB_WRP_Sector12 and OB_WRP_Sector23
  *            @arg OB_WRP_Sector_All                        
  * @param  Newstate: new state of the Write Protection.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None  
  */
void FLASH_OB_WRP1Config(uint32_t OB_WRP, FunctionalState NewState)
{ 
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_OB_WRP(OB_WRP));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  status = FLASH_WaitForLastOperation();

  if(status == FLASH_COMPLETE)
  { 
    if(NewState != DISABLE)
    {
      *(__IO uint16_t*)OPTCR1_BYTE2_ADDRESS &= (~OB_WRP);
    }
    else
    {
      *(__IO uint16_t*)OPTCR1_BYTE2_ADDRESS |= (uint16_t)OB_WRP;
    }
  }
}

/**
  * @brief  Select the Protection Mode (SPRMOD). 
  * 
  * @note   This function can be used only for STM32F42xxx/43xxx and STM32F401xx devices.       
  * 
  * @note   After PCROP activation, Option Byte modification is not possible. 
  *         Exception made for the global Read Out Protection modification level (level1 to level0) 
  * @note   Once SPRMOD bit is active unprotection of a protected sector is not possible 
  *   
  * @note   Read a protected sector will set RDERR Flag and write a protected sector will set WRPERR Flag
  *   
  * @note   Some Precautions should be taken when activating the PCROP feature :
  *         The active value of nWRPi bits is inverted when PCROP mode is active, this means if SPRMOD = 1
  *         and WRPi = 1 (default value), then the user sector i is read/write protected.
  *         In order to avoid activation of PCROP Mode for undesired sectors, please follow the
  *         below safety sequence :       
  *         - Disable PCROP for all Sectors using FLASH_OB_PCROPConfig(OB_PCROP_Sector_All, DISABLE) function 
  *           for Bank1 or FLASH_OB_PCROP1Config(OB_PCROP_Sector_All, DISABLE) function for Bank2   
  *         - Enable PCROP for the desired Sector i using FLASH_OB_PCROPConfig(Sector i, ENABLE) function
  *         - Activate the PCROP Mode FLASH_OB_PCROPSelectionConfig() function. 
  * 
  * @param  OB_PCROP:  Select the Protection Mode of nWPRi bits 
  *          This parameter can be one of the following values:
  *            @arg OB_PcROP_Disable: nWRPi control the write protection of respective user sectors.
  *            @arg OB_PcROP_Enable: nWRPi control the  read&write protection (PCROP) of respective user sectors.
  * @retval None
  */
void FLASH_OB_PCROPSelectionConfig(uint8_t OB_PcROP)
{  
  uint8_t optiontmp = 0xFF;
      
  /* Check the parameters */
  assert_param(IS_OB_PCROP_SELECT(OB_PcROP));
  
  /* Mask SPRMOD bit */
  optiontmp =  (uint8_t)((*(__IO uint8_t *)OPTCR_BYTE3_ADDRESS) & (uint8_t)0x7F); 
  /* Update Option Byte */
  *(__IO uint8_t *)OPTCR_BYTE3_ADDRESS = (uint8_t)(OB_PcROP | optiontmp); 
    
}

/**
  * @brief  Enables or disables the read/write protection (PCROP) of the desired 
  *         sectors, for the first 1 MB of the Flash.
  *           
  * @note   This function can be used only for STM32F42xxx/43xxx and STM32F401xx devices. 
  *   
  * @param  OB_PCROP: specifies the sector(s) to be read/write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_PCROP: A value between OB_PCROP_Sector0 and OB_PCROP_Sector11 for 
  *                           STM32F42xxx/43xxx devices and between OB_PCROP_Sector0 and 
  *                           OB_PCROP_Sector5 for STM32F401xx devices.
  *            @arg OB_PCROP_Sector_All                         
  * @param  Newstate: new state of the Write Protection.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None  
  */
void FLASH_OB_PCROPConfig(uint32_t OB_PCROP, FunctionalState NewState)
{ 
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_OB_PCROP(OB_PCROP));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  status = FLASH_WaitForLastOperation();

  if(status == FLASH_COMPLETE)
  { 
    if(NewState != DISABLE)
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS |= (uint16_t)OB_PCROP;    
    }
    else
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS &= (~OB_PCROP);
    }
  }
}

/**
   * @brief Enables or disables the read/write protection (PCROP) of the desired 
  *         sectors
  *           
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *   
  * @param  OB_PCROP: specifies the sector(s) to be read/write protected or unprotected.
  *          This parameter can be one of the following values:
  *            @arg OB_PCROP: A value between OB_PCROP_Sector12 and OB_PCROP_Sector23 
  *            @arg OB_PCROP_Sector_All                    
  * @param  Newstate: new state of the Write Protection.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None  
  */
void FLASH_OB_PCROP1Config(uint32_t OB_PCROP, FunctionalState NewState)
{ 
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_OB_PCROP(OB_PCROP));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  status = FLASH_WaitForLastOperation();

  if(status == FLASH_COMPLETE)
  { 
    if(NewState != DISABLE)
    {
      *(__IO uint16_t*)OPTCR1_BYTE2_ADDRESS |= (uint16_t)OB_PCROP;
    }
    else
    {
      *(__IO uint16_t*)OPTCR1_BYTE2_ADDRESS &= (~OB_PCROP);
    }
  }
}


/**
  * @brief  Sets the read protection level.
  * @param  OB_RDP: specifies the read protection level.
  *          This parameter can be one of the following values:
  *            @arg OB_RDP_Level_0: No protection
  *            @arg OB_RDP_Level_1: Read protection of the memory
  *            @arg OB_RDP_Level_2: Full chip protection
  *   
  * /!\ Warning /!\ When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  *    
  * @retval None
  */
void FLASH_OB_RDPConfig(uint8_t OB_RDP)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_OB_RDP(OB_RDP));

  status = FLASH_WaitForLastOperation();

  if(status == FLASH_COMPLETE)
  {
    *(__IO uint8_t*)OPTCR_BYTE1_ADDRESS = OB_RDP;

  }
}

/**
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.    
  * @param  OB_IWDG: Selects the IWDG mode
  *          This parameter can be one of the following values:
  *            @arg OB_IWDG_SW: Software IWDG selected
  *            @arg OB_IWDG_HW: Hardware IWDG selected
  * @param  OB_STOP: Reset event when entering STOP mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STOP_NoRST: No reset generated when entering in STOP
  *            @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  OB_STDBY: Reset event when entering Standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STDBY_NoRST: No reset generated when entering in STANDBY
  *            @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @retval None
  */
void FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY)
{
  uint8_t optiontmp = 0xFF;
  FLASH_Status status = FLASH_COMPLETE; 

  /* Check the parameters */
  assert_param(IS_OB_IWDG_SOURCE(OB_IWDG));
  assert_param(IS_OB_STOP_SOURCE(OB_STOP));
  assert_param(IS_OB_STDBY_SOURCE(OB_STDBY));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  { 
#if defined (STM32F42_43xxx)     
    /* Mask OPTLOCK, OPTSTRT, BOR_LEV and BFB2 bits */
    optiontmp =  (uint8_t)((*(__IO uint8_t *)OPTCR_BYTE0_ADDRESS) & (uint8_t)0x1F);
#endif /* STM32F42_43xxx */

#if defined (STM32F40_41xxx) || defined (STM32F401xx) 
    /* Mask OPTLOCK, OPTSTRT and BOR_LEV bits */
    optiontmp =  (uint8_t)((*(__IO uint8_t *)OPTCR_BYTE0_ADDRESS) & (uint8_t)0x0F); 
#endif /* STM32F40_41xxx || STM32F401xx */ 

    /* Update User Option Byte */
    *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS = OB_IWDG | (uint8_t)(OB_STDBY | (uint8_t)(OB_STOP | ((uint8_t)optiontmp))); 
  }  
}

/**
  * @brief  Configure the Dual Bank Boot.
  *   
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *      
  * @param  OB_BOOT: specifies the Dual Bank Boot Option byte.
  *          This parameter can be one of the following values:
  *            @arg OB_Dual_BootEnabled: Dual Bank Boot Enable
  *            @arg OB_Dual_BootDisabled: Dual Bank Boot Disabled
  * @retval None
  */
void FLASH_OB_BootConfig(uint8_t OB_BOOT)
{
  /* Check the parameters */
  assert_param(IS_OB_BOOT(OB_BOOT));

  /* Set Dual Bank Boot */
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS &= (~FLASH_OPTCR_BFB2);
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS |= OB_BOOT;

}

/**
  * @brief  Sets the BOR Level. 
  * @param  OB_BOR: specifies the Option Bytes BOR Reset Level.
  *          This parameter can be one of the following values:
  *            @arg OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *            @arg OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *            @arg OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *            @arg OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.1 V
  * @retval None
  */
void FLASH_OB_BORConfig(uint8_t OB_BOR)
{
  /* Check the parameters */
  assert_param(IS_OB_BOR(OB_BOR));

  /* Set the BOR Level */
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS &= (~FLASH_OPTCR_BOR_LEV);
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS |= OB_BOR;

}

/**
  * @brief  Launch the option byte loading.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_OB_Launch(void)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Set the OPTSTRT bit in OPTCR register */
  *(__IO uint8_t *)OPTCR_BYTE0_ADDRESS |= FLASH_OPTCR_OPTSTRT;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();

  return status;
}

/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @param  None
  * @retval The FLASH User Option Bytes values: IWDG_SW(Bit0), RST_STOP(Bit1)
  *         and RST_STDBY(Bit2).
  */
uint8_t FLASH_OB_GetUser(void)
{
  /* Return the User Option Byte */
  return (uint8_t)(FLASH->OPTCR >> 5);
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  * @param  None
  * @retval The FLASH Write Protection  Option Bytes value
  */
uint16_t FLASH_OB_GetWRP(void)
{
  /* Return the FLASH write protection Register value */
  return (*(__IO uint16_t *)(OPTCR_BYTE2_ADDRESS));
}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  *   
  * @note   This function can be used only for STM32F42xxx/43xxx devices.
  *   
  * @param  None
  * @retval The FLASH Write Protection  Option Bytes value
  */
uint16_t FLASH_OB_GetWRP1(void)
{
  /* Return the FLASH write protection Register value */
  return (*(__IO uint16_t *)(OPTCR1_BYTE2_ADDRESS));
}

/**
  * @brief  Returns the FLASH PC Read/Write Protection Option Bytes value.
  *   
  * @note   This function can be used only for STM32F42xxx/43xxx devices and STM32F401xx devices.
  *   
  * @param  None
  * @retval The FLASH PC Read/Write Protection Option Bytes value
  */
uint16_t FLASH_OB_GetPCROP(void)
{
  /* Return the FLASH PC Read/write protection Register value */
  return (*(__IO uint16_t *)(OPTCR_BYTE2_ADDRESS));
}

/**
  * @brief  Returns the FLASH PC Read/Write Protection Option Bytes value.
  *   
  * @note   This function can be used only for STM32F42xxx/43xxx devices. 
  *     
  * @param  None
  * @retval The FLASH PC Read/Write Protection Option Bytes value
  */
uint16_t FLASH_OB_GetPCROP1(void)
{
  /* Return the FLASH write protection Register value */
  return (*(__IO uint16_t *)(OPTCR1_BYTE2_ADDRESS));
}

/**
  * @brief  Returns the FLASH Read Protection level.
  * @param  None
  * @retval FLASH ReadOut Protection Status:
  *           - SET, when OB_RDP_Level_1 or OB_RDP_Level_2 is set
  *           - RESET, when OB_RDP_Level_0 is set
  */
FlagStatus FLASH_OB_GetRDP(void)
{
  FlagStatus readstatus = RESET;

  if ((*(__IO uint8_t*)(OPTCR_BYTE1_ADDRESS) != (uint8_t)OB_RDP_Level_0))
  {
    readstatus = SET;
  }
  else
  {
    readstatus = RESET;
  }
  return readstatus;
}

/**
  * @brief  Returns the FLASH BOR level.
  * @param  None
  * @retval The FLASH BOR level:
  *           - OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *           - OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *           - OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *           - OB_BOR_OFF   : Supply voltage ranges from 1.62 to 2.1 V  
  */
uint8_t FLASH_OB_GetBOR(void)
{
  /* Return the FLASH BOR level */
  return (uint8_t)(*(__IO uint8_t *)(OPTCR_BYTE0_ADDRESS) & (uint8_t)0x0C);
}

/**
  * @}
  */

/** @defgroup FLASH_Group4 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
              ##### Interrupts and flags management functions #####
 ===============================================================================  
@endverbatim
  * @{
  */

/**
  * @brief  Enables or disables the specified FLASH interrupts.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_IT_ERR: FLASH Error Interrupt
  *            @arg FLASH_IT_EOP: FLASH end of operation Interrupt
  * @retval None 
  */
void FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FLASH_IT(FLASH_IT)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the interrupt sources */
    FLASH->CR |= FLASH_IT;
  }
  else
  {
    /* Disable the interrupt sources */
    FLASH->CR &= ~(uint32_t)FLASH_IT;
  }
}

/**
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg FLASH_FLAG_EOP: FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR: FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_PGSERR: FLASH Programming Sequence error flag
  *            @arg FLASH_FLAG_RDERR: FLASH (PCROP) Read Protection error flag (STM32F42/43xxx and STM32F401xx devices) 
  *            @arg FLASH_FLAG_BSY: FLASH Busy flag
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
FlagStatus FLASH_GetFlagStatus(uint32_t FLASH_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_FLASH_GET_FLAG(FLASH_FLAG));

  if((FLASH->SR & FLASH_FLAG) != (uint32_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus; 
}

/**
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP: FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR: FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag 
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_PGSERR: FLASH Programming Sequence error flag
  *            @arg FLASH_FLAG_RDERR: FLASH Read Protection error flag (STM32F42/43xxx and STM32F401xx devices)   
  * @retval None
  */
void FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG));
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_RD, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & FLASH_FLAG_WRPERR) != (uint32_t)0x00)
    { 
      flashstatus = FLASH_ERROR_WRP;
    }
    else
    {
      if((FLASH->SR & FLASH_FLAG_RDERR) != (uint32_t)0x00)
      { 
        flashstatus = FLASH_ERROR_RD;
      } 
      else 
      {
        if((FLASH->SR & (uint32_t)0xEF) != (uint32_t)0x00)
        {
          flashstatus = FLASH_ERROR_PROGRAM; 
        }
        else
        {
          if((FLASH->SR & FLASH_FLAG_OPERR) != (uint32_t)0x00)
          {
            flashstatus = FLASH_ERROR_OPERATION;
          }
          else
          {
            flashstatus = FLASH_COMPLETE;
          }
        }
      }
    }
  }
  /* Return the FLASH Status */
  return flashstatus;
}

/**
  * @brief  Waits for a FLASH operation to complete.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_WaitForLastOperation(void)
{ 
  __IO FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the FLASH Status */
  status = FLASH_GetStatus();

  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  while(status == FLASH_BUSY)
  {
    status = FLASH_GetStatus();
  }
  /* Return the operation status */
  return status;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
