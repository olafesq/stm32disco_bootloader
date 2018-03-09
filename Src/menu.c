/**
  ******************************************************************************
  * @file    IAP_Main/Src/menu.c 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides the software which contains the main menu routine.
  *          The main menu gives the options of:
  *             - downloading a new binary file, 
  *             - uploading internal flash memory,
  *             - executing the binary file already loaded 
  *             - configuring the write protection of the Flash sectors where the 
  *               user loads his binary file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/** @addtogroup STM32L0xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "common.h"

#include "menu.h"

#include "stm32f4xx_hal_flash_ex.h"

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);
/* Private define ------------------------------------------------------------*/
/* Application start address */
#define APPLICATION_ADDRESS        0x0800C000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//uint8_t aFileName[FILE_NAME_LENGTH];
extern UART_HandleTypeDef huart1;
FLASH_OBProgramInitTypeDef pOBInit;
uint32_t FlashProtection = 1;
/* Private function prototypes -----------------------------------------------*/
void SerialDownload(void);
void SerialUpload(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Upload a file to flash via serial port
  * @param  None
  * @retval None
  */
void SerialUpload(void){
  HAL_StatusTypeDef status;

  Serial_PutString("Erasing flash..\n");

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

  FLASH_EraseInitTypeDef pEraseInit;
  	  pEraseInit.Sector = FLASH_SECTOR_3;
  	  pEraseInit.NbSectors = 11-3;
  	  pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
  	  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  uint32_t SectorError = 0xFFFFFFFFU;

  status = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  if(status ==  HAL_OK){Serial_PutString("Successfully erased!\n");}
    else{Serial_PutString("Erase failed");}
  if(SectorError ==  0xFFFFFFFFU){Serial_PutString("Flash erased successfully.\n"); }
  	else {Serial_PutString("Error in erasing\r");}

  Serial_PutString("Starting to reprogram flash..\n");

  int finished = 0;
  //uint8_t offset = 0;
  uint8_t nack = 19;
  uint8_t ack = 17;
  uint32_t flashAddr = APPLICATION_ADDRESS;

  while(finished == 0){
	  uint8_t data;
	  //Serial_PutString("send something.\n");
	  HAL_UART_Receive(&huart1, &data, 1, RX_TIMEOUT);
	  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddr, data);

	  if(status ==  HAL_OK){
		  //Serial_PutString("Successfully programmed!\n");
		  flashAddr++;
		  HAL_UART_Transmit(&huart1, &ack, 1, TX_TIMEOUT);
		  HAL_Delay(10);
		  if (flashAddr-APPLICATION_ADDRESS==5448) finished =1;

	  }
	  else{
		  Serial_PutString("Programming failed\n");
		  finished = 1;
		  HAL_UART_Transmit(&huart1, &nack, 1, TX_TIMEOUT);

	  }

  }

  //uint8_t *result=(uint8_t *)APPLICATION_ADDRESS;
  //if(*result== data){Serial_PutString("Klapip\n");}

  HAL_FLASH_Lock();

}

/**
  * @brief  Download a file from flash via serial port.
  * @param  None
  * @retval None
  */
void SerialDownload(void)
{
  Serial_PutString("\n\n\rSelect Receive File\n\r");
  //uint8_t result = *(__IO uint32_t *)APPLICATION_ADDRESS;
  const volatile uint8_t *result=(const volatile uint8_t *)APPLICATION_ADDRESS;
  if(*result== 0xFD){Serial_PutString("Klapip");}
  else{Serial_PutString("Ei klapi");  }


}

void FLASH_Init(void){
  /* Unlock the Program memory */
  HAL_FLASH_Unlock();
  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_OPERR |
                         FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR );
  /* Unlock the Program memory */
  HAL_FLASH_Lock();
}
/**
  * @brief  Display the Main Menu on HyperTerminal
  * @param  None
  * @retval None
  */
void Main_Menu(void)
{
	FLASH_Init();

  uint8_t key = 0;


  Serial_PutString("\r\n======================================================================");
  Serial_PutString("\r\n=              (C) COPYRIGHT 2015 STMicroelectronics                 =");
  Serial_PutString("\r\n=                                                                    =");
  Serial_PutString("\r\n=  STM32L0xx In-Application Programming Application  (Version 1.0.0) =");
  Serial_PutString("\r\n=                                                                    =");
  Serial_PutString("\r\n=                                   By MCD Application Team          =");
  Serial_PutString("\r\n======================================================================");
  Serial_PutString("\r\n\r\n");


  while (1)
  {
	/* Test if any sector of Flash memory where user application will be loaded is write protected */
	HAL_FLASH_OB_Unlock();
	HAL_FLASHEx_OBGetConfig(&pOBInit);
	FlashProtection = pOBInit.WRPState;
	HAL_FLASH_OB_Lock();

    Serial_PutString("\r\n=================== Main Menu ============================\r\n\n");
    Serial_PutString("  Upload image to the internal Flash ----------------- 1\r\n\n");
    Serial_PutString("  Get image from the internal Flash ----------------- 2\r\n\n");
    Serial_PutString("  Execute the loaded application ----------------------- 3\r\n\n");

    if(FlashProtection != OB_WRPSTATE_DISABLE )
    {
      Serial_PutString("  Disable the write protection ------------------------- 4\r\n\n");
    }
    else
    {
      Serial_PutString("  Enable the write protection -------------------------- 4\r\n\n");
    }
    Serial_PutString("==========================================================\r\n\n");

    /* Clean the input path */
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
	
    /* Receive key */
    HAL_UART_Receive(&huart1, &key, 1, RX_TIMEOUT);

    switch (key)
    {
    case '2' :
      /* Download user application from the Flash */
      SerialDownload();
      break;
    case '1' :
      /* Upload user application into the Flash */
      SerialUpload();
      break;
    case '3' :
      Serial_PutString("Start program execution......\r\n\n");

      	pFunction appEntry;
      	uint32_t appStack;

        /* Get the application stack pointer (First entry in the application vector table) */
      	appStack = (uint32_t) *((__IO uint32_t*)APPLICATION_ADDRESS);

      	/* Get the application entry point (Second entry in the application vector table) */
      	appEntry = (pFunction) *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);

      	/* Reconfigure vector table offset register to match the application location */
      	SCB->VTOR = APPLICATION_ADDRESS;

      	/* Set the application stack pointer */
      	__set_MSP(appStack);

      	/* Start the application */
      	appEntry();

      break;
    case '4' :


      if (FlashProtection != OB_WRPSTATE_DISABLE)      {
    	HAL_FLASH_OB_Unlock();
        /* Disable the write protection */
    	pOBInit.WRPState=OB_WRPSTATE_DISABLE;

        if (HAL_FLASHEx_OBProgram(&pOBInit) == HAL_OK)
        {
          Serial_PutString("Write Protection disabled...\r\n");
          Serial_PutString("System will now restart...\r\n");
          /* Launch the option byte loading */
          HAL_FLASH_OB_Launch();
          HAL_FLASH_OB_Lock();
        }
        else        {
          Serial_PutString("Error: Flash write un-protection failed...\r\n");
        }
      }
      else      {
    	HAL_FLASH_OB_Unlock();
    	pOBInit.WRPState=OB_WRPSTATE_ENABLE;
        if (HAL_FLASHEx_OBProgram(&pOBInit) == HAL_OK)
        {
          Serial_PutString("Write Protection enabled...\r\n");
          Serial_PutString("System will now restart...\r\n");
          /* Launch the option byte loading */
          HAL_FLASH_OB_Launch();
          HAL_FLASH_OB_Lock();
        }
        else
        {

          Serial_PutString("Error: Flash write protection failed...\r\n");
        }
      }

      break;
	default:
	Serial_PutString("Invalid Number ! ==> The number should be either 1, 2, 3 or 4\r");
	break;
    }
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
