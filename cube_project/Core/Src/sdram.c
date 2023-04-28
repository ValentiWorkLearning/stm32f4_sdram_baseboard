/*
 * sdram_impl.c
 *
 *  Created on: 3 Dec 2022
 *      Author: Valenti
 */
#include <sdram.h>

//Taken from STM32F429 BSP package:
// https://github.com/STMicroelectronics/32f429idiscovery-bsp/blob/9a874826803b7eed24ffb7ca7f55b7f04808b962/stm32f429i_discovery_sdram.c#L80
//  /* FMC Configuration -------------------------------------------------------*/
//  /* FMC SDRAM Bank configuration */
//  /* Timing configuration for 84 Mhz of SD clock frequency (180Mhz/2) */
//  /* TMRD: 2 Clock cycles */
//  Timing.LoadToActiveDelay    = 2;
//  /* TXSR: min=70ns (7x11.11ns) */
//  Timing.ExitSelfRefreshDelay = 7;
//  /* TRAS: min=42ns (4x11.11ns) max=120k (ns) */
//  Timing.SelfRefreshTime      = 4;
//  /* TRC:  min=70 (7x11.11ns) */
//  Timing.RowCycleDelay        = 7;
//  /* TWR:  min=1+ 7ns (1+1x11.11ns) */
//  Timing.WriteRecoveryTime    = 2;
//  /* TRP:  20ns => 2x11.11ns*/
//  Timing.RPDelay              = 2;
//  /* TRCD: 20ns => 2x11.11ns */
//  Timing.RCDDelay             = 2;
//
void bsp_sdram_init(SDRAM_HandleTypeDef* pSdramHandle)
{
  __IO uint32_t tmpmrd =0;
  const uint32_t SDRAM_TIMEOUT = 0xFFFF;
  const uint32_t REFRESH_COUNT = 605;
  FMC_SDRAM_CommandTypeDef Command;

  /* Step 1:  Configure a clock configuration enable command */
  Command.CommandMode             = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(pSdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode             = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(pSdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode             = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 4;
  Command.ModeRegisterDefinition  = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(pSdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_2           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode             = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget           = FMC_SDRAM_CMD_TARGET_BANK2;
  Command.AutoRefreshNumber       = 1;
  Command.ModeRegisterDefinition  = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(pSdramHandle, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(pSdramHandle, REFRESH_COUNT);
}


static const uint32_t SDRAM_BASE_ADDR = 0xD0000000; // RM0090 Flexible memory controller (FMC) Figure 457. FMC memory banks
// https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf

static const uint32_t SDRAM_SIZE = 0x00800000; // https://www.issi.com/ww/pdf/42-45s16400j.pdf 1 Meg Bits x 16 Bits x 4 Banks (64-MBIT)

uint8_t bsp_sdram_perform_test(SDRAM_HandleTypeDef* pSdramHandle)
{
	const uint32_t kTestStep = sizeof(uint32_t);

	for(uint32_t addrIt = SDRAM_BASE_ADDR; addrIt<SDRAM_BASE_ADDR+SDRAM_SIZE ; addrIt+=kTestStep)
	{
		uint32_t kTestSequence = 0xDEAFBEEF;

		const uint32_t kTestBufferSize = 1;

		HAL_SDRAM_WriteProtection_Disable(pSdramHandle);
		HAL_SDRAM_Write_32b(pSdramHandle, (uint32_t*)(addrIt), &kTestSequence, kTestBufferSize);
		uint32_t readBack = 0;
		HAL_SDRAM_Read_32b(pSdramHandle, (uint32_t*)(addrIt), &readBack, kTestBufferSize);
		if(readBack != kTestSequence)
			return 1;
	}

	return 0;
}
