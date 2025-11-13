#include "OTA.h"

volatile bool OTA_flag = false;

static uint32_t GetPage(uint32_t Address)
{
	for (int indx = 0; indx < 128; indx++)
	{
		if ((Address < (0x08000000 + (FLASH_PAGE_SIZE * (indx + 1))))
				&& (Address >= (0x08000000 + FLASH_PAGE_SIZE * indx)))
		{
			return (0x08000000 + FLASH_PAGE_SIZE * indx);
		}
	}

	return 0;
}

uint32_t Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t WordSize)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area*/

	uint32_t StartPage = GetPage(StartAddress);
	uint32_t EndPageAdress = StartAddress + WordSize * 4;
	uint32_t EndPage = GetPage(EndPageAdress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartPage;
	EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError();
	}

	/* Program the user Flash area word by word*/

	while (sofar < WordSize)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartAddress, Data[sofar])
				== HAL_OK)
		{
			StartAddress += 4; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		}
		else
		{
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return 0;
}

void Read_Data(uint32_t StartAddress, uint32_t *RxBuf, uint16_t WordSize)
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t*) StartAddress;
		StartAddress += 4;
		RxBuf++;
		if (!(WordSize--))
			break;
	}
}

void Flag_up()
{
	uint32_t Data[1];
	Data[0] = 0x10;
//	EEPROM_WriteData(CONFIGURATION_LOCATION, Data, 1);
	Write_Data(OTA_LOCATION, Data, 1);
	HAL_NVIC_SystemReset();
}

