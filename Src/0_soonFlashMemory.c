#include "0_soonFlashMemory.h"


/**************************************************************************************************************
*	Write ������ �帧�� �Ʒ��� ����.
*	1) Control register unlock
*	2) 4byte ������ ���� ����
*	3) Control register lock
***************************************************************************************************************/
HAL_StatusTypeDef WriteFlash(uint32_t Address, uint32_t data, uint32_t* errorcode)
{
	HAL_StatusTypeDef res = HAL_OK;

	/* Unlock to control */
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR| FLASH_FLAG_WRPERR);

	/* Writing data to flash memory */
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data) == HAL_OK)
	{
		errorcode = 0;
	}
	else
	{
		*errorcode = HAL_FLASH_GetError();
		res = HAL_ERROR;
	}

	/* Lock flash control register */
	HAL_FLASH_Lock();  

	return res;
}

/**************************************************************************************************************
*	Read ������ ��쿡�� ���������� �ش� Memory �ּҸ� Access �Ͽ� 4byte ������ �о�´�.
***************************************************************************************************************/
__IO uint32_t ReadFlash(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}

/**************************************************************************************************************
*	Flash�� ���� ���� �ش� ������ ����� Erase ������ ���캸��.
*	�ڵ��� �帧�� ���캸�� �Ʒ��� ����.
*	1) Control register unlock
*	2) ������� �ϴ� ������ ũ�� ���
*	3) ���� ���� ���
*	4) ������ ĳ�ÿ� �̹� �ö� �������� ��쿡 Cache���� ������ �ʿ��� ��쿡�� Cache Clear
* 	5) Control register lock
***************************************************************************************************************/
HAL_StatusTypeDef EraseFlash(uint32_t startAdd, uint32_t pages)
{
  uint32_t SectorError = 0;
  
  /* Unlock to control */
  HAL_FLASH_Unlock();
    
  /* Erase sectors */
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = FLASH_BANK_1;
  EraseInitStruct.PageAddress = startAdd;
  EraseInitStruct.NbPages = pages;

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    uint32_t errorcode = HAL_FLASH_GetError();            
    return HAL_ERROR;
  }
  
  /* Lock flash control register */
  HAL_FLASH_Lock();
  
  return HAL_OK;  
}

uint32_t doFlashWriteRevision(void)
{
	uint32_t	flashError = 0;
	uint8_t 	i;

	EraseFlash(FLASH_SAVE_CHK, 1);
	
	for(i = 0; i < 16; i++)
	{	
		WriteFlash(FLASH_ADD_THRESHOLD_TEMP 	  	+ (i * 4), TestData.Threshold[0][i].UI32, &flashError);
		WriteFlash(FLASH_ADD_THRESHOLD_TEMP1 	  	+ (i * 4), TestData.Threshold[1][i].UI32, &flashError);
		WriteFlash(FLASH_ADD_NTC_CALIBRATION_TABLE  + (i * 4), TestData.ntcCalibrationTable[0][i].UI32, &flashError);
		WriteFlash(FLASH_ADD_NTC_CALIBRATION_TABLE1	+ (i * 4), TestData.ntcCalibrationTable[1][i].UI32, &flashError);
	}
	
	WriteFlash(FLASH_ADD_NTC_CALIBRATION_CONSTANT, 	TestData.ntcCalibrationConstant.UI32, &flashError);
	WriteFlash(FLASH_ADD_REVISION_APPLY, 			(uint32_t)TestData.revisionApplyFlag, &flashError);
	WriteFlash(FLASH_ADD_REVISION_CONSTANT, 		TestData.revisionConstant.UI32, &flashError);

	WriteFlash(FLASH_SAVE_CHK, FLASH_SAVE_FLAG, &flashError);

	return flashError;
}


#if 0
/**************************************************************************************************************
*	WP�� Ȱ��ȭ �ϴ� �ڵ带 ���캸��. WRPState ���� OB_WRPSTATE_ENABLE �� �����ϰ� HAL_FLASH_OB_Launch �� ȣ���Ѵ�.
***************************************************************************************************************/
HAL_StatusTypeDef EnableWriteProtect(uint32_t add)
{
  FLASH_OBProgramInitTypeDef OBInit; 
  
  HAL_FLASH_OB_Unlock();
  HAL_FLASH_Unlock();
    
  OBInit.OptionType = OPTIONBYTE_WRP;
  OBInit.WRPState   = OB_WRPSTATE_ENABLE;
  OBInit.Banks      = FLASH_BANK_1;
  OBInit.WRPSector  = add;//FLASH_WRP_SECTORS;
  HAL_FLASHEx_OBProgram(&OBInit);   
    
  if (HAL_FLASH_OB_Launch() != HAL_OK)
  {
    return HAL_ERROR;
  }
    
  HAL_FLASH_OB_Lock();  
  HAL_FLASH_Lock();  
  
  return HAL_OK;
}

/**************************************************************************************************************
*	WP�� ��Ȱ��ȭ �ϴ� ����� Enable�� �����ϸ� WRPState ���� �ٸ� ���̴�. 
***************************************************************************************************************/
HAL_StatusTypeDef DisableWriteProtect(uint32_t add)
{
  FLASH_OBProgramInitTypeDef OBInit; 
  
  HAL_FLASH_OB_Unlock();
  HAL_FLASH_Unlock();
  
  OBInit.OptionType = OPTIONBYTE_WRP;
  OBInit.WRPState   = OB_WRPSTATE_DISABLE;
  OBInit.Banks      = FLASH_BANK_1;
  OBInit.WRPSector  = add;//FLASH_WRP_SECTORS;
  HAL_FLASHEx_OBProgram(&OBInit); 
  
  if (HAL_FLASH_OB_Launch() != HAL_OK)
  {
    return HAL_ERROR;
  }
  
  HAL_FLASH_OB_Lock();  
  HAL_FLASH_Lock();
  
  return HAL_OK;
}
#endif

void DoValueFormating(void)
{
	uint8_t i;
	
	for(i = 0; i < 16; i++)
	{	
		TestData.Threshold[0][i].Float				= (float)50.0;	//��� �µ� ���̺� 
		TestData.Threshold[1][i].Float				= (float)50.0;
		TestData.ntcCalibrationTable[0][i].Float	= (float)0.0;	//NTC ���� ���̺� 
		TestData.ntcCalibrationTable[1][i].Float	= (float)0.0;
	}
	TestData.ntcCalibrationConstant.Float = (float)0.0; 			//NTC ������� 
	TestData.revisionConstant.Float = (float)0.3672;				//������� 
	TestData.revisionApplyFlag = 0; 								//���� ���� 
}

void DoLoadFlash(void)
{
	uint8_t i;
	
	for(i = 0; i < 16; i++)
	{	
		TestData.Threshold[0][i].UI32			= ReadFlash(FLASH_ADD_THRESHOLD_TEMP + (i * 4));
		TestData.Threshold[1][i].UI32			= ReadFlash(FLASH_ADD_THRESHOLD_TEMP1 + (i * 4));
		TestData.ntcCalibrationTable[0][i].UI32 = ReadFlash(FLASH_ADD_NTC_CALIBRATION_TABLE + (i * 4));
		TestData.ntcCalibrationTable[1][i].UI32 = ReadFlash(FLASH_ADD_NTC_CALIBRATION_TABLE1 + (i * 4));
	}
	TestData.ntcCalibrationConstant.UI32		= ReadFlash(FLASH_ADD_NTC_CALIBRATION_CONSTANT);
	TestData.revisionApplyFlag					= (uint8_t)ReadFlash(FLASH_ADD_REVISION_APPLY);
	TestData.revisionConstant.Float 			= ReadFlash(FLASH_ADD_REVISION_CONSTANT);
}

