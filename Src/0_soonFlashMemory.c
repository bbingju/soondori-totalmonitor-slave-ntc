#include "0_soonFlashMemory.h"


/**************************************************************************************************************
*	Write 동작의 흐름은 아래와 같다.
*	1) Control register unlock
*	2) 4byte 단위로 쓰기 수행
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
*	Read 동작의 경우에는 직관적으로 해당 Memory 주소를 Access 하여 4byte 데이터 읽어온다.
***************************************************************************************************************/
__IO uint32_t ReadFlash(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}

/**************************************************************************************************************
*	Flash에 쓰기 위해 해당 영역을 지우는 Erase 동작을 살펴보자.
*	코드의 흐름을 살펴보면 아래와 같다.
*	1) Control register unlock
*	2) 지우고자 하는 섹터의 크기 계산
*	3) 섹터 삭제 명령
*	4) 데이터 캐시에 이미 올라간 데이터인 경우에 Cache까지 삭제가 필요한 경우에는 Cache Clear
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
*	WP을 활성화 하는 코드를 살펴보자. WRPState 값에 OB_WRPSTATE_ENABLE 을 설정하고 HAL_FLASH_OB_Launch 를 호출한다.
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
*	WP를 비활성화 하는 기능은 Enable과 동일하며 WRPState 값만 다를 뿐이다. 
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
		TestData.Threshold[0][i].Float				= (float)50.0;	//경고 온도 테이블 
		TestData.Threshold[1][i].Float				= (float)50.0;
		TestData.ntcCalibrationTable[0][i].Float	= (float)0.0;	//NTC 교정 테이블 
		TestData.ntcCalibrationTable[1][i].Float	= (float)0.0;
	}
	TestData.ntcCalibrationConstant.Float = (float)0.0; 			//NTC 증감상수 
	TestData.revisionConstant.Float = (float)0.3672;				//보정상수 
	TestData.revisionApplyFlag = 0; 								//보정 선택 
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

