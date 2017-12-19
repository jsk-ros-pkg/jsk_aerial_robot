/*
 * flashmemory.cpp
 *
 *  Created on: 2016/11/18
 *      Author: anzai
 */
#include "flashmemory.h"
#include <string.h>

namespace FlashMemory {
	namespace {
		struct Data {
			void* ptr;
			size_t size;
			Data (void* ptr, size_t size):ptr(ptr), size(size){}
		};
		std::vector<Data> data;
		uint32_t m_data_address, m_data_sector;
		constexpr int FLASH_TIMEOUT_VALUE = 50000; //50s
	}

	void init(uint32_t data_address, uint32_t data_sector){
		m_data_address = data_address;
		m_data_sector = data_sector;
	}

	void addValue(void* ptr, size_t size){
		Data tmp_data(ptr, size);
		data.push_back(tmp_data);
	}

	void read(){
		HAL_StatusTypeDef status = HAL_ERROR;
		status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

		uint32_t data_address = m_data_address;
		if (status == HAL_OK){
			for (unsigned int i = 0; i != data.size(); i++){
				memcpy(data[i].ptr, reinterpret_cast<void*>(data_address), data[i].size);
				data_address += data[i].size;
			}
		}

		/* If the program operation is completed, disable the PG Bit */
	    FLASH->CR &= (~FLASH_CR_PG);
	}

	void erase(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError = 0;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = m_data_sector;
		EraseInitStruct.NbSectors = 1;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		r = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
		if ( r != HAL_OK ) return;
		r = HAL_FLASH_Lock();
	}

	void write(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		uint32_t data_address = m_data_address;
		for (unsigned int i = 0; i != data.size(); i++){
			for (unsigned int j = 0; j != data[i].size; j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, data_address + j, *(static_cast<uint8_t*>(data[i].ptr) + j));
			}
			data_address += data[i].size;
		}
		r = HAL_FLASH_Lock();
	}
}



