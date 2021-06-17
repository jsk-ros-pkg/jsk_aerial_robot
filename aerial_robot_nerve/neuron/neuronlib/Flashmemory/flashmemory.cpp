/*
 * flashmemory.cpp
 *
 *  Created on: 2017/10/11
 *      Author: anzai
 */

/*
 * flashmemory.cpp
 *
 *  Created on: 2016/11/18
 *      Author: anzai
 */

#include "flashmemory.h"
#include <string.h>

namespace Flashmemory {
	namespace {
		struct Data {
			void* ptr;
			size_t size;
			Data (){}
			Data (void* ptr, size_t size):ptr(ptr), size(size){}
		};
#if defined(STM32F103xB)
		std::array<Data, 32> data;
		unsigned int data_index = 0;
		uint32_t m_data_address;
#else
                std::vector<Data> data;
		uint32_t m_data_address, m_data_sector;
		constexpr int FLASH_TIMEOUT_VALUE = 50000; //50s
#endif
	}

#if defined(STM32F103xB)
	void init(uint32_t data_address){
		m_data_address = data_address;
	}
#else
	void init(uint32_t data_address, uint32_t data_sector){
		m_data_address = data_address;
		m_data_sector = data_sector;
	}
#endif

	void addValue(void* ptr, size_t size){
		Data tmp_data(ptr, size);
#if defined(STM32F103xB)
                data[data_index++] = tmp_data;
#else
		data.push_back(tmp_data);
#endif
	}

	HAL_StatusTypeDef read(){
		HAL_StatusTypeDef status = HAL_ERROR;
		status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

		uint32_t data_address = m_data_address;
		if (status == HAL_OK){
#if !defined(STM32F103xB)
                        unsigned int data_index = data.size();
#endif
                        for (unsigned int i = 0; i != data_index; i++){
				memcpy(data[i].ptr, reinterpret_cast<void*>(data_address), data[i].size);
				data_address += data[i].size;
			}
		}

		/* If the program operation is completed, disable the PG Bit */
	    FLASH->CR &= (~FLASH_CR_PG);

            return status;
	}

	void erase(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		FLASH_EraseInitTypeDef EraseInitStruct;
#if defined(STM32F103xB)
		uint32_t PageError = 0;
		EraseInitStruct.TypeErase = TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = m_data_address;
		EraseInitStruct.NbPages = 1;

		r = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
#else
                uint32_t SectorError = 0;
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.Sector = m_data_sector;
		EraseInitStruct.NbSectors = 1;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

		r = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
#endif
		if ( r != HAL_OK ) return;
		r = HAL_FLASH_Lock();
	}

	void write(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		uint32_t data_address = m_data_address;

#if defined(STM32F103xB)
		for (unsigned int i = 0; i != data_index; i++){
			for (unsigned int j = 0; j < data[i].size; j+=2) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, data_address + j, *(static_cast<uint16_t*>(data[i].ptr) + j / 2));
			}
			data_address += data[i].size;
		}
#else
                for (unsigned int i = 0; i != data.size(); i++){
			for (unsigned int j = 0; j != data[i].size; j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, data_address + j, *(static_cast<uint8_t*>(data[i].ptr) + j));
			}
			data_address += data[i].size;
		}
#endif
		r = HAL_FLASH_Lock();
	}
}


