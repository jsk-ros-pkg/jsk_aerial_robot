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
#include <array>

namespace Flashmemory {
	namespace {
		struct Data {
			void* ptr;
			size_t size;
			Data(){}
			Data(void* ptr, size_t size):ptr(ptr), size(size){}
		};
		std::array<Data, 32> data;
		unsigned int data_index = 0;
		uint32_t m_data_address;
	}

	void init(uint32_t data_address){
		m_data_address = data_address;
	}

	void addValue(void* ptr, size_t size){
		Data tmp_data(ptr, size);
		data[data_index++] = tmp_data;
	}

	void read(){
		HAL_StatusTypeDef status = HAL_ERROR;
		status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);

		uint32_t data_address = m_data_address;
		if (status == HAL_OK){
			for (unsigned int i = 0; i != data_index; i++){
				memcpy(data[i].ptr, reinterpret_cast<void*>(data_address), data[i].size);
				data_address += data[i].size;
			}
		}

	    FLASH->CR &= (~FLASH_CR_PG);
	}

	void erase(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t PageError = 0;
		EraseInitStruct.TypeErase = TYPEERASE_PAGES;
		EraseInitStruct.PageAddress = m_data_address;
		EraseInitStruct.NbPages = 1;

		r = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
		if ( r != HAL_OK ) return;
		r = HAL_FLASH_Lock();
	}

	void write(){
		HAL_StatusTypeDef r;

		r = HAL_FLASH_Unlock();
		if( r != HAL_OK ) return;

		uint32_t data_address = m_data_address;
		for (unsigned int i = 0; i != data_index; i++){
			for (unsigned int j = 0; j < data[i].size; j+=2) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, data_address + j, *(static_cast<uint16_t*>(data[i].ptr) + j / 2));
			}
			data_address += data[i].size;
		}
		r = HAL_FLASH_Lock();
	}
}



