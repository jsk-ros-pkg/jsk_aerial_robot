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
#if defined(STM32F103xx)
    std::array<Data, 32> data;
    unsigned int data_index = 0;
    uint32_t m_data_address;
#elif defined(STM32F413xx)
    std::vector<Data> data;
    uint32_t m_data_address, m_data_sector;
    constexpr int FLASH_TIMEOUT_VALUE = 50000; //50s
#elif defined(STM32G473xx)
    std::vector<Data> data;
    uint32_t m_data_address, m_data_bank, m_data_page;
#else
#error "unsupported type"
#endif
  }

#if defined(STM32F103xx)
  void init(uint32_t data_address){
    m_data_address = data_address;
  }
#elif defined(STM32F413xx)
  void init(uint32_t data_address, uint32_t data_sector){
    m_data_address = data_address;
    m_data_sector = data_sector;
  }
#elif defined(STM32G473xx)
  void init(uint32_t data_address, uint32_t data_bank, uint32_t data_page){
    m_data_address = data_address;
    m_data_bank = data_bank;
    m_data_page = data_page;
  }
#else
#error "unsupported type"
#endif

  void addValue(void* ptr, size_t size){
    Data tmp_data(ptr, size);
#if defined(STM32F103xx)
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
      unsigned int data_index = data.size();
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

    FLASH_EraseInitTypeDef erase_init_struct;
    uint32_t error = 0;
#if defined(STM32F103xx)
    erase_init_struct.TypeErase = TYPEERASE_PAGES;
    erase_init_struct.PageAddress = m_data_address;
    erase_init_struct.NbPages = 1;
#elif defined(STM32F413xx)
    erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init_struct.Sector = m_data_sector;
    erase_init_struct.NbSectors = 1;
    erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#elif defined(STM32G473xx)
    erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init_struct.Banks = m_data_bank;
    erase_init_struct.Page = m_data_page;
    erase_init_struct.NbPages = 1;
#else
#error "unsupported type"
#endif

    r = HAL_FLASHEx_Erase(&erase_init_struct, &error);
    if ( r != HAL_OK ) return;
    r = HAL_FLASH_Lock();
  }

  void write(){
    HAL_StatusTypeDef r;

    r = HAL_FLASH_Unlock();
    if( r != HAL_OK ) return;

    uint32_t data_address = m_data_address;

#if defined(STM32F103xx)
    for (unsigned int i = 0; i != data_index; i++){
      for (unsigned int j = 0; j < data[i].size; j+=2) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, data_address + j, *(static_cast<uint16_t*>(data[i].ptr) + j / 2));
      }
      data_address += data[i].size;
    }
#elif defined(STM32F413xx)
    for (unsigned int i = 0; i != data.size(); i++){
      for (unsigned int j = 0; j != data[i].size; j++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, data_address + j, *(static_cast<uint8_t*>(data[i].ptr) + j));
      }
      data_address += data[i].size;
    }
#elif defined(STM32G473xx)
    uint64_t temp = 0xFFFFFFFFFFFFFFFF;
    uint32_t cnt = 0;
    bool flag = false;
    for (unsigned int i = 0; i != data.size(); i++){
      for (unsigned int j = 0; j != data[i].size; j++) {

        memcpy(reinterpret_cast<uint8_t*>(&temp) + cnt % 8,
               static_cast<uint8_t*>(data[i].ptr) + j, 1);

        if (cnt % 8 == 7) {
          uint32_t offset = (cnt / 8) * 8;
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, data_address + offset, temp);
          temp = 0xFFFFFFFFFFFFFFFF;
        }

        cnt ++;
      }
    }

    // last data
    cnt --;
    if (cnt % 8 < 7) {
      uint32_t offset = (cnt / 8) * 8;
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, data_address + offset, temp);
    }

#else
#error "unsupported type"
#endif
    r = HAL_FLASH_Lock();
  }
}


