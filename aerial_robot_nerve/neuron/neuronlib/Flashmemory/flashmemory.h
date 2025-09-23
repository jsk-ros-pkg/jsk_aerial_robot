/*
 * flashmemory.h
 *
 *  Created on: 2017/10/11
 *      Author: anzai
 */

#ifndef APPLICATION_FLASHMEMORY_FLASHMEMORY_H_
#define APPLICATION_FLASHMEMORY_FLASHMEMORY_H_

#if defined(STM32F103xx)
#include "stm32f1xx_hal.h"
        #include <array>
#elif defined(STM32F413xx)
#include "stm32f4xx_hal.h"
        #include <vector>
#elif defined(STM32G473xx)
#include "stm32g4xx_hal.h"
        #include <vector>
#else
#error "please specify the STM32 series"
#endif

namespace Flashmemory {
#if defined(STM32F103xx)
  void init(uint32_t data_address);
#elif defined(STM32F413xx)
  void init(uint32_t data_address, uint32_t data_sector);
#elif defined(STM32G473xx)
  void init(uint32_t data_address, uint32_t data_bank, uint32_t data_page);
#else
#error "please specify the STM32 series"
#endif

  void addValue(void* ptr, size_t size);
  HAL_StatusTypeDef read();
  void erase();
  void write();
}

#endif /* APPLICATION_JSK_LIB_FLASHMEMORY_FLASHMEMORY_H_ */
