/*
 * flashmemory.h
 *
 *  Created on: 2017/10/11
 *      Author: anzai
 */

#ifndef APPLICATION_FLASHMEMORY_FLASHMEMORY_H_
#define APPLICATION_FLASHMEMORY_FLASHMEMORY_H_

#if defined(STM32F103xB)
	#include "stm32f1xx_hal.h"
        #include <array>
#else
	#include "stm32f4xx_hal.h"
        #include <vector>
#endif

namespace Flashmemory {
#if defined(STM32F103xB)
  	void init(uint32_t data_address);
#else
	void init(uint32_t data_address, uint32_t data_sector);
#endif
	void addValue(void* ptr, size_t size);
	HAL_StatusTypeDef read();
	void erase();
	void write();
}

#endif /* APPLICATION_JSK_LIB_FLASHMEMORY_FLASHMEMORY_H_ */
