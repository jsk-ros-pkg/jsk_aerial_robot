/*
 * flashmemory.h
 *
 *  Created on: 2017/10/11
 *      Author: anzai
 */

#ifndef APPLICATION_FLASHMEMORY_FLASHMEMORY_H_
#define APPLICATION_FLASHMEMORY_FLASHMEMORY_H_

#include "stm32f4xx_hal.h"
#include <vector>

namespace Flashmemory {
	void init(uint32_t data_address, uint32_t data_sector);
	void addValue(void* ptr, size_t size);
	void read();
	void erase();
	void write();
}

#endif /* APPLICATION_JSK_LIB_FLASHMEMORY_FLASHMEMORY_H_ */
