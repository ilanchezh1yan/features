/*
 * DWT_delay.h
 *
 *  Created on: Oct 10, 2024
 *      Author: ilanc
 */

#ifndef INC_DWT_DELAY_H_
#define INC_DWT_DELAY_H_

#include <stdint.h>

void DWT_Init(void);
void DWT_Delay(uint32_t us);
#endif /* INC_DWT_DELAY_H_ */

