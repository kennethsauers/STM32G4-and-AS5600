/*
 * AS5600.h
 *
 *  Created on: Jun 1, 2025
 *      Author: Kenneth
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32G4xx_hal.h"  // Adjust as necessary for your MCU series


#define AS5600_ADDRESS         (0x36 << 1)  // 7-bit address shifted for HAL
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_REG_ANGLE_H     0x0E
#define AS5600_RESOLUTION      4096

// External I2C handle
extern I2C_HandleTypeDef hi2c1;

// Public API
uint8_t  AS5600_ReadStatus(void);
uint16_t AS5600_ReadRawAngle(void);
uint16_t AS5600_ReadAngle(void);
void DEBUG_ScanI2CBus(void);

#endif /* INC_AS5600_H_ */
