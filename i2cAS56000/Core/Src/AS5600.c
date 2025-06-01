/*
 * AS5600.c
 *
 *  Created on: Jun 1, 2025
 *      Author: Kenneth
 */


#include "AS5600.h"
#include <stdio.h>

// Private helper
static uint8_t AS5600_ReadRegister(uint8_t reg)
{
    HAL_StatusTypeDef status;
    uint8_t value = 0;

    status = HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    HAL_Delay(10);

    if (status != HAL_OK)
    {
        printf("I2C read error at reg 0x%02X: %#x\r\n", reg, status);
    }

    return value;
}

// Private helper
static uint16_t AS5600_ReadTwoRegisters(uint8_t reg)
{
    uint8_t high = AS5600_ReadRegister(reg);
    uint8_t low  = AS5600_ReadRegister(reg + 1);
    return ((uint16_t)high << 8) | low;
}

// Public API
uint8_t AS5600_ReadStatus(void)
{
    return AS5600_ReadRegister(AS5600_REG_STATUS);
}

uint16_t AS5600_ReadRawAngle(void)
{
    return AS5600_ReadTwoRegisters(AS5600_REG_RAW_ANGLE_H);
}

uint16_t AS5600_ReadAngle(void)
{
    return AS5600_ReadTwoRegisters(AS5600_REG_ANGLE_H);
}

void DEBUG_ScanI2CBus(void)
{
    printf("Scanning I2C bus:\r\n");
    HAL_StatusTypeDef result;

    for (uint8_t i = 1; i < 128; i++)
    {
        result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 2, 2);
        if (result == HAL_OK)
        {
            printf("0x%X ", i);
        }
        else
        {
            printf(".");
        }
    }
    printf("\r\n");
}
