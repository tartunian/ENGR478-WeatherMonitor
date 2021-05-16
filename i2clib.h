/*
 * i2clib.h
 *
 *  Created on: Mar 16, 2021
 *      Author: tartunian
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/i2c.h"

#ifndef I2C_H_
#define I2C_H_

void    I2CSend(uint8_t, uint8_t, ...);
uint8_t I2CReceive(uint32_t, uint8_t);
void    I2CWrite(uint8_t, uint8_t, uint8_t);
uint8_t I2CRead(uint32_t, uint8_t);

#endif /* I2C_H_ */
