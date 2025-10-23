#ifndef NAU7802_H
#define NAU7802_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Device address (7-bit) */
#define NAU7802_DEVICE_ADDRESS         0x2A

/* Register map (subset used) */
#define NAU7802_PU_CTRL                0x00
#define NAU7802_CTRL1                  0x01
#define NAU7802_CTRL2                  0x02

#define NAU7802_OCAL1_B2               0x03
/* ... other OCAL/GCal registers omittaed ... */

#define NAU7802_I2C_CONTROL            0x11
#define NAU7802_ADCO_B2                0x12
#define NAU7802_ADCO_B1                0x13
#define NAU7802_ADCO_B0                0x14
#define NAU7802_ADC                    0x15

#define NAU7802_PGA                    0x1B
#define NAU7802_PGA_PWR                0x1C
#define NAU7802_DEVICE_REV             0x1F

/* Bits in PU_CTRL */
#define NAU7802_PU_CTRL_RR    0
#define NAU7802_PU_CTRL_PUD   1
#define NAU7802_PU_CTRL_PUA   2
#define NAU7802_PU_CTRL_PUR   3
#define NAU7802_PU_CTRL_CS    4
#define NAU7802_PU_CTRL_CR    5
#define NAU7802_PU_CTRL_OSCS  6
#define NAU7802_PU_CTRL_AVDDS 7

/* CTRL1 bits - gain field starts at bit 0 (3 bits) */
#define NAU7802_CTRL1_GAIN     0
#define NAU7802_CTRL1_VLDO     5
#define NAU7802_CTRL1_CRP      7

/* CTRL2 bits - sample rate at bits 4..6 */
#define NAU7802_CTRL2_CALS     0
#define NAU7802_CTRL2_CAL_ERROR 1
#define NAU7802_CTRL2_CHS      2
/* and CRS in bits 4..6 */

/* Convenience macros for HAL addresses */
#define NAU7802_HAL_ADDR_8BIT  ((NAU7802_DEVICE_ADDRESS << 1))

/* API */
bool NAU7802_begin(I2C_HandleTypeDef *hi2c);
bool NAU7802_isConnected(I2C_HandleTypeDef *hi2c);
uint8_t NAU7802_getRegister(I2C_HandleTypeDef *hi2c, uint8_t reg);
bool NAU7802_setRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
bool NAU7802_setBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg);
bool NAU7802_clearBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg);
bool NAU7802_getBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg);

int32_t NAU7802_get24Bit(I2C_HandleTypeDef *hi2c, uint8_t regMSB);
bool NAU7802_available(I2C_HandleTypeDef *hi2c);

bool NAU7802_reset(I2C_HandleTypeDef *hi2c);
bool NAU7802_powerUp(I2C_HandleTypeDef *hi2c);
bool NAU7802_setLDO(I2C_HandleTypeDef *hi2c, uint8_t ldoValue); /* 0..7 */
bool NAU7802_setGain(I2C_HandleTypeDef *hi2c, uint8_t gainValue); /* 0..7 */
bool NAU7802_setSampleRate(I2C_HandleTypeDef *hi2c, uint8_t rateValue); /* 0..7 */

int32_t NAU7802_getReading(I2C_HandleTypeDef *hi2c);
int32_t NAU7802_getAverage(I2C_HandleTypeDef *hi2c, uint8_t samples, uint32_t timeout_ms);

/* calibration helpers (store locally in user code as needed) */
void NAU7802_setZeroOffset(int32_t val);
int32_t NAU7802_getZeroOffset(void);
void NAU7802_setCalibrationFactor(float f);
float NAU7802_getCalibrationFactor(void);
float NAU7802_getWeight(I2C_HandleTypeDef *hi2c, bool allowNegative, uint8_t samples, uint32_t timeout_ms);

#endif /* NAU7802_H */
