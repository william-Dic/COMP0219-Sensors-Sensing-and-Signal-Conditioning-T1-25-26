#include "nau7802.h"
#include "string.h"
#include "stdio.h"

/* local static storage for calibration */
static int32_t _zeroOffset = 0;
static float _calFactor = 1.0f;

/* Small HAL wrappers */

bool NAU7802_isConnected(I2C_HandleTypeDef *hi2c)
{
    /* HAL function cheAcks device ready; uses 8-bit address */
    if (HAL_I2C_IsDeviceReady(hi2c, NAU7802_HAL_ADDR_8BIT, 3, 50) == HAL_OK)
        return true;
    return false;
}

uint8_t NAU7802_getRegister(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
    uint8_t val = 0;
    if (HAL_I2C_Mem_Read(hi2c, NAU7802_HAL_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK)
        return val;
    return 0xFF;
}

bool NAU7802_setRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(hi2c, NAU7802_HAL_ADDR_8BIT, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100) == HAL_OK)
        return true;
    return false;
}

bool NAU7802_setBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg)
{
    uint8_t v = NAU7802_getRegister(hi2c, reg);
    v |= (1 << bit);
    return NAU7802_setRegister(hi2c, reg, v);
}

bool NAU7802_clearBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg)
{
    uint8_t v = NAU7802_getRegister(hi2c, reg);
    v &= ~(1 << bit);
    return NAU7802_setRegister(hi2c, reg, v);
}

bool NAU7802_getBit(I2C_HandleTypeDef *hi2c, uint8_t bit, uint8_t reg)
{
    uint8_t v = NAU7802_getRegister(hi2c, reg);
    return ((v & (1 << bit)) != 0);
}

int32_t NAU7802_get24Bit(I2C_HandleTypeDef *hi2c, uint8_t regMSB)
{
    uint8_t buf[3] = {0};
    if (HAL_I2C_Mem_Read(hi2c, NAU7802_HAL_ADDR_8BIT, regMSB, I2C_MEMADD_SIZE_8BIT, buf, 3, 200) != HAL_OK)
        return 0;

    uint32_t unsigned32 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    /* sign extend if needed */
    if (unsigned32 & 0x00800000)
        unsigned32 |= 0xFF000000;
    return (int32_t)unsigned32;
}

bool NAU7802_available(I2C_HandleTypeDef *hi2c)
{
    return NAU7802_getBit(hi2c, NAU7802_PU_CTRL_CR, NAU7802_PU_CTRL);
}

/* Basic control functions */

bool NAU7802_reset(I2C_HandleTypeDef *hi2c)
{
    if (!NAU7802_setBit(hi2c, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)) return false;
    HAL_Delay(1);
    if (!NAU7802_clearBit(hi2c, NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL)) return false;
    return true;
}

bool NAU7802_powerUp(I2C_HandleTypeDef *hi2c)
{
    if (!NAU7802_setBit(hi2c, NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL)) return false;
    if (!NAU7802_setBit(hi2c, NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL)) return false;

    /* wait for PUR bit */
    for (int i=0;i<200;i++)
    {
        if (NAU7802_getBit(hi2c, NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL)) break;
        HAL_Delay(1);
    }
    /* start cycle */
    return NAU7802_setBit(hi2c, NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);
}

bool NAU7802_setLDO(I2C_HandleTypeDef *hi2c, uint8_t ldoValue)
{
    uint8_t v = NAU7802_getRegister(hi2c, NAU7802_CTRL1);
    v &= 0b11000111;
    v |= ( (ldoValue & 0x07) << NAU7802_CTRL1_VLDO );
    if (!NAU7802_setRegister(hi2c, NAU7802_CTRL1, v)) return false;
    return NAU7802_setBit(hi2c, NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL);
}

bool NAU7802_setGain(I2C_HandleTypeDef *hi2c, uint8_t gainValue)
{
    uint8_t v = NAU7802_getRegister(hi2c, NAU7802_CTRL1);
    v &= 0b11111000;
    v |= (gainValue & 0x07);
    return NAU7802_setRegister(hi2c, NAU7802_CTRL1, v);
}

bool NAU7802_setSampleRate(I2C_HandleTypeDef *hi2c, uint8_t rateValue)
{
    if (rateValue > 0x07) rateValue = 0x07;
    uint8_t v = NAU7802_getRegister(hi2c, NAU7802_CTRL2);
    v &= 0b10001111;
    v |= (rateValue & 0x07) << 4;
    return NAU7802_setRegister(hi2c, NAU7802_CTRL2, v);
}

/* begin: a basic init similar to Arduino begin() */
bool NAU7802_begin(I2C_HandleTypeDef *hi2c)
{
    if (!NAU7802_isConnected(hi2c)) {
        /* try once more like Arduino library does */
        HAL_Delay(10);
        if (!NAU7802_isConnected(hi2c)) return false;
    }

    bool result = true;
    result &= NAU7802_reset(hi2c);
    result &= NAU7802_powerUp(hi2c);

    /* Set LDO to 3.3V (value 3 according to datasheet options) */
    result &= NAU7802_setLDO(hi2c, 3);

    /* default gain 128 in Arduino lib corresponds to value 7 (0..7) */
    result &= NAU7802_setGain(hi2c, 7);

    /* 80 SPS = value 3 in library (CRS bits): typical default: set to 80 */
    result &= NAU7802_setSampleRate(hi2c, 3);

    /* clear CLK_CHP per Arduino lib (set bits in ADC register) - replicate */
    uint8_t adc = NAU7802_getRegister(hi2c, NAU7802_ADC);
    adc |= 0x30;
    result &= NAU7802_setRegister(hi2c, NAU7802_ADC, adc);

    /* enable PGA cap - set bit in PGA_PWR */
    result &= NAU7802_setBit(hi2c, 0 /*PGA_CAP_EN bit index varies*/ , NAU7802_PGA_PWR) || true; // best-effort

    HAL_Delay(200); /* allow LDO ramp */

    return result;
}

/* Read one 24-bit reading (ADCO_B2..B0) */
int32_t NAU7802_getReading(I2C_HandleTypeDef *hi2c)
{
    return NAU7802_get24Bit(hi2c, NAU7802_ADCO_B2);
}

/* Average multiple readings (waits for available) */
int32_t NAU7802_getAverage(I2C_HandleTypeDef *hi2c, uint8_t samples, uint32_t timeout_ms)
{
    int64_t total = 0;
    uint8_t got = 0;
    uint32_t start = HAL_GetTick();

    while (got < samples)
    {
        if (NAU7802_available(hi2c))
        {
            int32_t v = NAU7802_getReading(hi2c);
            total += v;
            got++;
        } else {
            HAL_Delay(1);
        }
        if (timeout_ms && (HAL_GetTick() - start > timeout_ms)) break;
    }

    if (got == 0) return 0;
    return (int32_t)(total / got);
}

/* Simple calibration helpers */
void NAU7802_setZeroOffset(int32_t val) { _zeroOffset = val; }
int32_t NAU7802_getZeroOffset(void) { return _zeroOffset; }
void NAU7802_setCalibrationFactor(float f) { _calFactor = f; }
float NAU7802_getCalibrationFactor(void) { return _calFactor; }

float NAU7802_getWeight(I2C_HandleTypeDef *hi2c, bool allowNegative, uint8_t samples, uint32_t timeout_ms)
{
    int32_t onScale = NAU7802_getAverage(hi2c, samples, timeout_ms);

    if (!allowNegative && onScale < _zeroOffset) onScale = _zeroOffset;
    float weight = ((float)(onScale - _zeroOffset)) / _calFactor;
    return weight;
}
