/*
MS5611.h
Library for barometric pressure sensor MS5611-01BA on I2C with arduino

by Petr Gronat@2014

Edited by l.heywang in 2025 to make it compatible with nRF (Zephyr) API.
*/

/*
 * -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */

// Libs
#include "drivers/MS5611.h"

// Zephyr
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

// init lib
#include "init/init.h"
#include "config.h"

// STD
#include <cstdint>

/*
 * -----------------------------------------------------------------
 * LOGGER MODULES
 * -----------------------------------------------------------------
 */
LOG_MODULE_REGISTER(ms5611, PROJECT_LOG_LEVEL);

constexpr uint8_t OSR = 3; // 0-3
constexpr uint8_t CMD_RESET = 0x1E;
constexpr uint8_t CMD_ADC_READ = 0x00;
constexpr uint8_t CMD_CONV_D1_BASE = 0x40;
constexpr uint8_t CMD_CONV_D2_BASE = 0x50;
constexpr uint8_t CONV_REG_SIZE = 0x02;
constexpr uint8_t CMD_PROM_READ_BASE = 0xA2;
constexpr uint8_t PROM_REG_SIZE = 0x02;
constexpr uint8_t NBYTES_CONV = 3;
constexpr uint8_t NBYTES_PROM = 2;

// Temperature sampling period threshold [milliseconds]
// Kindly read the comment bellow in getPressure() method
constexpr int T_THR = 1000;

/*
 * -----------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------
 */
MS5611::MS5611()
{
    // Defining some settings
    _T = 0;
    _P = 0;
    _lastTime = T_THR;

    for (uint8_t k = 0; k < N_PROM_PARAMS; k++)
        _C[k] = 69;

    // Fetch the associated device on the DT
    this->dev = INIT_GetAnI2C(I2CS::BAROMETER);

    begin();
}

MS5611::~MS5611()
{
    // Close the I2C device
    INIT_FreeAnI2C(I2CS::BAROMETER, this->dev);

    // Variables are handled by the C++ compiler
}

void MS5611::begin()
{
    reset();
    k_msleep(200);
    readCalibration();
}

double *MS5611::getPressure()
{
    double temp = getTemperature(); // updates offset values !
    uint32_t D1 = getRawPressure();

    // Datasheet P8 to understand what's done here...
    int64_t OFF = (this->_C[2 - 1] * (2 ^ 16)) +
                  (this->_C[4 - 1] * (this->_dT / (2 ^ 7))) -
                  this->_Off2;

    int64_t SENS = (this->_C[1 - 1] * (2 ^ 15)) +
                   (this->_C[3 - 1] * (this->_dT / (2 ^ 8))) -
                   this->_Sens2;

    this->_P = D1 * SENS - OFF;

    double *retval = (double *)k_malloc(2 * sizeof(double));
    retval[0] = this->_P / 100;
    retval[1] = temp;

    return retval;
}

uint32_t MS5611::getRawPressure()
{
    sendCommand(CMD_CONV_D1_BASE + OSR * CONV_REG_SIZE); // read sensor, prepare a data
    k_msleep(9);
    return readnBytes(&CMD_ADC_READ, 1, NBYTES_CONV); // reading the data
}

double MS5611::getTemperature()
{
    uint32_t D2 = getRawTemperature();

    // As per the datasheet, calculate the values
    this->_dT = D2 - (this->_C[5 - 1] * (2 ^ 8));

    this->_T = 2000 +
               (this->_dT * (this->_C[6 - 1] / (2 ^ 23)));

    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Datasheet P8 to understand what's done here...
    if (this->_T < 20)
    {
        T2 = this->_dT / (2 ^ 31);

        OFF2 = 5 * (((this->_T - 2000) ^ 2) / 2);

        SENS2 = OFF2 / 2;

        if (this->_T < -15)
        {
            OFF2 = OFF2 +
                   (7 * ((this->_T + 1500) ^ 2));

            SENS2 = SENS2 +
                    (11 * (((this->_T + 1500) ^ 2) / 2));
        }
    }

    // Store the values
    this->_T = (this->_T - T2) / 100;
    this->_Off2 = OFF2;
    this->_Sens2 = SENS2;

    return (this->_T - T2) / 100;
}

uint32_t MS5611::getRawTemperature()
{
    sendCommand(CMD_CONV_D2_BASE + OSR * CONV_REG_SIZE); // read sensor, prepare a data
    k_msleep(9);
    return readnBytes(&CMD_ADC_READ, 1, NBYTES_CONV);
}

void MS5611::readCalibration()
{
    for (uint8_t k = 0; k < 6; k++)
    {
        uint8_t cmd = CMD_PROM_READ_BASE + k * 2;
        _C[k] = (uint16_t)(readnBytes(&cmd, 1, NBYTES_PROM) & 0xFFFF); // masking out two LSB
    }
}

void MS5611::getCalibration(uint16_t *C)
{
    for (uint8_t k = 0; k < N_PROM_PARAMS; k++)
        C[k] = _C[k];
    return;
}

void MS5611::sendCommand(uint8_t cmd)
{
    // Replaced with the zephyr call
    int ret = i2c_write_dt(this->dev, &cmd, sizeof(cmd));
    if (ret != 0)
    {
        LOG_ERR("I2C Write failed ! Error code : %d", ret);
    }
    return;
}

uint32_t MS5611::readnBytes(const uint8_t *cmd, uint8_t wlen, uint8_t rlen)
{
    if (!(0 < rlen) & (rlen < 5))
        return -1;

    // Allocate and clear the output buffer
    int rbuf[4];
    memset(rbuf, 0, rlen);

    // Read bytes
    int ret = i2c_write_read_dt(this->dev, cmd, wlen, rbuf, rlen);
    if (ret != 0)
    {
        LOG_ERR("I2C Write failed ! Error code : %d", ret);
    }

    int data = 0;
    for (int8_t k = rlen - 1; k >= 0; k--)
        data |= ((uint32_t)rbuf[sizeof(rlen) - k] << (8 * k)); // concantenate bytes

    return data;
}

void MS5611::reset()
{
    // Replaced with the zephyr call
    const uint8_t buf = CMD_RESET;
    int ret = i2c_write_dt(this->dev, &buf, 1);
    if (ret != 0)
    {
        LOG_ERR("I2C Write failed ! Error code : %d", ret);
    }
    return;
}
