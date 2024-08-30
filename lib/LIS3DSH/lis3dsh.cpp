#include <stdio.h>

#include <lis3dsh.h>
#include <Wire.h>
#include <eBadge.h>

#define _MULTI_REGISTER_ACCEL_READ

#define LIS3DSH_ADDR_SEL_HIGH	0x1d	// with SEL/SDO pulled up to VDD
#define LIS3DSH_ADDR_SEL_LOW	0x1e	// with SEL/SDO pulled down to VSS

// Private/Protected Methods

// Writes an accelerometer register
bool LIS3DSH::writeReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(I2CAddr);
    Wire.write(reg);
    Wire.write(value);
    uint8_t res = Wire.endTransmission();

    return (res == 0);
}

// Reads an accelerometer register
bool LIS3DSH::readReg(uint8_t *output, uint8_t reg)
{
    uint8_t value;

    Wire.beginTransmission(I2CAddr);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    Wire.requestFrom(I2CAddr, 1);

    while(Wire.available())
    {
        value = Wire.read();
    }

    *output = value;

    return true;
}

// Public Methods

LIS3DSH::LIS3DSH()
{
    I2CAddr = LIS3DSH_ADDR_SEL_HIGH;
}

lis3dsh_status_t LIS3DSH::getWhoAmI()
{
    uint8_t _whoAmI;

    readReg(&_whoAmI, LIS3DSH_WHO_AM_I);
    if (_whoAmI != 0x3F) {
        return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::softReset()
{
    if (!writeReg(LIS3DSH_CTRL_REG3, 0x01))
        return LIS3DSH_ERROR;
    
    return LIS3DSH_SUCCESS;
}

void LIS3DSH::setSELState(uint8_t SELState)
{
    if(0 == SELState)
    {
        I2CAddr = LIS3DSH_ADDR_SEL_LOW;
    }
    else if(1 == SELState)
    {
        I2CAddr = LIS3DSH_ADDR_SEL_HIGH;
    }
}

// Turns on the LIS3DSH and places it in normal mode.
lis3dsh_status_t LIS3DSH::enableDefault()
{
    // Normal power mode, all axes enabled, 50 Hz ODR
    if (!writeReg(LIS3DSH_CTRL_REG4, 0x5F))
        return LIS3DSH_ERROR;

    // 200 Hz antialias filter, +/- 2g FS range
    if (!writeReg(LIS3DSH_CTRL_REG5, 0x80))
        return LIS3DSH_ERROR;

    // configure FIFO for bypass mode
    if (!writeReg(LIS3DSH_FIFO_CTRL_REG, 0))
        return LIS3DSH_ERROR;

    // disable FIFO, enable register address auto-increment
    if (!writeReg(LIS3DSH_CTRL_REG6, 0x10))
        return LIS3DSH_ERROR;
    
    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::setup()
{
    // LIS3DSH_CTRL_REG4: 0x77
    if (setOutputDataRate(7) != LIS3DSH_SUCCESS) // ODR = 400 Hz
        return LIS3DSH_ERROR;

    if (enableXYZ(7) != LIS3DSH_SUCCESS) // X, Y, Z enabled
        return LIS3DSH_ERROR;

    if (setIntActive(HIGH) != LIS3DSH_SUCCESS) // Interrupt signal active HIGH.
        return LIS3DSH_ERROR;

    my_delay(2);

    // LIS3DSH_CTRL_REG5: 0x00
    if (setFullScaleRange(0) != LIS3DSH_SUCCESS) // Full-scale: ±2 g
        return LIS3DSH_ERROR;

    if (setBandwidth(0) != LIS3DSH_SUCCESS) // 800 Hz antialias filter
        return LIS3DSH_ERROR;
    
    if (!writeReg(LIS3DSH_FIFO_CTRL_REG, 0)) // configure FIFO for bypass mode
        return LIS3DSH_ERROR;
    
    // disable FIFO, enable register address auto-increment
    if (!writeReg(LIS3DSH_CTRL_REG6, 0x10))
        return LIS3DSH_ERROR;

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::configFreeFallInterrupt(float minDuration, float threshold)
{
    uint8_t timer1 = convertDuration(minDuration);
    uint8_t threshold1 = convertThreshold(threshold);
    uint8_t RegisterValue;

    // INT1 signal enabled
    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG3))
    {
        return LIS3DSH_ERROR;
    }
    RegisterValue &= ~(0x08);
    RegisterValue |= 0x08;
    if (!writeReg(LIS3DSH_CTRL_REG3, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    if (!writeReg(LIS3DSH_TIM1_1L, timer1)) // configure duration
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_THRS2_1, threshold1)) // configure threshold
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_MASK1_B, 0xA8)) // enable positive X, Y and Z in mask B
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_MASK1_A, 0xA8)) // enable positive X, Y and Z in mask A
        return LIS3DSH_ERROR;

    // STOP and CONT commands generate an interrupt and perform output actions as OUTC command
    // NEXT condition validation : standard mask always evaluated
    if (!writeReg(LIS3DSH_SETT1, 0x03))
        return LIS3DSH_ERROR;
    
    /* ========================== STATE MACHINE 1 CONFIG =======================*/

    /* set NOP (no-operation) in the RESET condition; 
     * also set the NEXT condition (how to go in the next state) : if LLTH2 (the acc 
     * of axis are less than or equal to threshold 2) */
    if (!writeReg(LIS3DSH_ST1_1, (NOP_CMD RESET) | (LLTH2 NEXT))) // 0x0A
        return LIS3DSH_ERROR;
    
    /* set the RESET condition : if GNTH2 (the acc of axis become greater than threshold 2) ;
     * also set the NEXT condition : TI1 (check if 100 ms passed) */
    if (!writeReg(LIS3DSH_ST1_2, (GNTH2 RESET) | (TI1 NEXT))) // 0x61
        return LIS3DSH_ERROR;
    
    // if (!writeReg(LIS3DSH_ST1_3, CONT))
    //     return LIS3DSH_ERROR;
    
    // if (!writeReg(LIS3DSH_ST1_4, 0)) // set CONT (the final state where the freefall condition is verified)
    //     return LIS3DSH_ERROR;
    if (!writeReg(LIS3DSH_ST1_3, SSYNC))
        return LIS3DSH_ERROR;
    
    if (!writeReg(LIS3DSH_ST1_4, CONT)) // set CONT (the final state where the freefall condition is verified)
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_CTRL_REG1, 0x01)) // enabled State Machine 1
        return LIS3DSH_ERROR;
    
    /*====================== END OF STATE MACHINE 1 CONFIG =====================*/

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::resetInterrupt1(void)
{
    uint8_t _reg;

    DEBUG_SERIAL("IMU_INT_1_PIN before: %s", (digitalRead(IMU_INT1_PIN) == HIGH) ? "HIGH" : "LOW");
    // Reading the OUTS1 register releases the latch
    if (!readReg(&_reg, LIS3DSH_OUTS1))
        return LIS3DSH_ERROR;
    
    DEBUG_SERIAL(">>>>> RESET INT1");
    DEBUG_SERIAL("IMU_INT_1_PIN after: %s", (digitalRead(IMU_INT1_PIN) == HIGH) ? "HIGH" : "LOW");

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::configMotionInterrupt(float threshold)
{
    uint8_t threshold2 = convertThreshold(threshold);
    uint8_t RegisterValue;

    // INT2 signal enabled
    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG3))
    {
        return LIS3DSH_ERROR;
    }
    RegisterValue &= ~(0x10);
    RegisterValue |= 0x10;
    if (!writeReg(LIS3DSH_CTRL_REG3, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    if (!writeReg(LIS3DSH_THRS1_1, 0x0A)) // configure threshold
        return LIS3DSH_ERROR;
    // if (!writeReg(LIS3DSH_THRS1_1, threshold2)) // configure threshold
    //     return LIS3DSH_ERROR;
    
    if (!writeReg(LIS3DSH_MASK2_B, 0xFC)) // enable positive X, Y and Z in mask B
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_MASK2_A, 0xFC)) // enable positive X, Y and Z in mask A
        return LIS3DSH_ERROR;

    // STOP and CONT commands generate an interrupt and perform output actions as OUTC command
    // Diff data for State Machine 2
    if (!writeReg(LIS3DSH_SETT2, 0x11))
        return LIS3DSH_ERROR;

    /* ========================== STATE MACHINE 2 CONFIG =======================*/

    /* the the RESET condition is ignored (NOP); 
     * and the NEXT condition is a “any/triggered axis greater than threshold 1” (GNTH1). */
    // if (!writeReg(LIS3DSH_ST2_3, 0))
    //     return LIS3DSH_ERROR;
    if (!writeReg(LIS3DSH_ST2_1, SSYNC))
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_ST2_2, (NOP_CMD RESET) | (GNTH1 NEXT))) // 0x05
        return LIS3DSH_ERROR;

    if (!writeReg(LIS3DSH_ST2_3, CONT)) // 0x11
        return LIS3DSH_ERROR;
    
    if (!writeReg(LIS3DSH_CTRL_REG2, 0x09)) // enabled State Machine 2
        return LIS3DSH_ERROR;
    
    /*====================== END OF STATE MACHINE 2 CONFIG =====================*/

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::resetInterrupt2(void)
{
    uint8_t _reg;

    DEBUG_SERIAL("IMU_INT_2_PIN before: %s", (digitalRead(IMU_INT2_PIN) == HIGH) ? "HIGH" : "LOW");
    // Reading the OUTS2 register releases the latch
    if (!readReg(&_reg, LIS3DSH_OUTS2))
        return LIS3DSH_ERROR;

    DEBUG_SERIAL(">>>>> RESET INT2");
    DEBUG_SERIAL("IMU_INT_2_PIN after: %s", (digitalRead(IMU_INT2_PIN) == HIGH) ? "HIGH" : "LOW");

    return LIS3DSH_SUCCESS;
}

void LIS3DSH::readConfig()
{
    uint8_t _reg;

    readReg(&_reg, LIS3DSH_STAT);
    DEBUG_SERIAL("LIS3DSH_STAT: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG1);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG2);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG2: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG3);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG3: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG4);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG4: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG5);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG5: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_CTRL_REG6);
    DEBUG_SERIAL("LIS3DSH_CTRL_REG6: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_TIM1_1L);
    DEBUG_SERIAL("LIS3DSH_TIM1_1L: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_THRS2_1);
    DEBUG_SERIAL("LIS3DSH_THRS2_1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_SETT1);
    DEBUG_SERIAL("LIS3DSH_SETT1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST1_1);
    DEBUG_SERIAL("LIS3DSH_ST1_1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST1_2);
    DEBUG_SERIAL("LIS3DSH_ST1_2: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST1_3);
    DEBUG_SERIAL("LIS3DSH_ST1_3: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST1_4);
    DEBUG_SERIAL("LIS3DSH_ST1_4: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_THRS1_1);
    DEBUG_SERIAL("LIS3DSH_THRS1_1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_SETT2);
    DEBUG_SERIAL("LIS3DSH_SETT2: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST2_1);
    DEBUG_SERIAL("LIS3DSH_ST2_1: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST2_2);
    DEBUG_SERIAL("LIS3DSH_ST1_2: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST2_3);
    DEBUG_SERIAL("LIS3DSH_ST1_3: 0x%02X", _reg);

    _reg = 0xff;
    readReg(&_reg, LIS3DSH_ST2_4);
    DEBUG_SERIAL("LIS3DSH_ST1_4: 0x%02X", _reg);
}

// Reads the 3 accelerometer channels
lis3dsh_status_t LIS3DSH::readAccel(int16_t *pX, int16_t *pY, int16_t *pZ)
{
    uint8_t xla;
    uint8_t xha;
    uint8_t yla;
    uint8_t yha;
    uint8_t zla;
    uint8_t zha;

    if((NULL != pX) && (NULL != pY) && (NULL != pZ))
    {
        Wire.beginTransmission(I2CAddr);

#ifdef _MULTI_REGISTER_ACCEL_READ
        // assert the MSB of the address to get the accelerometer
        // to do slave-transmit subaddress updating.
        Wire.write(LIS3DSH_OUT_X_L | (1 << 7));
        if (Wire.endTransmission() != 0) {
            *pX = 0;
            *pY = 0;
            *pZ = 0;
            return LIS3DSH_ERROR;
        }
        Wire.requestFrom(I2CAddr, 6);

        while (Wire.available() < 6);

        xla = Wire.read();
        xha = Wire.read();
        yla = Wire.read();
        yha = Wire.read();
        zla = Wire.read();
        zha = Wire.read();

#else

        readReg(&xla, LIS3DSH_OUT_X_L);
        readReg(&xha, LIS3DSH_OUT_X_H);
        readReg(&yla, LIS3DSH_OUT_Y_L);
        readReg(&yha, LIS3DSH_OUT_Y_H);
        readReg(&zla, LIS3DSH_OUT_Z_L);
        readReg(&zha, LIS3DSH_OUT_Z_H);

#endif	//MULTI_REGISTER_ACCEL_READ

        *pX = (int16_t)(xha << 8 | xla);
        *pY = (int16_t)(yha << 8 | yla);
        *pZ = (int16_t)(zha << 8 | zla);

        return LIS3DSH_SUCCESS;
    }
    else
    {
        return LIS3DSH_ERROR;
    }
}

lis3dsh_status_t LIS3DSH::readTemperature(int8_t *pTemperature)
{
    uint8_t RegisterValue;

    if(NULL == pTemperature)
    {
        return LIS3DSH_ERROR;
    }
    else
    {
        if (readReg(&RegisterValue, LIS3DSH_OUT_TEMP))
        {
            *pTemperature = (int8_t)RegisterValue + 25;
            return LIS3DSH_SUCCESS;
        }
        else
        {
            *pTemperature = 0;
            return LIS3DSH_ERROR;
        }
    }
}

lis3dsh_status_t LIS3DSH::setFullScaleRange(uint8_t FullScaleRange)
{
    uint8_t RegisterValue;

    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG5))
    {
        return LIS3DSH_ERROR;
    }

    RegisterValue &= ~(0x38);
    RegisterValue |= ((FullScaleRange & 0x07) << 3);

    if (!writeReg(LIS3DSH_CTRL_REG5, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::setBandwidth(uint8_t Bandwidth)
{
    uint8_t RegisterValue;

    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG5))
    {
        return LIS3DSH_ERROR;
    }

    RegisterValue &= ~(0xC0);
    RegisterValue |= ((Bandwidth & 0x03) << 6);

    if (!writeReg(LIS3DSH_CTRL_REG5, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::setOutputDataRate(uint8_t DataRate)
{
    uint8_t RegisterValue;

    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG4))
    {
        return LIS3DSH_ERROR;
    }

    RegisterValue &= ~(0xF0);
    RegisterValue |= ((DataRate & 0x0f) << 4);

    if (!writeReg(LIS3DSH_CTRL_REG4, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::setIntActive(bool polarity)
{
    if (polarity == HIGH)
    {
        if (!writeReg(LIS3DSH_CTRL_REG3, 0x40)) // DRY active HIGH on INT pin, interrupt signal latched
            return LIS3DSH_ERROR;
    }
    else if (polarity == LOW)
    {
        if (!writeReg(LIS3DSH_CTRL_REG3, 0x00)) // DRY active LOW on INT pin, interrupt signal latched
            return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

lis3dsh_status_t LIS3DSH::enableXYZ(uint8_t XYZValue)
{
    uint8_t RegisterValue;

    if (!readReg(&RegisterValue, LIS3DSH_CTRL_REG4))
    {
        return LIS3DSH_ERROR;
    }

    RegisterValue &= ~(0x07);
    RegisterValue |= (XYZValue & 0x07);

    if (!writeReg(LIS3DSH_CTRL_REG4, RegisterValue))
    {
        return LIS3DSH_ERROR;
    }

    return LIS3DSH_SUCCESS;
}

/**
 * @brief converts time from milliseconds to the corresponding byte
 * @param milliseconds : the freefall detection time interval expressed in milliseconds
 * @return byte : the time interval converted in byte
 */
uint8_t LIS3DSH::convertDuration(float milliseconds)
{
    // 1 LSB = 1/ODR = 1/400 Hz
    float temp = milliseconds /(2.5);
    int byte = (int) temp;
    if (byte < 0)
        return 0;
    else if (byte > 255)
        return 255;
    else
        return (uint8_t) byte;
}

/**
 * @brief converts acceleration from milli-g to the corresponding byte
 * @param milliG : the threshold expressed in milliG
 * @return byte : the threshold converted in byte
 */
uint8_t LIS3DSH::convertThreshold(float milliG)
{
    // 1 LSB = 2g/(2^7)
    float temp = milliG / (15.625);
    int byte = (int) temp;
    if (byte < 0)
        return 0;
    else if (byte > 255)
        return 255;
    else
        return (uint8_t) byte;
}

// lis3dsh_status_t LIS3DSH::test()
// {
//     if (!writeReg(LIS3DSH_CTRL_REG1, 0x01)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_CTRL_REG3, 0x48)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_CTRL_REG4, 0x67)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_CTRL_REG5, 0x00)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_THRS1_1, 0x55)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_ST1_1, 0x05)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_ST1_2, 0x11)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_ST1_3, 0x00)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_ST1_4, 0x00)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_MASK1_A, 0xFC)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_MASK1_B, 0xFC)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
//     if (!writeReg(LIS3DSH_SETT1, 0x01)) // enable positive X, Y and Z in mask B
//         return LIS3DSH_ERROR;
    
//     return LIS3DSH_SUCCESS;
// }

LIS3DSH lis3dsh = LIS3DSH();
