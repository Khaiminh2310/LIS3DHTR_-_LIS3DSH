#include "lis3dh.h"

/*!
 * @brief  LIS3DH constructor
 */
LIS3DH::LIS3DH(TwoWire &wire, uint8_t sad0) : SAD_0(sad0), wire(wire) {}

uint8_t LIS3DH::readReg8(uint8_t addr)
{
    uint8_t resp[1];
    readData(addr, resp, sizeof(resp));

    return resp[0];
}

uint16_t LIS3DH::readReg16(uint8_t addr)
{
    uint8_t resp[2];
    readData(addr, resp, sizeof(resp));

    return resp[0] | (((uint16_t)resp[1]) << 8);
}


bool LIS3DH::writeReg8(uint8_t addr, uint8_t value)
{
    uint8_t req[1];
    req[0] = value;

    return (writeData(addr, req, sizeof(req)));
}

bool LIS3DH::writeReg16(uint8_t addr, uint16_t value)
{
    uint8_t req[2];
    req[0] = value & 0xff;
    req[1] = value >> 8;

    return (writeData(addr, req, sizeof(req)));
}

bool LIS3DH::readData(uint8_t addr, uint8_t *buf, size_t numBytes)
{
    uint8_t i2cAddr = getI2CAddr();
    wire.beginTransmission(i2cAddr);

    if (numBytes > 1) {
        addr |= I2C_INCREMENT;
    }
    wire.write(addr);

    uint8_t res = wire.endTransmission();
    if (res != 0)
    {
        return false;
    }

    wire.requestFrom(i2cAddr, numBytes);
    for(size_t ii = 0; ii < numBytes && wire.available(); ii++)
    {
        buf[ii] = wire.read();
    }
    return true;
}

bool LIS3DH::writeData(uint8_t addr, const uint8_t *buf, size_t numBytes)
{
    uint8_t i2cAddr = getI2CAddr();
    wire.beginTransmission(i2cAddr);

    if (numBytes > 1)
    {
        addr |= I2C_INCREMENT;
    }
    wire.write(addr);
    for(size_t ii = 0; ii < numBytes; ii++)
    {
        wire.write(buf[ii]);
    }

    uint8_t res = wire.endTransmission();

    return (res == 0);
}

uint8_t LIS3DH::getI2CAddr() const
{
    uint8_t addr = (0x18 | SAD_0);

    return addr;
}

lis3dh_status_t LIS3DH::getWhoAmI()
{
    for(int tries = 0; tries < 10; tries++)
    {
        uint8_t whoami = readReg8(LIS3DH_WHO_AM_I);
        if (whoami == WHO_AM_I)
        {
            return LIS3DH_SUCCESS;
        }
        delay(1);
    }

    return LIS3DH_HW_ERROR;
}

lis3dh_status_t LIS3DH::setDataRate(uint8_t DataRate)
{
    uint8_t RegisterValue;

    RegisterValue = readReg8(LIS3DH_CTRL_REG1);

    RegisterValue &= ~(0xF0);
    RegisterValue |= ((DataRate & 0x0f) << 4);

    if (!writeReg8(LIS3DH_CTRL_REG1, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::enableXYZ(uint8_t _en)
{
    uint8_t RegisterValue;

    RegisterValue = readReg8(LIS3DH_CTRL_REG1);

    RegisterValue &= ~(0x07);
    RegisterValue |= (_en & 0x07);

    if (!writeReg8(LIS3DH_CTRL_REG1, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::setOperatingMode(uint8_t LPen_bit, uint8_t HR_bit)
{
    uint8_t RegisterValue;

    RegisterValue = readReg8(LIS3DH_CTRL_REG1);

    RegisterValue &= ~(0x08);
    RegisterValue |= ((LPen_bit & 0x01) << 3);

    if (!writeReg8(LIS3DH_CTRL_REG1, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    RegisterValue = readReg8(LIS3DH_CTRL_REG4);

    RegisterValue &= ~(0x08);
    RegisterValue |= ((HR_bit & 0x01) << 3);

    if (!writeReg8(LIS3DH_CTRL_REG1, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::setFullScaleRange(uint8_t _fs)
{
    uint8_t RegisterValue;

    RegisterValue = readReg8(LIS3DH_CTRL_REG4);

    RegisterValue &= ~(0x30);
    RegisterValue |= ((_fs & 0x03) << 4);

    if (!writeReg8(LIS3DH_CTRL_REG4, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

void LIS3DH::enableTemperature(bool enable)
{
    if (enable)
    {
        writeReg8(LIS3DH_TEMP_CFG_REG, 0xC0);
    }
    else
    {
        writeReg8(LIS3DH_TEMP_CFG_REG, 0x00);
    }
}

lis3dh_status_t LIS3DH::configFreeFallInterrupt(float _minDuration, float _threshold)
{
    u8_t _reg;
    uint8_t timer = convertDuration(_minDuration);
    uint8_t threshold = convertThreshold(_threshold);

    // enable IA1 interrupt on INT1
    _reg = readReg8(LIS3DH_CTRL_REG3);
    _reg |= 0x40;
    if (!writeReg8(LIS3DH_CTRL_REG3, _reg))
    {
        return LIS3DH_ERROR;
    }

    // interrupt 1 request latched
    _reg = readReg8(LIS3DH_CTRL_REG5);
    _reg |= 0x08;
    if (!writeReg8(LIS3DH_CTRL_REG5, _reg))
    {
        return LIS3DH_ERROR;
    }

    if (!writeReg8(LIS3DH_INT1_THS, threshold))
    {
        return LIS3DH_ERROR;
    }

    if (!writeReg8(LIS3DH_INT1_DURATION, timer))
    {
        return LIS3DH_ERROR;
    }

    if (!writeReg8(LIS3DH_INT1_CFG, 0x95))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::configMotionInterrupt(float _minDuration, float _threshold)
{
    /*
    * CTRL_REG6 = 0x02 // IA2 Interrupt on INT2
    * INT2_THS= 0x10 // wake-up threshold 3g
    * INT2_DURATION = 0x00
    * INT2_CFG = 0x2A // setup wake-up
    */
    u8_t _reg;
    uint8_t timer = convertDuration(_minDuration);
    uint8_t threshold = convertThreshold(_threshold);

    // enable IA2 interrupt on INT1
    _reg = readReg8(LIS3DH_CTRL_REG6);
    _reg |= 0x20;
    if (!writeReg8(LIS3DH_CTRL_REG6, _reg))
    {
        return LIS3DH_ERROR;
    }

    // interrupt 2 request latched
    _reg = readReg8(LIS3DH_CTRL_REG5);
    _reg |= 0x02;
    if (!writeReg8(LIS3DH_CTRL_REG5, _reg))
    {
        return LIS3DH_ERROR;
    }

    if (!writeReg8(LIS3DH_INT2_THS, threshold))
    {
        return LIS3DH_ERROR;
    }

    if (!writeReg8(LIS3DH_INT2_DURATION, timer))
    {
        return LIS3DH_ERROR;
    }

    // Enable XH and YH interrupt generation
    if (!writeReg8(LIS3DH_INT2_CFG, 0x0A))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::resetInterrupt1(void)
{
    u8_t value;

    value = readReg8(LIS3DH_INT1_SRC);

    value = 0x95;
    if (!writeReg8(LIS3DH_INT1_CFG, value))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

lis3dh_status_t LIS3DH::resetInterrupt2(void)
{
    u8_t value;

    value = readReg8(LIS3DH_INT2_SRC);

    value = 0x2A;
    if (!writeReg8(LIS3DH_INT2_CFG, value))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

bool LIS3DH::readAccel(LIS3DHSample &sample)
{
    uint8_t statusAuxReg = readReg8(LIS3DH_STATUS_REG);

    bool hasData = ((statusAuxReg & STATUS_ZYXDA) != 0);

    if (hasData)
    {
        uint8_t resp[6];
        readData(LIS3DH_OUT_X_L, resp, sizeof(resp));

        sample.x = (int16_t) (resp[0] | (((uint16_t)resp[1]) << 8));
        sample.y = (int16_t) (resp[2] | (((uint16_t)resp[3]) << 8));
        sample.z = (int16_t) (resp[4] | (((uint16_t)resp[5]) << 8));
    }
    return hasData;
}

void LIS3DH::readTemperature(int16_t *pTemperature)
{
    // The resolution is 10 bits if the LPen in CTRL_REG1 is cleared (highresolution / normal mode)
    // otherwise, in low-power mode, the ADC resolution is 8-bit.
    if (lis3dh.LPen == 0)
    {
        *pTemperature = ((int16_t) readReg16(LIS3DH_OUT_ADC3_L)) / 1024;
    }
    else
    {
        *pTemperature = ((int16_t) readReg16(LIS3DH_OUT_ADC3_L)) / 256;
    }
}

lis3dh_status_t LIS3DH::softReset()
{

}

lis3dh_status_t LIS3DH::setIntActive(bool polarity)
{
    uint8_t RegisterValue;

    RegisterValue = readReg8(LIS3DH_CTRL_REG6);

    RegisterValue &= ~(0x02);
    if (polarity == LOW)
    {
        RegisterValue |= 0x02;
    }
    else if (polarity == HIGH)
    {
        RegisterValue |= 0x00;
    }

    if (!writeReg8(LIS3DH_CTRL_REG6, RegisterValue))
    {
        return LIS3DH_ERROR;
    }

    return LIS3DH_SUCCESS;
}

bool LIS3DH::calibrateFilter(unsigned long stationaryTime, unsigned long maxWaitTime)
{
    bool ready = false;

    unsigned long start = millis();
    unsigned long lastMovement = start;
    unsigned long lastRecalibrate = start - RECALIBRATION_MOVEMENT_DELAY;

    while(maxWaitTime == 0 || millis() - start < maxWaitTime)
    {
        uint8_t int1_src = readReg8(LIS3DH_INT1_SRC);
        if ((int1_src & 0x40) != 0)
        {
            lastMovement = lastRecalibrate = millis();
            resetInterrupt1();
        }

        if (lastRecalibrate != 0 && millis() - lastRecalibrate >= RECALIBRATION_MOVEMENT_DELAY)
        {
            Serial.println("recalibrating");
            lastRecalibrate = 0;
            readReg8(LIS3DH_REFERENCE);
            resetInterrupt1();
        }

        if (millis() - lastMovement >= stationaryTime)
        {
            ready = true;
            break;
        }
    }

    return ready;
}

lis3dh_status_t LIS3DH::setup()
{
    lis3dh_status_t return_err;
    uint8_t _reg;

    // To ensure correct operation of the device
    _reg = readReg8(LIS3DH_CTRL_REG0);
    if (_reg != 0x10)
    {
        if (!writeReg8(LIS3DH_INT1_DURATION, 0x10))
        {
            return LIS3DH_ERROR;
        }
    }

    // if (setReference) {
    //     // In normal mode, reading the reference register sets it for the current normal force
    //     // (the normal force of gravity acting on the device)
    //     readReg8(LIS3DH_REFERENCE);
    // }

    return_err = setOperatingMode(LPen, HR); // Nomal mode, BW = ORD/2
    if (return_err != LIS3DH_SUCCESS)
    {
        return return_err;
    }

    my_delay(2);

    return_err = setDataRate(RATE_100_HZ);
    if (return_err != LIS3DH_SUCCESS)
    {
        return return_err;
    }

    return_err = enableXYZ(ALL_AXIS_ENABLE);
    if (return_err != LIS3DH_SUCCESS)
    {
        return return_err;
    }

    // High-pass filter normal mode, disable filter
    if (!writeReg8(LIS3DH_CTRL_REG2, 0x00))
    {
        return LIS3DH_ERROR;
    }

    // BDU: continuous update
    _reg = readReg8(LIS3DH_CTRL_REG4);
    _reg &= 0x7F;
    if (!writeReg8(LIS3DH_CTRL_REG4, _reg))
    {
        return LIS3DH_ERROR;
    }

    return_err = setFullScaleRange(FS_2G);
    if (return_err != LIS3DH_SUCCESS)
    {
        return return_err;
    }

    return_err = setIntActive(HIGH);
    if (return_err != LIS3DH_SUCCESS)
    {
        return return_err;
    }

    return LIS3DH_SUCCESS;
}

uint8_t LIS3DH::convertDuration(float milliseconds)
{
    // 1 LSB = 1/ODR = 1/100 Hz
    float temp = milliseconds / 10;
    int byte = (int) temp;
    if (byte < 0)
    {
        return 0;
    }
    else if (byte > 127)
    {
        return 127;
    }
    else
    {
        return (uint8_t) byte;
    }
}

uint8_t LIS3DH::convertThreshold(float milliG)
{
    // 1 LSB = 16mg (FS = ±2 g)
    float temp = milliG / 16;
    int byte = (int) temp;
    if (byte < 0)
    {
        return 0;
    }
    else if (byte > 127)
    {
        return 127;
    }
    else
    {
        return (uint8_t) byte;
    }
}

LIS3DH lis3dh(Wire, 0x01);
LIS3DHSample lis3dh_sample;
