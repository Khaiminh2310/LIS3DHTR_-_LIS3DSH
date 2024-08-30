#ifndef __LIS3DHTR_H__
#define __LIS3DHTR_H__

#include <stdbool.h>
#include <Arduino.h>
#include <Wire.h>
#include <eBadge.h>

typedef unsigned char u8_t;
typedef short int i16_t;

typedef enum
{
    LIS3DH_ERROR = 0x00,
    LIS3DH_SUCCESS = 0x01,
    LIS3DH_HW_ERROR = 0x02
} lis3dh_status_t;

typedef struct
{
    i16_t AXIS_X;
    i16_t AXIS_Y;
    i16_t AXIS_Z;
    float AXIS_X_f;
    float AXIS_Y_f;
    float AXIS_Z_f;
    float AXIS_ALL;
} AxesRaw_t;

typedef struct {
	int16_t 	x; //!< acceleration in the X axis (signed)
	int16_t		y; //!< acceleration in the Y axis (signed)
	int16_t		z; //!< acceleration in the Z axis (signed)
} LIS3DHSample;

/*!
 * @brief  LIS3DH class
 */
class LIS3DH
{
public:
    LIS3DH(TwoWire &wire, uint8_t sad0 = 0);

    lis3dh_status_t configFreeFallInterrupt(float _minDuration, float _threshold);
    lis3dh_status_t configMotionInterrupt(float _minDuration, float _threshold);
    lis3dh_status_t resetInterrupt1(void);
    lis3dh_status_t resetInterrupt2(void);
    lis3dh_status_t getWhoAmI();
    lis3dh_status_t setup();

    bool readAccel(LIS3DHSample &sample);
    void readTemperature(int16_t *pTemperature);
    void enableTemperature(bool enable = true);

    uint8_t SAD_0; // 0 or 1 depending on the state of the SD0/SA0 pin
    TwoWire &wire; // Wire or Wire1, when using I2C mode

    static const uint8_t I2C_INCREMENT= 0x80;
    static const uint8_t WHO_AM_I = 0x33;
    static const uint8_t STATUS_ZYXDA = 0x08;
    const uint8_t SENSITIVITY = 0x04;
    static const uint8_t RATE_100_HZ = 0x05; 
    static const uint8_t ALL_AXIS_ENABLE = 0x07; 
    static const uint8_t FS_2G = 0x00; 
    static const uint8_t LPen = 0;
    static const uint8_t HR = 0;
    static const unsigned long RECALIBRATION_MOVEMENT_DELAY = 100;

    const float freefall_duration   = 20;       //ms
    const float freefall_threshold  = 350;      //mg
    const float motion_duration     = 0;        //ms
    const float motion_threshold    = 200;      //mg

private:
    bool writeData(uint8_t addr, const uint8_t *buf, size_t numBytes);
    bool readData(uint8_t addr, uint8_t *buf, size_t numBytes);

    uint8_t readReg8(uint8_t addr);
    uint16_t readReg16(uint8_t addr);
    bool writeReg8(uint8_t addr, uint8_t value);
    bool writeReg16(uint8_t addr, uint16_t value);

    uint8_t getI2CAddr() const;
    lis3dh_status_t setDataRate(uint8_t DataRate);
    lis3dh_status_t enableXYZ(uint8_t _en);
    lis3dh_status_t setOperatingMode(uint8_t LPen_bit, uint8_t HR_bit);
    lis3dh_status_t setFullScaleRange(uint8_t _fs);
    lis3dh_status_t setIntActive(bool polarity);
    lis3dh_status_t softReset();
    bool calibrateFilter(unsigned long stationaryTime, unsigned long maxWaitTime = 0);

    uint8_t convertDuration(float milliseconds);
    uint8_t convertThreshold(float milliG);
};

extern LIS3DH lis3dh;
extern LIS3DHSample lis3dh_sample;

// IDENTIFICATION REGISTER
#define LIS3DH_WHO_AM_I         0x0F

// CONFIGURATION REGISTER
#define LIS3DH_CTRL_REG0        0x1E
#define LIS3DH_CTRL_REG1        0x20
#define LIS3DH_CTRL_REG2        0x21
#define LIS3DH_CTRL_REG3        0x22
#define LIS3DH_CTRL_REG4        0x23
#define LIS3DH_CTRL_REG5        0x24
#define LIS3DH_CTRL_REG6        0x25
#define LIS3DH_REFERENCE        0x26

// INTERRUPT 1
#define LIS3DH_INT1_CFG         0x30
#define LIS3DH_INT1_SRC         0x31
#define LIS3DH_INT1_THS         0X32
#define LIS3DH_INT1_DURATION    0x33

// INTERRUPT 2
#define LIS3DH_INT2_CFG         0x34
#define LIS3DH_INT2_SRC         0x35
#define LIS3DH_INT2_THS         0X36
#define LIS3DH_INT2_DURATION    0x37

// OUTPUT REGISTER
#define LIS3DH_STATUS_REG       0x27
#define LIS3DH_OUT_X_L          0x28
#define LIS3DH_OUT_X_H          0x29
#define LIS3DH_OUT_Y_L          0x2A
#define LIS3DH_OUT_Y_H          0x2B
#define LIS3DH_OUT_Z_L          0x2C
#define LIS3DH_OUT_Z_H          0x2D

// TEMPERATURE
#define LIS3DH_TEMP_CFG_REG     0x1F
#define LIS3DH_OUT_ADC3_L       0x0C

#endif /* __LIS3DHTR_H__ */
