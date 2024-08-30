#ifndef __LIS3DSH_H__
#define __LIS3DSH_H__

#include <Arduino.h> // for byte data type

// register addresses

#define LIS3DSH_WHO_AM_I        0x0F

#define LIS3DSH_CTRL_REG1       0x21
#define LIS3DSH_CTRL_REG2       0x22
#define LIS3DSH_CTRL_REG3       0x23
#define LIS3DSH_CTRL_REG4       0x20
#define LIS3DSH_CTRL_REG5       0x24
#define LIS3DSH_CTRL_REG6       0x25
#define LIS3DSH_OUT_TEMP        0x0C

#define LIS3DSH_OUT_X_L         0x28
#define LIS3DSH_OUT_X_H         0x29
#define LIS3DSH_OUT_Y_L         0x2A
#define LIS3DSH_OUT_Y_H         0x2B
#define LIS3DSH_OUT_Z_L         0x2C
#define LIS3DSH_OUT_Z_H         0x2D

#define LIS3DSH_TIM1_1L         0x54
#define LIS3DSH_THRS2_1         0x56
#define LIS3DSH_MASK1_A         0x5A
#define LIS3DSH_MASK1_B         0x59
#define LIS3DSH_SETT1           0x5B
#define LIS3DSH_ST1_1           0x40
#define LIS3DSH_ST1_2           0x41
#define LIS3DSH_ST1_3           0x42
#define LIS3DSH_ST1_4           0x43
#define LIS3DSH_OUTS1           0x5F
#define LIS3DSH_STAT            0x18

#define LIS3DSH_OUTS2           0x7F
#define LIS3DSH_THRS1_1         0x57
#define LIS3DSH_MASK2_A         0x7A
#define LIS3DSH_MASK2_B         0x79
#define LIS3DSH_SETT2           0x7B
#define LIS3DSH_ST2_1           0x60
#define LIS3DSH_ST2_2           0x61
#define LIS3DSH_ST2_3           0x62
#define LIS3DSH_ST2_4           0x63

#define LIS3DSH_FIFO_CTRL_REG	0x2E

/* bit mask for state machine operation codes */
#define NEXT                    <<0 //don't surround with parenthesis!
#define RESET                   <<4
#define NOP_CMD                 0x0
#define LNTH1                   0x7
#define LLTH2                   0xA
#define GNTH2                   0x6
#define GNTH1                   0x5
#define TI1                     0x1
#define SSYNC                   0xFF
#define CONT                    0x11

typedef enum
{
    LIS3DSH_ERROR = 0x00,
    LIS3DSH_SUCCESS = 0x01
} lis3dsh_status_t;

class LIS3DSH
{
    public:
        LIS3DSH();

    public:
        void setSELState(uint8_t SELState);
        lis3dsh_status_t enableDefault();
        lis3dsh_status_t readAccel(int16_t *pX, int16_t *pY, int16_t *pZ);
        lis3dsh_status_t readTemperature(int8_t *pTemperature);
        lis3dsh_status_t setFullScaleRange(const uint8_t FullScaleRange);
        lis3dsh_status_t setIntActive(bool polarity);
        lis3dsh_status_t setBandwidth(const uint8_t Bandwidth);
        lis3dsh_status_t setOutputDataRate(const uint8_t DataRate);
        lis3dsh_status_t enableXYZ(const uint8_t XYZValue);
        lis3dsh_status_t getWhoAmI();
        lis3dsh_status_t setup();
        lis3dsh_status_t softReset();

        lis3dsh_status_t configFreeFallInterrupt(float minDuration, float threshold);
        lis3dsh_status_t resetInterrupt1(void);

        lis3dsh_status_t configMotionInterrupt(float threshold);
        lis3dsh_status_t resetInterrupt2(void);

        void readConfig(void);
        // lis3dsh_status_t test(void);

        uint8_t convertDuration(float milliseconds);
        uint8_t convertThreshold(float milliG);

        const float min_duration = 100;
        const float threshold1 = 280;
        const float threshold2 = 1000;

    private:
        bool writeReg(const uint8_t reg, const uint8_t value);
        bool readReg(uint8_t *output, const uint8_t reg);

    private:
        int I2CAddr;
};

extern LIS3DSH lis3dsh;

#endif //__LIS3DSH_H__
