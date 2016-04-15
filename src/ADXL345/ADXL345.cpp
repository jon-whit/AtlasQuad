#include "mbed.h"
#include "ADXL345.h"

ADXL345::ADXL345(PinName sda, PinName scl) : i2c_(sda, scl)
{
    // Set I2C data rate to 400kHz
    i2c_.frequency(400000);
    
    wait_us(500);
}

uint8_t ADXL345::getDeviceID(void)
{
    char tx = ADXL345_DEVID_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

uint8_t ADXL345::getTapThreshold(void)
{
    char tx = ADXL345_THRESH_TAP_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTapThreshold(uint8_t threshold)
{
    char tx[2];
    tx[0] = ADXL345_THRESH_TAP_REG;
    tx[1] = threshold;
        
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

int8_t ADXL345::getOffset(uint8_t axis)
{
    char tx, rx;
    
    switch(axis) 
    {
        case ADXL345_X:
            tx = ADXL345_OFSX_REG;
            break;
        case ADXL345_Y:
            tx = ADXL345_OFSY_REG;
            break;
        case ADXL345_Z:
            tx = ADXL345_OFSZ_REG;
            break;
        default:
            tx = ADXL345_OFSX_REG;
    }
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (int8_t) rx;
}

void ADXL345::setOffset(uint8_t axis, int8_t offset)
{
    char tx[2];
    tx[1] = offset;
    
    switch(axis)
    {
        case ADXL345_X:
            tx[0] = ADXL345_OFSX_REG;
            break;
        case ADXL345_Y:
            tx[0] = ADXL345_OFSY_REG;
            break;
        case ADXL345_Z:
            tx[0] = ADXL345_OFSZ_REG;
            break;
        default:
            tx[0] = ADXL345_OFSX_REG;
    }
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTapDuration(void)
{
    char tx = ADXL345_DUR_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTapDuration(uint8_t duration_us)
{
    char tx[2];
    tx[0] = ADXL345_DUR_REG;
    tx[1] = duration_us;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTapLatency(void)
{
    char tx = ADXL345_LATENT_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTapLatency(uint8_t delay_ms)
{
    char tx[2];
    tx[0] = ADXL345_LATENT_REG;
    tx[1] = delay_ms;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTapWindow(void)
{
    char tx = ADXL345_WINDOW_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTapWindow(uint8_t window_ms)
{
    char tx[2];
    tx[0] = ADXL345_WINDOW_REG;
    tx[1] = window_ms;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getActivityThreshold(void)
{
    char tx = ADXL345_THRESH_ACT_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.write(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setActivityThreshold(uint8_t threshold)
{
    char tx[2];
    tx[0] = ADXL345_THRESH_ACT_REG;
    tx[1] = threshold;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getInactivityThreshold(void)
{
    char tx = ADXL345_THRESH_INACT_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setInactivityThreshold(uint8_t threshold)
{
    char tx[2];
    tx[0] = ADXL345_THRESH_INACT_REG;
    tx[1] = threshold;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTimeInactivity(void)
{
    char tx = ADXL345_TIME_INACT_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTimeInactivity(uint8_t inactivity)
{
    char tx[2];
    tx[0] = ADXL345_TIME_INACT_REG;
    tx[1] = inactivity;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getActivityInactivityControl(void)
{
    char tx = ADXL345_ACT_INACT_CTL_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setActivityInactivityControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_ACT_INACT_CTL_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getFreeFallThreshold(void)
{
    char tx = ADXL345_THRESH_FF_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setFreeFallThreshold(uint8_t threshold)
{
    char tx[2];
    tx[0] = ADXL345_THRESH_FF_REG;
    tx[1] = threshold;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getFreeFallTime(void)
{
    char tx = ADXL345_TIME_FF_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setFreeFallTime(uint8_t time_ms)
{
    char tx[2];
    tx[0] = ADXL345_TIME_FF_REG;
    tx[1] = time_ms;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTapAxesControl(void)
{
    char tx = ADXL345_TAP_AXES_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setTapAxesControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_TAP_AXES_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getTapSource(void)
{
    char tx = ADXL345_ACT_TAP_STATUS_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

uint8_t ADXL345::getPowerMode(void)
{
    char tx = ADXL345_BW_RATE_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (rx & 0x10);
}

void ADXL345::setPowerMode(uint8_t mode)
{
    char tx[2];
    tx[0] = ADXL345_BW_RATE_REG;
    
    switch(mode)
    {
        case 0:
            tx[1] = 0x00;
            break;
        case 1:
            tx[1] = 0x10;
            break;
        default:
            tx[1] = 0x00;
    }
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

void ADXL345::setDataRate(uint8_t rate)
{
    char tx[2];
    tx[0] = ADXL345_BW_RATE_REG;
    
    switch(rate)
    {
        case ADXL345_3200HZ:
            tx[1] = ADXL345_3200HZ;
            break;
        case ADXL345_1600HZ:
            tx[1] = ADXL345_1600HZ;
            break;
        case ADXL345_800HZ:
            tx[1] = ADXL345_800HZ;
            break;
        case ADXL345_400HZ:
            tx[1] = ADXL345_400HZ;
            break;
        case ADXL345_200HZ:
            tx[1] = ADXL345_200HZ;
            break;
        case ADXL345_100HZ:
            tx[1] = ADXL345_100HZ;
            break;
        case ADXL345_50HZ:
            tx[1] = ADXL345_50HZ;
            break;
        case ADXL345_25HZ:
            tx[1] = ADXL345_25HZ;
            break;
        case ADXL345_12HZ5:
            tx[1] = ADXL345_12HZ5;
            break;
        case ADXL345_6HZ25:
            tx[1] = ADXL345_6HZ25;
            break;
        default:
            tx[1] = ADXL345_100HZ;
    }
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getPowerControl(void)
{
    char tx = ADXL345_POWER_CTL_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setPowerControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_POWER_CTL_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getInterruptEnableControl(void)
{
    char tx = ADXL345_INT_ENABLE_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
    
}

void ADXL345::setInterruptEnableControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_INT_ENABLE_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getInterruptMappingControl(void)
{
    char tx = ADXL345_INT_MAP_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
    
}

void ADXL345::setInterruptMappingControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_INT_MAP_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);
}

uint8_t ADXL345::getInterruptSource(void)
{
    char tx = ADXL345_INT_SOURCE_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

uint8_t ADXL345::getDataFormatControl(void)
{
    char tx = ADXL345_DATA_FORMAT_REG;
    char rx;
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, &rx, 1);
    
    return (uint8_t) rx;
}

void ADXL345::setDataFormatControl(uint8_t settings)
{
    char tx[2];
    tx[0] = ADXL345_DATA_FORMAT_REG;
    tx[1] = settings;
    
    i2c_.write(ADXL345_I2C_WRITE, tx, 2);   
}

void ADXL345::getRawOutput(int16_t *readings)
{
    char tx = ADXL345_DATAX0_REG;
    char output[6];
    
    i2c_.write(ADXL345_I2C_WRITE, &tx, 1);
    i2c_.read(ADXL345_I2C_READ, output, 6); // read DATAX0-1, DATAY0-1, DATAZ0-1
    
    readings[0] = ((int16_t) output[1] << 8) | output[0];
    readings[1] = ((int16_t) output[3] << 8) | output[2];
    readings[2] = ((int16_t) output[5] << 8) | output[4];
}

AccelG ADXL345::getAccelG(void)
{
    int16_t readings[3] = {-1, -1, -1};
    getRawOutput(readings);
    
    uint8_t format = getDataFormatControl();
    
    /*
     * The range is calculated with the function:
     * 
     * f(x) = 2^(x+2)
     *
     * where x can be:
     * 0 -> +/- 2g
     * 1 -> +/- 4g
     * 2 -> +/- 8g
     * 3 -> +/- 16g
     */
    uint8_t range = 1 << ((format & 0x03) + 2);
    
    float scale;
    if (format & 0x08)
    {
        scale = 0.004;
    } 
    else 
    {   
        scale = range / 1024.0;
    }
    
    AccelG res;
    res.x = readings[0] * scale;
    res.y = readings[1] * scale;
    res.z = readings[2] * scale;
    
    return res;
}
