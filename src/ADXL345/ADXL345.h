/**
 * @author Jonathan Whitaker
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * ADXL345, triple axis, digital interface, accelerometer.
 *
 * Datasheet:
 *
 * http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
 */

#ifndef ADXL345_H
#define ADXL345_H

/**
 * Includes
 */
#include "mbed.h"
#include "stdint.h"

/**
 * Defines
 */
#define ADXL345_I2C_ADDRESS 0x53

//----------
// Registers
//----------
#define ADXL345_DEVID_REG            0x00
#define ADXL345_THRESH_TAP_REG       0x1D
#define ADXL345_OFSX_REG             0x1E
#define ADXL345_OFSY_REG             0x1F
#define ADXL345_OFSZ_REG             0x20
#define ADXL345_DUR_REG              0x21
#define ADXL345_LATENT_REG           0x22
#define ADXL345_WINDOW_REG           0x23
#define ADXL345_THRESH_ACT_REG       0x24
#define ADXL345_THRESH_INACT_REG     0x25
#define ADXL345_TIME_INACT_REG       0x26
#define ADXL345_ACT_INACT_CTL_REG    0x27
#define ADXL345_THRESH_FF_REG        0x28
#define ADXL345_TIME_FF_REG          0x29
#define ADXL345_TAP_AXES_REG         0x2A
#define ADXL345_ACT_TAP_STATUS_REG   0x2B
#define ADXL345_BW_RATE_REG          0x2C
#define ADXL345_POWER_CTL_REG        0x2D
#define ADXL345_INT_ENABLE_REG       0x2E
#define ADXL345_INT_MAP_REG          0x2F
#define ADXL345_INT_SOURCE_REG       0x30
#define ADXL345_DATA_FORMAT_REG      0x31
#define ADXL345_DATAX0_REG           0x32
#define ADXL345_DATAX1_REG           0x33
#define ADXL345_DATAY0_REG           0x34
#define ADXL345_DATAY1_REG           0x35
#define ADXL345_DATAZ0_REG           0x36
#define ADXL345_DATAZ1_REG           0x37
#define ADXL345_FIFO_CTL_REG         0x38
#define ADXL345_FIFO_STATUS_REG      0x39

// Data rate codes
#define ADXL345_3200HZ 0x0F
#define ADXL345_1600HZ 0x0E
#define ADXL345_800HZ  0x0D
#define ADXL345_400HZ  0x0C
#define ADXL345_200HZ  0x0B
#define ADXL345_100HZ  0x0A
#define ADXL345_50HZ   0x09
#define ADXL345_25HZ   0x08
#define ADXL345_12HZ5  0x07
#define ADXL345_6HZ25  0x06

// Activity/Inactivity control settings
#define ADXL345_ACTACDC_EN    0x80
#define ADXL345_ACTX_EN       0x40
#define ADXL345_ACTY_EN       0x20
#define ADXL345_ACTZ_EN       0x10
#define ADXL345_INACTACDC_EN  0x08
#define ADXL345_INACTX_EN     0x04
#define ADXL345_INACTY_EN     0x02
#define ADXL345_INACTZ_EN     0x01

// Axis tap settings
#define ADXL345_SUPPRESS_EN   0x08
#define ADXL345_TAPX_EN       0x04
#define ADXL345_TAPY_EN       0x02
#define ADXL345_TAPZ_EN       0x01

#define ADXL345_I2C_WRITE 0xA6
#define ADXL345_I2C_READ  0xA7

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02

typedef struct AccelG {
    
    float x;
    float y;
    float z;
    
} AccelG;

class ADXL345
{
public:
    
    /**
     * ADXL345 Constructor.
     *
     * @param sda - mbed pin to use for SDA line of I2C interface.
     * @param scl - mbed pin to use for SCL line of I2C interface.
     */
    ADXL345(PinName sda, PinName scl);
    
    /**
     * Read the device ID register on the device.
     *
     * @return The device ID code (0xE5).
     */
    uint8_t getDeviceID(void);
    
    /**
     * Read the tap threshold register on the device.
     *
     * @return The tap threshold as an unsigned 8-bit number with a scale factor
     *         of 62.5mg/LSB.
     */
    uint8_t getTapThreshold(void);
    
    /**
     * Set the tap threshold register on the device.
     *
     * @param threshold - The 8-bit tap threshold with a scale factor of 62.4mg/LSB.
     */
    void setTapThreshold(uint8_t threshold);
    
    /**
     * Read the offset register on the device for a particular axis.
     *
     * @param axis - ADXL345_X (0x00) -> X-axis
     *               ADXL345_Y (0x01) -> Y-axis
     *               ADXL345_Z (0x02) -> Z-axis
     *
     * @return The offset as a twos complement 8-bit number with a scale factor of
     *         15.6mg/LSB.
     */
    int8_t getOffset(uint8_t axis);
    
    
    /**
     * Set the offset for a particular axis.
     *
     * @param axis - ADXL345_X (0x00) -> X-axis
     *               ADXL345_Y (0x01) -> Y-axis
     *               ADXL345_Z (0x02) -> Z-axis
     * @param offset - The offset as a twos complement 8-bit number with a scale factor
     *                 of 15.6mg/LSB.
     */
    void setOffset(uint8_t axis, int8_t offset);
    
    /**
     * Read the tap duration register on the device.
     *
     * @return An 8-bit unsigned time value with a 625us/LSB scale factor representing 
     *         the max time that an event must be above the THRESH_TAP threshold to 
     *         qualify as a tap event.
     */
    uint8_t getTapDuration(void);
    
    /**
     * Set the tap duration required to trigger an event.
     *
     * @arg duration_us - The tap duration (in microseconds) as an 8-bit unsigned number 
     *                    with a scale factor of 625us/LSB. A value of 0 disables the 
     *                    tap/double tap functions.
     */
    void setTapDuration(uint8_t duration_us);
    
    /**
     * Read the tap latency register on the device.
     *
     * @return An 8-bit unsigned time value with a 1.25ms/LSB scale factor representing
     *         the wait time from the detection of a tap event to the start of the time
     *         window (defined by the window register).
     */
    uint8_t getTapLatency(void);
    
    /**
     * Set the tap latency between the detection of a tap and the time window.
     *
     * @param delay_ms - The wait time (in milliseconds) from the detection of a 
     *                   tap event to the start of the time window during which a 
     *                   possible second tap event can be detected. A value of 0
     *                   disables the double tap function.
     */
    void setTapLatency(uint8_t delay_ms);
    
    /**
     * Get the time of window between tap latency and a double tap.
     *
     * @return The amount of time (in milliseconds) after the expiration 
     *         of the latency time during which a second valid tap can begin.
     */
    uint8_t getTapWindow(void);
    
    /**
     * Set the time of the window between tap latency and a double tap.
     *
     * @param window_ms The amount of time (in milliseconds) after the 
     *                  expiration of the latency time during which a 
     *                  second valid tap can begin.
     */
    void setTapWindow(uint8_t window_ms);
    
    /**
     * Get the threshold value for detecting activity.
     *
     * @return The threshold value for detecting activity as an 8-bit number.
     *         Scale factor is 62.5mg/LSB.
     */
    uint8_t getActivityThreshold(void);
    
    /**
     * Set the threshold value for detecting activity.
     *
     * @param threshold The threshold value for detecting activity as an 8-bit
     *                  unsigned number. Scale factor is 62.5mg/LSB. A value of 
     *                  0 may result in undesirable behavior if the activity
     *                  interrupt is enabled.
     */
    void setActivityThreshold(uint8_t threshold);
    
    /**
     * Get the threshold value for detecting inactivity.
     *
     * @return The threshold value for detecting inactivity as an 8-bit 
     *         unsigned number. Scale factor is 62.5mg/LSB.
     */
    uint8_t getInactivityThreshold(void);
    
    /**
     * Set the threshold value for detecting inactivity.
     *
     * @param threshold The threshold value for detecting inactivity as an
     *                  8-bit unsigned number. Scale factor is 62.5mg/LSB.
     */
    void setInactivityThreshold(uint8_t threshold);
    
    /**
     * Get the time required for inactivity to be declared.
     *
     * @return The amount of time (in seconds) that acceleration must 
     *         be less than the inactivity threshold for inactivity to 
     *         be declared.
     */
    uint8_t getTimeInactivity(void);
    
    /**
     * Set the time required for inactivity to be declared.
     *
     * @param inactivity The amount of time that acceleration must be less than
     *                   the inactivity threshold for inactivity to be
     *                   declared, in seconds. A value of 0 results in an
     *                   interrupt when the output data is less than the
     *                   threshold inactivity.
     */
    void setTimeInactivity(uint8_t inactivity);
    
    /**
     * Get the activity/inactivity control settings.
     *
     *      D7            D6             D5            D4
     * +-----------+--------------+--------------+--------------+
     * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
     * +-----------+--------------+--------------+--------------+
     *
     *        D3             D2               D1              D0
     * +-------------+----------------+----------------+----------------+
     * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
     * +-------------+----------------+----------------+----------------+
     *
     * See datasheet for details.
     *
     * @return The contents of the ACT_INACT_CTL register.
     */
    uint8_t getActivityInactivityControl(void);
    
    /**
     * Set the activity/inactivity control settings.
     *
     *      D7            D6             D5            D4
     * +-----------+--------------+--------------+--------------+
     * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
     * +-----------+--------------+--------------+--------------+
     *
     *        D3             D2               D1              D0
     * +-------------+----------------+----------------+----------------+
     * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
     * +-------------+----------------+----------------+----------------+
     *
     * See datasheet for details.
     *
     * @param settings - The control byte to write to the ACT_INACT_CTL register.
     */
    void setActivityInactivityControl(uint8_t settings);
    
    /**
     * Get the threshold for free fall detection.
     *
     * @return The threshold value for free-fall detection as an unsigned
     *         8-bit number, with scale factor 62.5mg/LSB.
     */
    uint8_t getFreeFallThreshold(void);
    
    /**
     * Set the threshold for free fall detection.
     *
     * @return The threshold value for free fall detection, as an 8-bit unsigned 
     *         number, with scale factor 62.5mg/LSB. A value of 0 may result in 
     *         undesirable behavior if the free fall interrupt is enabled. Values 
     *         between 300 mg and 600 mg (0x05 to 0x09) are recommended.
     */
    void setFreeFallThreshold(uint8_t threshold);
    
    /**
     * Get the time required to generate a free fall interrupt.
     *
     * @return The minimum time (in milliseconds) that the value of all axes must be less than
     *         the free fall threshold to generate a free fall interrupt.
     */
    uint8_t getFreeFallTime(void);
    
    /**
     * Set the time required to generate a freefall interrupt.
     *
     * @param time_ms - The minimum time (in milliseconds) that the value of all axes
     *                  must be less than the free fall threshold to generate a free
     *                  fall interrupt. A value of 0 may result in undesirable behavior
     *                  if the free fall interrupt is enabled. Values between 100ms and
     *                  350ms are recommended.
     */
    void setFreeFallTime(uint8_t time_ms);
    
    /**
     * Get the axis tap settings.
     *
     *      D3           D2            D1             D0
     * +----------+--------------+--------------+--------------+
     * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
     * +----------+--------------+--------------+--------------+
     *
     * (D7-D4 are 0s).
     *
     * See datasheet for more details.
     *
     * @return The contents of the TAP_AXES register.
     */ 
    uint8_t getTapAxesControl(void);
    
    /**
     * Set the axis tap settings.
     *
     *      D3           D2            D1             D0
     * +----------+--------------+--------------+--------------+
     * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
     * +----------+--------------+--------------+--------------+
     *
     * (D7-D4 are 0s).
     *
     * See datasheet for more details.
     *
     * @param settings - The control settings to write to the TAP_AXES register.
     */
    void setTapAxesControl(uint8_t settings);
    
    /**
     * Get the source of a tap.
     *
     * @return The contents of the ACT_TAP_STATUS register.
     */
    uint8_t getTapSource(void);
    
    /**
     * Get the current power mode.
     *
     * @return 0 -> Normal mode.
     *         1 -> Low power mode.
     */
    uint8_t getPowerMode(void);
    
    /**
     * Set the power mode.
     *
     * @param mode 0 -> Normal mode.
     *             1 -> Low power mode.
     */
    void setPowerMode(uint8_t mode);
    
    /**
     * Set the device bandwidth and output data rate.
     *
     * @param rate - The device bandwidth and output data rate. This param can be one of:
     *               ADXL345_3200HZ
     *               ADXL345_1600HZ
     *               ADXL345_800HZ
     *               ADXL345_400HZ
     *               ADXL345_200HZ
     *               ADXL345_100HZ
     *               ADXL345_50HZ
     *               ADXL345_25HZ
     *               ADXL345_12HZ5 (12.5Hz)
     *               ADXL345_6HZ25 (6.25Hz)
     */
    void setDataRate(uint8_t rate);
    
    /**
     * Get the power control settings.
     *
     * See datasheet for details.
     *
     * @return The contents of the POWER_CTL register.
     */
    uint8_t getPowerControl(void);
    
    /**
     * Set the power control settings.
     *
     * See datasheet for details.
     *
     * @param settings - The control byte to write to the POWER_CTL register.
     */
    void setPowerControl(uint8_t settings);
    
    /**
     * Get the interrupt enable settings.
     *
     * @return The contents of the INT_ENABLE register.
     */
    uint8_t getInterruptEnableControl(void);
    
    /**
     * Set the interrupt enable settings.
     *
     * @param settings - The control byte to write to the INT_ENABLE register.
     */
    void setInterruptEnableControl(uint8_t settings);
    
    /**
     * Get the interrupt mapping settings.
     *
     * @return The contents of the INT_MAP register.
     */
    uint8_t getInterruptMappingControl(void);
    
    /**
     * Set the interrupt mapping settings.
     *
     * @param settings - The control byte to write to the INT_MAP register.
     */
    void setInterruptMappingControl(uint8_t settings);
    
    /**
     * Get the interrupt source.
     *
     * @return The contents of the INT_SOURCE register.
     */
    uint8_t getInterruptSource(void);
    
    /**
     * Get the data format settings.
     *
     * @return The contents of the DATA_FORMAT register.
     */
    uint8_t getDataFormatControl(void);
    
    /**
     * Set the data format settings.
     *
     * The DATA_FORMAT register controls the presentation of data
     * to Register 0x32 through Register 0x37.
     *
     * @param settings - The control byte to write to the DATA_FORMAT register.
     */
    void setDataFormatControl(uint8_t settings);
    
    /**
     * Get the output of all three axes.
     *
     * @param readings - A pointer to a buffer to hold the raw accelerometer values for the
     *        x-axis, y-axis and z-axis [in that order].
     */
    void getRawOutput(int16_t *readings);
    
    /**
     * Get the acceleration in G's according to the data format
     * that is currently set.
     *
     * @return A struct containing the acceleration in the X, Y, and Z coordinates.
     */
    AccelG getAccelG(void);
    
    /**
     * Get the FIFO control settings.
     *
     * @return The contents of the FIFO_CTL register.
     */
    uint8_t getFifoControl(void);
    
    /**
     * Set the FIFO control settings.
     *
     * @param settings - The control byte to write to the FIFO_CTL register.
     */
    void setFifoControl(uint8_t settings);
    
    /**
     * Get FIFO status.
     *
     * @return The contents of the FIFO_STATUS register.
     */
    uint8_t getFifoStatus(void);
    
private:
    
    I2C i2c_;
};

#endif