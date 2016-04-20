/*
 * This class manages XBee UART communications between a base station computer and the quadcopter.
 * Mostly just a wrapper for the XBeeLib library, to consolidate code that handles communication.
 * It accepts messages from the base station, and makes sure the connection is alive and
 * messages are valid.
 */

#ifndef XBEEUART_H
#define XBEEUART_H

/**
 * Includes
 */
#include "mbed.h"
#include "stdint.h"
#include "XBeeLib.h"

/**
 * Motor/movement mode definitions - should this be moved to config.h?
 *  These values could then be used elsewhere
 */
#define ROT_X     0
#define ROT_Y     1
#define ROT_Z     2
#define GET_ROT_X 3
#define GET_ROT_Y 4
#define GET_ROT_Z 5
#define ESC_1     1
#define ESC_2     2
#define ESC_3     3
#define ESC_4     4
#define THROTTLE  5
#define UART_KP   0
#define UART_KI   1
#define UART_KD   2
#define ESC_AUTO  6
#define IMU_RST   0

typedef void     (*UARTBasicCallback_t)();
typedef void     (*UARTMoveCallback_t)(uint8_t, float);
typedef void     (*UARTMotorCallback_t)(uint8_t, uint16_t);
typedef void     (*UARTPIDCallback_t)(uint8_t, float);

class XBeeUART
{
public:

    /**
     * XBee UART constructor.
     *
     */
    XBeeUART();
    XBeeUART(uint16_t remote_address);

    /**
     * Initialize XBee module
     */
    void init();

    /**
     * Register callbacks to be used whenever a message is received. The callbacks are run
     * as soon as a message is received, depending on the message content and/or command.
     */
    uint8_t register_callbacks(UARTBasicCallback_t stop, UARTBasicCallback_t heartbeat, UARTMoveCallback_t move, UARTMotorCallback_t motor, UARTPIDCallback_t pid);

    /**
     * Get most recent message (first byte, anyway) received by the radio.
     * Note that this specifically ignores broadcast packets, just in case there is an
     * unknown device spamming the channel with bogus packets.
     *
     * Once this function is called, the most recent byte gets reset back to zero (until
     * a new message arrives)
     */
    uint8_t get_message_byte();

    /**
     * Broadcast a message on the XBee radio network.
     * This will return a value - zero if the transmission was successful, other values for failure
     */
    uint8_t broadcast_data(uint8_t data[]);

    /**
     * Send a message to a specific remote XBee radio. Note that this requires the XBeeUART object
     * to be set up with a remote radio address. send_data() will always send to this remote address.
     * This will return a value - zero if the transmission was successful, other values for failure
     */
    uint8_t send_data(uint8_t data[]);

    /**
     * Process received XBee frames. This must be called periodically, though
     * it does not return anything.
     */
    void process_frames();

private:
    XBeeLib::XBee802 *xbee_;
    XBeeLib::RemoteXBee802 *remote_device_;
    static void receive_cb_(const XBeeLib::RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len);
    static uint8_t recent_msg_byte_;
    static UARTBasicCallback_t  stopcallback_;
    static UARTBasicCallback_t  heartbeatcallback_;
    static UARTMoveCallback_t  movecallback_;
    static UARTMotorCallback_t motorcallback_;
    static UARTPIDCallback_t   pidcallback_;
};

#endif
