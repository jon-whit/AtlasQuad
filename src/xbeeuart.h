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
     * Get most recent message (first byte, anyway) received by the radio.
     * Note that this specifically ignores broadcast packets, just in case there is an
     * unknown device spamming the channel with bogus packets.
     *
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
    XBeeLib::XBee802 xbee_;
    XBeeLib::RemoteXBee802 remote_device_;
    static void receive_cb_(const XBeeLib::RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len);
    uint8_t recent_msg_byte_;
};

#endif
