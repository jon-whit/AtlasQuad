#include "mbed.h"
#include "config.h"
#include "xbeeuart.h"

using namespace XBeeLib;

uint8_t recent_msg_byte_ = 0;

XBeeUART::XBeeUART() {
    remote_device_ = NULL;
}

XBeeUART::XBeeUART(uint16_t remote_address) {
    remote_device_ = new RemoteXBee802(remote_address);
}

void XBeeUART::init() {
    xbee_ = new XBee802(RADIO_TX, RADIO_RX, RADIO_RESET, RADIO_RTS, RADIO_CTS, 9600); // constants in config.h
    xbee_->register_receive_cb(&receive_cb_);
    xbee_->init();
}

void XBeeUART::process_frames() {
    xbee_->process_rx_frames();
}

uint8_t XBeeUART::get_message_byte() {
    return recent_msg_byte_;
}

uint8_t XBeeUART::broadcast_data(uint8_t data[]) {
    const uint16_t data_len = sizeof(data) / sizeof(*data) - 1; // get data length by checking pointers
    TxStatus txStatus = xbee_->send_data_broadcast(data, data_len);
    if(txStatus != TxStatusSuccess) {
        return (uint8_t) txStatus;
    } else {
        return 0;
    }
}

uint8_t XBeeUART::send_data(uint8_t data[]) {
    // check for remote_device
    if(remote_device_ == NULL) {
        return 254; // remote device was not defined
    }

    const uint16_t data_len = sizeof(data) / sizeof(*data) - 1; // get data length by checking pointers
    TxStatus txStatus = xbee_->send_data(*remote_device_, data, data_len);
    if(txStatus != TxStatusSuccess) {
        // this should probably handle the error and give more information
        return (uint8_t) txStatus;
    } else {
        return 0;
    }
}

void XBeeUART::receive_cb_(const RemoteXBee802& remote, bool broadcast, const uint8_t *const data, uint16_t len) {
    // process data - future implementation
    


    // set as the most recent message, if valid
    if(!broadcast && len > 0 && data[0] != 0) {
        recent_msg_byte_ = data[0];
    }
}
