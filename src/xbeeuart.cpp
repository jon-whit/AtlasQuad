#include "mbed.h"
#include "config.h"
#include "xbeeuart.h"

using namespace XBeeLib;

uint8_t XBeeUART::recent_msg_byte_ = 0;

UARTBasicCallback_t  XBeeUART::stopcallback_ = NULL;
UARTBasicCallback_t  XBeeUART::heartbeatcallback_ = NULL;
UARTMoveCallback_t  XBeeUART::movecallback_ = NULL;
UARTMotorCallback_t XBeeUART::motorcallback_ = NULL;
UARTIMUCallback_t   XBeeUART::imucallback_ = NULL;

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

uint8_t XBeeUART::register_callbacks(UARTBasicCallback_t stop, UARTBasicCallback_t heartbeat, UARTMoveCallback_t move, UARTMotorCallback_t motor, UARTIMUCallback_t imu) {
    stopcallback_ = stop;
    heartbeatcallback_ = heartbeat;
    movecallback_ = move;
    motorcallback_ = motor;
    imucallback_ = imu;

    if(stopcallback_ == NULL || heartbeatcallback_ == NULL || movecallback_ == NULL || motorcallback_ == NULL || imucallback_ == NULL)
        return 1;
    else
        return 0;
}

void XBeeUART::process_frames() {
    xbee_->process_rx_frames();
}

uint8_t XBeeUART::get_message_byte() {
    uint8_t msg = recent_msg_byte_;
    recent_msg_byte_ = 0;
    return msg;
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
    // set as the most recent message, if valid
    if(!broadcast && len > 0 && data[0] != 0) {
        recent_msg_byte_ = data[0];
    }

    // parse the message, and use callbacks if available
    if(!broadcast && len >= 2) {
        // get command from message
        char command[2];
        memcpy(command, data, 2);
        if(!strncmp("SA", command, 2)) {
            stopcallback_();
        } else if(!strncmp("HB", command, 2)) {
            heartbeatcallback_();
        } else if(!strncmp("IR", command, 2)) {
            imucallback_(IMU_RST);
        } else {
            // get value from message
            char value_string[11];
            memcpy(value_string, &data[2], len - 2); // copy characters 2 through len
            uint32_t value = atoi(value_string);
            if(!strncmp("XR", command, 2)) {
                movecallback_(ROT_X, value);
            } else if(!strncmp("YR", command, 2)) {
                movecallback_(ROT_Y, value);
            } else if(!strncmp("ZR", command, 2)) {
                movecallback_(ROT_Z, value);
            } else if(!strncmp("RX", command, 2)) {
                movecallback_(GET_ROT_X, value);
            } else if(!strncmp("RY", command, 2)) {
                movecallback_(GET_ROT_Y, value);
            } else if(!strncmp("RZ", command, 2)) {
                movecallback_(GET_ROT_Z, value);
            } else if(!strncmp("M1", command, 2)) {
                motorcallback_(ESC_1, value);
            } else if(!strncmp("M2", command, 2)) {
                motorcallback_(ESC_2, value);
            } else if(!strncmp("M3", command, 2)) {
                motorcallback_(ESC_3, value);
            } else if(!strncmp("M4", command, 2)) {
                motorcallback_(ESC_4, value);
            } else if(!strncmp("TH", command, 2)) {
                motorcallback_(THROTTLE, value);
            }
        }
    }
}
