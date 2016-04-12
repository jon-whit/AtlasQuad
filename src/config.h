// enable logging through the DigiLogger
//#define ENABLE_LOGGING

// internal circular buffer; currently an arbitrary value
#define FRAME_BUFFER_SIZE 15

// limit max frame length
#define MAX_FRAME_PAYLOAD_LEN 128

// set default timeout in milliseconds
#define SYNC_OPS_TIMEOUT_MS 2000

// XBee pin definitions
#define RADIO_TX    PC_4
#define RADIO_RX    PC_5
#define RADIO_RESET PC_6
#define RADIO_RTS   PB_1
#define RADIO_CTS   PB_13

// IMU pin definitions
#define IMU_SDA          PB_9
#define IMU_SCL          PB_8
#define IMU_SAMPLE_TIME  10000 // in us (10ms)

// ESC pin definitions
#define ESC1_PIN    D4
#define ESC2_PIN    D5
#define ESC3_PIN    D8
#define ESC4_PIN    D7
