#pragma once



#define WIFI_SSID "ORBI44"

#define WIFI_PASSWORD "blacksun310"

#define LOCAL_UDP_PORT 5000



#define REMOTE_IP "192.168.1.4"

#define REMOTE_PORT 5000

#define LEARN_REMOTE_ENABLED 1



#define SERIAL_BAUD 921600

#define UART_RX_PIN 16

#define UART_TX_PIN 17



#define WIFI_SLEEP_ENABLED 0



// 0 = standalone (no USB). 1 = boot/overflow logs @ 115200 on COM port.
#define DEBUG_USB 1



#define RING_BUF_SIZE 16384

#define UART_HW_RX_BUF_SIZE 4096

#define BRIDGE_CHUNK_SIZE 1200



/** Maximum TM frame accepted from the UART protocol. */
#define TELEMETRY_MAX_FRAME_BYTES 512

/** Complete validated TM frames buffered while UDP is unavailable. */
#define TELEMETRY_FRAME_QUEUE_DEPTH 32



// Batch TM frames per UDP datagram (~500/5 = 100 sendto/s instead of 500).

#define UDP_FRAMES_PER_PACKET 5

#define UDP_FLUSH_MS 4



#define UDP_PACKETS_PER_LOOP 2

/** Robot Telemetry UDP envelope: "RT", version, sequence, frame count, payload length, CRC16. */
#define UDP_ENVELOPE_VERSION 1

/** Re-send READY for as long as the STM32 has not started streaming. */
#define TM_READY_INTERVAL_MS 500

/** Valid-frame silence that indicates an STM32 restart. */
#define TM_RESTART_SILENCE_MS 1500

#define UDP_SOCKET_RESET_FAILS 128



#define BRIDGE_TASK_PRIORITY 12

#define BRIDGE_TASK_CORE 1

