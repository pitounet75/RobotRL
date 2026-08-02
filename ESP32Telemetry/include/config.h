#pragma once

// Wi-Fi (station mode) — ESP32 = 2.4 GHz only, WPA2
// Only one active pair; duplicate #define lines silently override the first.
#define WIFI_SSID "ORBI44"
#define WIFI_PASSWORD ""
// UDP port the ESP32 listens on (PC sends subscribe/ping here)
#define LOCAL_UDP_PORT 5000

// Default destination when LEARN_REMOTE_ENABLED=0. Ignored when learn-remote is on.
#define REMOTE_IP "0.0.0.0"
#define REMOTE_PORT 5000

// First incoming UDP packet overrides REMOTE_IP/PORT when enabled.
#define LEARN_REMOTE_ENABLED 1

// UART link to STM32 (UART4 on robot board) — ESP32 Serial2, silkscreen RX2 / TX2
#define SERIAL_BAUD 921600
#define UART_RX_PIN 16 // RX2 <- STM32 PD1 (TX)
#define UART_TX_PIN 17 // TX2 -> STM32 PD0 (RX)

// Low latency: disable WiFi modem sleep (adds ~ tens of ms jitter if left on).
#define WIFI_SLEEP_ENABLED 0

// Optional status on USB serial (UART0). Set 0 in production to avoid USB overhead.
#define DEBUG_USB 1

#define RING_BUF_SIZE 4096
#define UART_HW_RX_BUF_SIZE 2048
#define BRIDGE_CHUNK_SIZE 512
