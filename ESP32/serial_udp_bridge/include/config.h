#pragma once

// Wi-Fi (station mode — ESP32 joins your router/AP)
#define WIFI_SSID "YourSSID"
#define WIFI_PASSWORD "YourPassword"

// UDP port the ESP32 listens on (PC / host sends commands here)
#define LOCAL_UDP_PORT 5000

// Default destination for STM32 -> WiFi traffic.
// If LEARN_REMOTE_ENABLED is 1, the first incoming UDP packet overrides this.
#define REMOTE_IP "192.168.1.100"
#define REMOTE_PORT 5000

// When 1, remember the IP:port of the last host that sent a UDP packet
// and send serial data back to that endpoint (handy when the PC IP changes).
#define LEARN_REMOTE_ENABLED 1

// UART link to STM32 (not USB — use a second hardware UART)
#define SERIAL_BAUD 115200
#define UART_RX_PIN 16 // ESP32 RX  <- STM32 TX
#define UART_TX_PIN 17 // ESP32 TX  -> STM32 RX

// Optional status on USB serial (UART0) at 115200 — set 0 to disable prints
#define DEBUG_USB 1

// Software ring buffers (power of 2). Decouple UART bursts from WiFi/UDP work.
#define RING_BUF_SIZE 4096

// ESP32 driver RX FIFO before the ring (bytes)
#define UART_HW_RX_BUF_SIZE 2048

// Max bytes moved per loop iteration (UDP payload chunk)
#define BRIDGE_CHUNK_SIZE 512
