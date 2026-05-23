/**
 * ESP32 transparent serial <-> UDP bridge.
 *
 * STM32 UART  <->  ESP32 UART2  <->  WiFi UDP
 *
 * Ring buffers on both paths so UART RX (ISR/callback) is not tied to UDP send latency.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "config.h"
#include "ring_buffer.h"

namespace {

constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
constexpr uint32_t WIFI_RETRY_MS = 5000;

WiFiUDP udp;
HardwareSerial &mcuSerial = Serial2;

ByteRingBuffer uartToUdp;
ByteRingBuffer udpToUart;

IPAddress remoteIp;
uint16_t remotePort = REMOTE_PORT;
bool remoteConfigured = false;

uint8_t chunkBuf[BRIDGE_CHUNK_SIZE];

#if DEBUG_USB
uint32_t lastOverflowReportMs = 0;

void debugPrint(const char *msg) { Serial.println(msg); }

void reportOverflows() {
  const uint32_t uartDrop = uartToUdp.overflow_count();
  const uint32_t udpDrop = udpToUart.overflow_count();
  if (uartDrop == 0 && udpDrop == 0) {
    return;
  }
  const uint32_t now = millis();
  if (now - lastOverflowReportMs < 2000) {
    return;
  }
  lastOverflowReportMs = now;
  char line[80];
  snprintf(line, sizeof(line), "ring overflow: uart->udp %lu  udp->uart %lu", uartDrop, udpDrop);
  Serial.println(line);
}
#else
void debugPrint(const char *) {}
void reportOverflows() {}
#endif

void onMcuUartRx() { uartToUdp.push_from_stream(mcuSerial); }

void configureDefaultRemote() {
  if (!remoteIp.fromString(REMOTE_IP)) {
    remoteConfigured = false;
    return;
  }
  remotePort = REMOTE_PORT;
  remoteConfigured = true;
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  debugPrint("WiFi: connecting...");
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
      debugPrint("WiFi: timeout");
      return false;
    }
    delay(250);
  }

  char line[80];
  snprintf(line, sizeof(line), "WiFi: %s  IP %s", WIFI_SSID, WiFi.localIP().toString().c_str());
  debugPrint(line);
  return true;
}

bool startUdp() {
  if (!udp.begin(LOCAL_UDP_PORT)) {
    debugPrint("UDP: bind failed");
    return false;
  }
  char line[64];
  snprintf(line, sizeof(line), "UDP: listen %u  ring %u B", static_cast<unsigned>(LOCAL_UDP_PORT),
           static_cast<unsigned>(RING_BUF_SIZE));
  debugPrint(line);
  return true;
}

void learnRemote(const IPAddress &ip, uint16_t port) {
#if LEARN_REMOTE_ENABLED
  remoteIp = ip;
  remotePort = port;
  remoteConfigured = true;
#else
  (void)ip;
  (void)port;
#endif
}

void ingestUdpToRing() {
  const int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  learnRemote(udp.remoteIP(), udp.remotePort());

  int remaining = packetSize;
  while (remaining > 0) {
    const int toRead =
        remaining > static_cast<int>(BRIDGE_CHUNK_SIZE) ? static_cast<int>(BRIDGE_CHUNK_SIZE) : remaining;
    const int n = udp.read(chunkBuf, toRead);
    if (n <= 0) {
      break;
    }
    udpToUart.push(chunkBuf, static_cast<size_t>(n));
    remaining -= n;
  }
}

void drainRingToUart() {
  const size_t n = udpToUart.pop(chunkBuf, BRIDGE_CHUNK_SIZE);
  if (n > 0) {
    mcuSerial.write(chunkBuf, n);
  }
}

void drainRingToUdp() {
  if (!remoteConfigured) {
    uartToUdp.discard_all();
    return;
  }

  size_t n = uartToUdp.pop(chunkBuf, BRIDGE_CHUNK_SIZE);
  while (n > 0) {
    udp.beginPacket(remoteIp, remotePort);
    udp.write(chunkBuf, n);
    udp.endPacket();
    n = uartToUdp.pop(chunkBuf, BRIDGE_CHUNK_SIZE);
  }
}

}  // namespace

void setup() {
#if DEBUG_USB
  Serial.begin(115200);
  delay(500);
  debugPrint("serial_udp_bridge: starting");
#endif

  mcuSerial.setRxBufferSize(UART_HW_RX_BUF_SIZE);
  mcuSerial.begin(SERIAL_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  mcuSerial.onReceive(onMcuUartRx);

  configureDefaultRemote();

  while (!connectWiFi()) {
    delay(WIFI_RETRY_MS);
  }
  while (!startUdp()) {
    delay(1000);
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    debugPrint("WiFi: lost — reconnecting");
    udp.stop();
    uartToUdp.discard_all();
    udpToUart.discard_all();
    while (!connectWiFi()) {
      delay(WIFI_RETRY_MS);
    }
    while (!startUdp()) {
      delay(1000);
    }
    configureDefaultRemote();
  }

  ingestUdpToRing();
  drainRingToUart();
  drainRingToUdp();
  reportOverflows();
  yield();
}
