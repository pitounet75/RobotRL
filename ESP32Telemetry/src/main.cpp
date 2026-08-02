/**
 * ESP32 transparent UART <-> UDP bridge for STM32 telemetry (TM binary frames).
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

void debugWiFiStatus() {
  const wl_status_t st = WiFi.status();
  char line[96];
  snprintf(line, sizeof(line), "WiFi status=%d (1=SSID missing 4=FAIL 6=DISCONNECTED)", static_cast<int>(st));
  debugPrint(line);
}

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
  snprintf(line, sizeof(line), "overflow uart->udp %lu  udp->uart %lu", uartDrop, udpDrop);
  Serial.println(line);
}
#else
void debugPrint(const char *) {}
void reportOverflows() {}
#endif

void onMcuUartRx() { uartToUdp.push_from_stream(mcuSerial); }

void pollMcuUart() {
  /* onReceive can miss bytes during WiFi work — always drain hardware FIFO in loop(). */
  if (mcuSerial.available() > 0) {
    uartToUdp.push_from_stream(mcuSerial);
  }
}

void configureDefaultRemote() {
#if LEARN_REMOTE_ENABLED
  remoteConfigured = false;
#else
  if (!remoteIp.fromString(REMOTE_IP)) {
    remoteConfigured = false;
    return;
  }
  remotePort = REMOTE_PORT;
  remoteConfigured = true;
#endif
}

void configureDefaultRemoteAfterWifiDown() {
#if LEARN_REMOTE_ENABLED
  /* Keep last learned PC address across WiFi glitches. */
#else
  configureDefaultRemote();
#endif
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
#if !WIFI_SLEEP_ENABLED
  WiFi.setSleep(false);
#endif
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  debugPrint("WiFi: connecting...");
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {
      debugPrint("WiFi: timeout");
      debugWiFiStatus();
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
  snprintf(line, sizeof(line), "UDP listen %u", static_cast<unsigned>(LOCAL_UDP_PORT));
  debugPrint(line);
  return true;
}

void learnRemote(const IPAddress &ip, uint16_t port) {
#if LEARN_REMOTE_ENABLED
  remoteIp = ip;
  remotePort = port;
  remoteConfigured = true;
  char line[64];
  snprintf(line, sizeof(line), "remote learned %s:%u", remoteIp.toString().c_str(), remotePort);
  debugPrint(line);
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
  size_t total = 0;
  while (remaining > 0 && total < BRIDGE_CHUNK_SIZE) {
    const int toRead = remaining > static_cast<int>(BRIDGE_CHUNK_SIZE - total)
                           ? static_cast<int>(BRIDGE_CHUNK_SIZE - total)
                           : remaining;
    const int n = udp.read(chunkBuf + total, toRead);
    if (n <= 0) {
      break;
    }
    total += static_cast<size_t>(n);
    remaining -= n;
  }

  /* Learn-remote ping only — must not hit STM32 UART (corrupts TM frame parser). */
  if (total == 9 && memcmp(chunkBuf, "subscribe", 9) == 0) {
    return;
  }

  if (total > 0) {
    udpToUart.push(chunkBuf, total);
  }
}

void drainRingToUart() {
  const size_t n = udpToUart.pop(chunkBuf, BRIDGE_CHUNK_SIZE);
  if (n > 0) {
    mcuSerial.write(chunkBuf, n);
  }
}

void drainRingToUdp() {
  static size_t pendingLen = 0;

  if (!remoteConfigured) {
    /* Buffer UART until PC subscribe — do not discard (old behavior caused burst gaps). */
    return;
  }

  if (pendingLen == 0) {
    pendingLen = uartToUdp.pop(chunkBuf, BRIDGE_CHUNK_SIZE);
    if (pendingLen == 0) {
      return;
    }
  }

  if (!udp.beginPacket(remoteIp, remotePort)) {
    return;
  }
  udp.write(chunkBuf, pendingLen);
  if (udp.endPacket()) {
    pendingLen = 0;
    /* Send more if the ring still has data. */
    while ((pendingLen = uartToUdp.pop(chunkBuf, BRIDGE_CHUNK_SIZE)) > 0) {
      if (!udp.beginPacket(remoteIp, remotePort)) {
        return;
      }
      udp.write(chunkBuf, pendingLen);
      if (!udp.endPacket()) {
        return;
      }
      pendingLen = 0;
    }
  }
}

}  // namespace

void setup() {
#if DEBUG_USB
  Serial.begin(115200);
  delay(500);
  debugPrint("ESP32Telemetry: starting");
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
    debugPrint("WiFi: reconnect");
    udp.stop();
    uartToUdp.discard_all();
    udpToUart.discard_all();
    while (!connectWiFi()) {
      delay(WIFI_RETRY_MS);
    }
    while (!startUdp()) {
      delay(1000);
    }
    configureDefaultRemoteAfterWifiDown();
  }

  pollMcuUart();
  ingestUdpToRing();
  drainRingToUart();
  drainRingToUdp();
  reportOverflows();
  // No delay(): keep loop tight so UART->UDP forwarding stays low-latency.
  yield();
}
