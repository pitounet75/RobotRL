/**

 * ESP32 transparent UART <-> UDP bridge for STM32 telemetry (TM binary frames).

 */



#include <Arduino.h>

#include <WiFi.h>

#include <esp_wifi.h>



#include <errno.h>

#include <fcntl.h>

#include <netinet/in.h>

#include <sys/socket.h>

#include <unistd.h>



#include "config.h"

#include "ring_buffer.h"



namespace {



constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;

constexpr uint32_t WIFI_RETRY_MS = 5000;



HardwareSerial &mcuSerial = Serial2;



ByteRingBuffer uartToUdp;

ByteRingBuffer udpToUart;



int udp_sock = -1;



IPAddress remoteIp;

uint16_t remotePort = REMOTE_PORT;

bool remoteConfigured = false;

volatile bool g_forwarding_enabled = false;

bool g_forwarding_was_enabled = false;

bool g_uart_capture_enabled = false;

constexpr uint8_t kTmMagic0 = 0x4Du;

constexpr uint8_t kTmMagic1 = 0x54u;

constexpr uint8_t kUdpMagic0 = 0x52u;  // 'R'

constexpr uint8_t kUdpMagic1 = 0x54u;  // 'T'

constexpr size_t kUdpEnvelopeHeaderBytes = 12;

constexpr size_t kUdpEnvelopeCrcBytes = 2;

static_assert(BRIDGE_CHUNK_SIZE >=
                  TELEMETRY_MAX_FRAME_BYTES + kUdpEnvelopeHeaderBytes + kUdpEnvelopeCrcBytes,
              "BRIDGE_CHUNK_SIZE must hold one maximum TM frame plus UDP envelope");

struct TelemetryFrameSlot {
  uint16_t len;
  uint8_t data[TELEMETRY_MAX_FRAME_BYTES];
};

TelemetryFrameSlot frameQueue[TELEMETRY_FRAME_QUEUE_DEPTH];

size_t frameQueueHead = 0;

size_t frameQueueTail = 0;

size_t frameQueueCount = 0;

size_t frameQueueHighWater = 0;

uint32_t frameQueueFirstMs = 0;

uint32_t udpDatagramSequence = 0;

struct BridgeMetrics {
  uint32_t uart_bytes;
  uint32_t uart_fifo_overflow;
  uint32_t uart_buffer_full;
  uint32_t uart_frame_error;
  uint32_t uart_parity_error;
  uint32_t tm_frames_valid;
  uint32_t tm_crc_error;
  uint32_t tm_length_error;
  uint32_t tm_version_error;
  uint32_t tm_resync_bytes;
  uint32_t tm_sequence_gaps;
  uint32_t tm_ready_signals;
  uint32_t tm_restart_detected;
  uint32_t frame_queue_drop_oldest;
  uint32_t udp_attempted;
  uint32_t udp_sent;
  uint32_t udp_retry;
  uint32_t udp_dropped;
};

BridgeMetrics metrics {};

portMUX_TYPE metricsMux = portMUX_INITIALIZER_UNLOCKED;

bool haveLastTmSequence = false;

uint16_t lastTmSequence = 0;

uint32_t lastValidTmFrameMs = 0;

uint32_t lastReadySignalMs = 0;

bool tmHandshakeActive = true;

/* Separate TX/RX buffers — sharing one chunkBuf corrupted pending UDP payloads after sendto() fail. */

uint8_t rxChunkBuf[BRIDGE_CHUNK_SIZE];

uint8_t pendingBuf[BRIDGE_CHUNK_SIZE];

size_t pendingLen = 0;



uint32_t udpBackoffUntilMs = 0;

uint16_t udpConsecutiveFails = 0;



TaskHandle_t bridge_task_handle = nullptr;



#if DEBUG_USB

uint32_t lastOverflowReportMs = 0;

uint32_t udpSendFailCount = 0;



void debugPrint(const char *msg) {
  Serial.println(msg);
}



void reportOverflows() {

  const uint32_t uartDrop = uartToUdp.overflow_count();

  const uint32_t udpDrop = udpToUart.overflow_count();

  const uint32_t udpFail = udpSendFailCount;

  const uint32_t now = millis();

  if (now - lastOverflowReportMs < 5000) {

    return;

  }

  lastOverflowReportMs = now;

  char line[384];

  snprintf(line, sizeof(line),

           "bridge uart=%lu raw_drop=%lu raw_hwm=%u valid=%lu crc=%lu len=%lu ver=%lu "
           "resync=%lu seq_gap=%lu ready=%lu restart=%lu fq=%u/%u fq_drop=%lu "
           "udp=%lu/%lu retry=%lu drop=%lu",

           metrics.uart_bytes, uartDrop, static_cast<unsigned>(uartToUdp.high_water_mark()),
           metrics.tm_frames_valid, metrics.tm_crc_error, metrics.tm_length_error,
           metrics.tm_version_error, metrics.tm_resync_bytes, metrics.tm_sequence_gaps,
           metrics.tm_ready_signals, metrics.tm_restart_detected,
           static_cast<unsigned>(frameQueueCount), static_cast<unsigned>(frameQueueHighWater),
           metrics.frame_queue_drop_oldest, metrics.udp_sent, metrics.udp_attempted,
           metrics.udp_retry, metrics.udp_dropped);

  Serial.println(line);

  uint32_t uart_fifo_overflow = 0;
  uint32_t uart_buffer_full = 0;
  uint32_t uart_frame_error = 0;
  uint32_t uart_parity_error = 0;
  portENTER_CRITICAL(&metricsMux);
  uart_fifo_overflow = metrics.uart_fifo_overflow;
  uart_buffer_full = metrics.uart_buffer_full;
  uart_frame_error = metrics.uart_frame_error;
  uart_parity_error = metrics.uart_parity_error;
  portEXIT_CRITICAL(&metricsMux);

  if (udpDrop != 0 || udpFail != 0 || uart_fifo_overflow != 0 ||
      uart_buffer_full != 0 || uart_frame_error != 0 || uart_parity_error != 0) {
    snprintf(line, sizeof(line),
             "bridge errors udp_rx_drop=%lu send_fail=%lu fifo=%lu hwbuf=%lu frame=%lu parity=%lu",
             udpDrop, udpFail, uart_fifo_overflow, uart_buffer_full,
             uart_frame_error, uart_parity_error);
    Serial.println(line);
  }
}

#else

void debugPrint(const char *) {}

void reportOverflows() {}

#endif



bool remoteIpConfigured(const char *ip_str) {

  return ip_str != nullptr && ip_str[0] != '\0' && strcmp(ip_str, "0.0.0.0") != 0;

}



void onMcuUartRx() {
  if (bridge_task_handle != nullptr) {
    xTaskNotifyGive(bridge_task_handle);
  }
}



void onMcuUartError(hardwareSerial_error_t error) {
  portENTER_CRITICAL(&metricsMux);
  switch (error) {
    case UART_FIFO_OVF_ERROR:
      metrics.uart_fifo_overflow++;
      break;
    case UART_BUFFER_FULL_ERROR:
      metrics.uart_buffer_full++;
      break;
    case UART_FRAME_ERROR:
      metrics.uart_frame_error++;
      break;
    case UART_PARITY_ERROR:
      metrics.uart_parity_error++;
      break;
    default:
      break;
  }
  portEXIT_CRITICAL(&metricsMux);
}



void drainMcuUartTrash() {

  uint8_t junk[128];

  int avail = mcuSerial.available();

  while (avail > 0) {

    const int chunk = avail > static_cast<int>(sizeof(junk)) ? static_cast<int>(sizeof(junk)) : avail;

    const int n = static_cast<int>(mcuSerial.read(junk, static_cast<size_t>(chunk)));

    if (n <= 0) {

      break;

    }

    avail = mcuSerial.available();

  }

}



void setUartRingCapture(bool enabled) {

  if (enabled) {

    mcuSerial.onReceive(onMcuUartRx);

    mcuSerial.onReceiveError(onMcuUartError);

    g_uart_capture_enabled = true;

  } else {

    g_uart_capture_enabled = false;

    mcuSerial.onReceive(nullptr);

    mcuSerial.onReceiveError(nullptr);

    drainMcuUartTrash();

    uartToUdp.discard_all();

    pendingLen = 0;

  }

}



void pollMcuUart();

void processRawTelemetry();

void maintainStm32Handshake();

void beginForwarding(bool send_ready_signal);



void signalStm32Ready() {

  static const char kReady[] = "READY\n";

  mcuSerial.write(reinterpret_cast<const uint8_t *>(kReady), sizeof(kReady) - 1);

  mcuSerial.flush();

  lastReadySignalMs = millis();

  metrics.tm_ready_signals++;

  debugPrint("STM32 ready signal sent");

}



void pollMcuUart() {

  if (g_uart_capture_enabled && mcuSerial.available() > 0) {

    metrics.uart_bytes += static_cast<uint32_t>(uartToUdp.push_from_stream(mcuSerial));

  }

}



uint8_t tmCrc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80u) != 0u ? static_cast<uint8_t>((crc << 1u) ^ 0x07u)
                               : static_cast<uint8_t>(crc << 1u);
    }
  }
  return crc;
}



uint16_t udpCrc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8u;
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0u ? static_cast<uint16_t>((crc << 1u) ^ 0x1021u)
                                 : static_cast<uint16_t>(crc << 1u);
    }
  }
  return crc;
}



void frameQueueDropFront() {
  if (frameQueueCount == 0) {
    return;
  }
  frameQueueTail = (frameQueueTail + 1u) % TELEMETRY_FRAME_QUEUE_DEPTH;
  frameQueueCount--;
}



void frameQueuePush(const uint8_t *data, uint16_t len) {
  if (data == nullptr || len == 0 || len > TELEMETRY_MAX_FRAME_BYTES) {
    return;
  }
  if (frameQueueCount == TELEMETRY_FRAME_QUEUE_DEPTH) {
    frameQueueDropFront();
    metrics.frame_queue_drop_oldest++;
  }
  TelemetryFrameSlot &slot = frameQueue[frameQueueHead];
  slot.len = len;
  memcpy(slot.data, data, len);
  frameQueueHead = (frameQueueHead + 1u) % TELEMETRY_FRAME_QUEUE_DEPTH;
  frameQueueCount++;
  if (frameQueueCount > frameQueueHighWater) {
    frameQueueHighWater = frameQueueCount;
  }
  if (frameQueueCount == 1) {
    frameQueueFirstMs = millis();
  }
}



void processRawTelemetry() {
  uint8_t prefix[4];
  uint8_t frame[TELEMETRY_MAX_FRAME_BYTES];

  while (uartToUdp.available() >= 2) {
    if (uartToUdp.peek_bytes(0, prefix, 2) != 2) {
      return;
    }
    if (prefix[0] != kTmMagic0 || prefix[1] != kTmMagic1) {
      uartToUdp.discard_prefix(1);
      metrics.tm_resync_bytes++;
      continue;
    }
    if (uartToUdp.available() < 4 || uartToUdp.peek_bytes(0, prefix, 4) != 4) {
      return;
    }

    const uint16_t body_len =
        static_cast<uint16_t>(prefix[2]) | (static_cast<uint16_t>(prefix[3]) << 8u);
    const size_t frame_len = 4u + static_cast<size_t>(body_len);
    if (body_len < 7u || frame_len > TELEMETRY_MAX_FRAME_BYTES) {
      uartToUdp.discard_prefix(1);
      metrics.tm_length_error++;
      metrics.tm_resync_bytes++;
      continue;
    }
    if (uartToUdp.available() < frame_len) {
      return;
    }
    if (uartToUdp.peek_bytes(0, frame, frame_len) != frame_len) {
      return;
    }
    if (frame[4] != 1u) {
      uartToUdp.discard_prefix(1);
      metrics.tm_version_error++;
      metrics.tm_resync_bytes++;
      continue;
    }
    if (tmCrc8(frame, frame_len - 1u) != frame[frame_len - 1u]) {
      uartToUdp.discard_prefix(1);
      metrics.tm_crc_error++;
      metrics.tm_resync_bytes++;
      continue;
    }

    uartToUdp.discard_prefix(frame_len);
    const uint16_t sequence =
        static_cast<uint16_t>(frame[5]) | (static_cast<uint16_t>(frame[6]) << 8u);
    if (haveLastTmSequence) {
      const uint16_t delta = static_cast<uint16_t>(sequence - lastTmSequence);
      if (delta > 1u) {
        metrics.tm_sequence_gaps += static_cast<uint32_t>(delta - 1u);
      }
    }
    haveLastTmSequence = true;
    lastTmSequence = sequence;
    metrics.tm_frames_valid++;
    lastValidTmFrameMs = millis();
    tmHandshakeActive = false;
    frameQueuePush(frame, static_cast<uint16_t>(frame_len));
  }
}



void maintainStm32Handshake() {
  const uint32_t now = millis();

  if (!tmHandshakeActive &&
      (now - lastValidTmFrameMs) >= static_cast<uint32_t>(TM_RESTART_SILENCE_MS)) {
    /*
     * Preserve complete queued frames but discard a possible partial frame
     * from the previous STM32 boot before restarting synchronization.
     */
    uartToUdp.discard_all();
    haveLastTmSequence = false;
    tmHandshakeActive = true;
    metrics.tm_restart_detected++;
    debugPrint("STM32 silence detected; restarting handshake");
  }

  if (tmHandshakeActive &&
      (now - lastReadySignalMs) >= static_cast<uint32_t>(TM_READY_INTERVAL_MS)) {
    signalStm32Ready();
  }
}



void configureDefaultRemote() {

#if LEARN_REMOTE_ENABLED

  if (remoteIpConfigured(REMOTE_IP) && remoteIp.fromString(REMOTE_IP)) {

    remotePort = REMOTE_PORT;

    remoteConfigured = true;

    char line[64];

    snprintf(line, sizeof(line), "remote default %s:%u", remoteIp.toString().c_str(), remotePort);

    debugPrint(line);

  } else {

    remoteConfigured = false;

  }

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

#else

  configureDefaultRemote();

#endif

}



void applyWifiLowLatency() {

#if !WIFI_SLEEP_ENABLED

  WiFi.setSleep(false);

  if (WiFi.status() == WL_CONNECTED) {

    esp_wifi_set_ps(WIFI_PS_NONE);

  }

#endif

}



void onWifiEvent(WiFiEvent_t event) {

  if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED || event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {

    applyWifiLowLatency();

  }

}



bool connectWiFi() {

  debugPrint("WiFi: connecting...");

  static bool event_registered = false;
  if (!event_registered) {
    WiFi.onEvent(onWifiEvent);
    event_registered = true;
  }

  if (WiFi.getMode() != WIFI_STA) {

    WiFi.mode(WIFI_STA);

  }

  applyWifiLowLatency();

  /* WiFi.disconnect(true) -> ESP_ERR_WIFI_STOP_STATE (12308) on Arduino-ESP32 3.x */

  if (WiFi.status() == WL_CONNECTED) {

    WiFi.disconnect(false);

    delay(100);

  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED) {

    if (millis() - start > WIFI_CONNECT_TIMEOUT_MS) {

      debugPrint("WiFi: timeout");

      return false;

    }

    pollMcuUart();
    processRawTelemetry();
    maintainStm32Handshake();
    delay(10);

  }



  applyWifiLowLatency();



  char line[80];

  snprintf(line, sizeof(line), "WiFi: %s  IP %s", WIFI_SSID, WiFi.localIP().toString().c_str());

  debugPrint(line);

  return true;

}



void closeUdp() {

  if (udp_sock >= 0) {

    close(udp_sock);

    udp_sock = -1;

  }

}



bool startUdp() {

  closeUdp();



  udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (udp_sock < 0) {

    debugPrint("UDP: socket failed");

    return false;

  }



  int yes = 1;

  setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));



  struct sockaddr_in local {};

  local.sin_family = AF_INET;

  local.sin_port = htons(LOCAL_UDP_PORT);

  local.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(udp_sock, reinterpret_cast<struct sockaddr *>(&local), sizeof(local)) < 0) {

    debugPrint("UDP: bind failed");

    closeUdp();

    return false;

  }



  fcntl(udp_sock, F_SETFL, O_NONBLOCK);



  udpConsecutiveFails = 0;

  udpBackoffUntilMs = 0;

  char line[64];

  snprintf(line, sizeof(line), "UDP listen %u", static_cast<unsigned>(LOCAL_UDP_PORT));

  debugPrint(line);

  return true;

}



void learnRemote(const IPAddress &ip, uint16_t port) {

#if LEARN_REMOTE_ENABLED

  if (remoteConfigured && remoteIp == ip && remotePort == port) {

    return;

  }

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

  if (udp_sock < 0) {

    return;

  }



  struct sockaddr_in from {};

  socklen_t from_len = sizeof(from);

  const int len =

      recvfrom(udp_sock, rxChunkBuf, BRIDGE_CHUNK_SIZE, MSG_DONTWAIT,

               reinterpret_cast<struct sockaddr *>(&from), &from_len);

  if (len <= 0) {

    return;

  }



  learnRemote(IPAddress(from.sin_addr.s_addr), ntohs(from.sin_port));



  const size_t total = static_cast<size_t>(len);

  if (total == 9 && memcmp(rxChunkBuf, "subscribe", 9) == 0) {

    return;

  }



  udpToUart.push(rxChunkBuf, total);

}



void drainRingToUart() {

  const size_t n = udpToUart.pop(rxChunkBuf, BRIDGE_CHUNK_SIZE);

  if (n > 0) {

    mcuSerial.write(rxChunkBuf, n);

  }

}



size_t buildPendingDatagram() {
  if (frameQueueCount == 0 || pendingLen != 0) {
    return 0;
  }

  const bool batch_ready = frameQueueCount >= static_cast<size_t>(UDP_FRAMES_PER_PACKET);
  const bool flush_due = (millis() - frameQueueFirstMs) >= static_cast<uint32_t>(UDP_FLUSH_MS);
  if (!batch_ready && !flush_due) {
    return 0;
  }

  pendingBuf[0] = kUdpMagic0;
  pendingBuf[1] = kUdpMagic1;
  pendingBuf[2] = UDP_ENVELOPE_VERSION;
  pendingBuf[3] = 0;
  const uint32_t sequence = udpDatagramSequence++;
  pendingBuf[4] = static_cast<uint8_t>(sequence & 0xFFu);
  pendingBuf[5] = static_cast<uint8_t>((sequence >> 8u) & 0xFFu);
  pendingBuf[6] = static_cast<uint8_t>((sequence >> 16u) & 0xFFu);
  pendingBuf[7] = static_cast<uint8_t>((sequence >> 24u) & 0xFFu);
  pendingBuf[8] = 0;
  pendingBuf[9] = 0;

  size_t payload_len = 0;
  uint8_t frame_count = 0;
  while (frameQueueCount > 0 &&
         frame_count < static_cast<uint8_t>(UDP_FRAMES_PER_PACKET)) {
    TelemetryFrameSlot &slot = frameQueue[frameQueueTail];
    const size_t datagram_len =
        kUdpEnvelopeHeaderBytes + payload_len + slot.len + kUdpEnvelopeCrcBytes;
    if (datagram_len > sizeof(pendingBuf)) {
      if (frame_count == 0) {
        frameQueueDropFront();
        metrics.udp_dropped++;
        if (frameQueueCount > 0) {
          frameQueueFirstMs = millis();
        }
        continue;
      }
      break;
    }
    memcpy(&pendingBuf[kUdpEnvelopeHeaderBytes + payload_len], slot.data, slot.len);
    payload_len += slot.len;
    frame_count++;
    frameQueueDropFront();
  }

  if (frame_count == 0) {
    return 0;
  }
  pendingBuf[8] = frame_count;
  pendingBuf[10] = static_cast<uint8_t>(payload_len & 0xFFu);
  pendingBuf[11] = static_cast<uint8_t>((payload_len >> 8u) & 0xFFu);
  const size_t without_crc = kUdpEnvelopeHeaderBytes + payload_len;
  const uint16_t crc = udpCrc16(pendingBuf, without_crc);
  pendingBuf[without_crc] = static_cast<uint8_t>(crc & 0xFFu);
  pendingBuf[without_crc + 1u] = static_cast<uint8_t>((crc >> 8u) & 0xFFu);
  pendingLen = without_crc + kUdpEnvelopeCrcBytes;
  if (frameQueueCount > 0) {
    frameQueueFirstMs = millis();
  }
  return pendingLen;
}



bool sendPendingUdp() {

  if (pendingLen == 0 || udp_sock < 0 || !remoteConfigured) {

    return true;

  }



  const uint32_t now = millis();

  if (now < udpBackoffUntilMs) {

    return false;

  }



  struct sockaddr_in dest {};

  dest.sin_family = AF_INET;

  dest.sin_port = htons(remotePort);

  dest.sin_addr.s_addr = static_cast<uint32_t>(remoteIp);



  metrics.udp_attempted++;
  const int sent = sendto(udp_sock, pendingBuf, pendingLen, 0,

                          reinterpret_cast<struct sockaddr *>(&dest), sizeof(dest));

  if (sent < 0) {

    const int err = errno;

    if (err == ENOMEM || err == EAGAIN || err == EWOULDBLOCK) {

#if DEBUG_USB

      ++udpSendFailCount;

#endif

      ++udpConsecutiveFails;

      metrics.udp_retry++;

      const uint32_t backoff_ms =
          udpConsecutiveFails < 50u ? static_cast<uint32_t>(udpConsecutiveFails) : 50u;
      udpBackoffUntilMs = now + backoff_ms;

      if (udpConsecutiveFails >= UDP_SOCKET_RESET_FAILS) {

        debugPrint("UDP: reset socket after send failures");

        startUdp();

      }

      return false;

    }

#if DEBUG_USB

    ++udpSendFailCount;

    char line[64];

    snprintf(line, sizeof(line), "UDP send err %d", err);

    debugPrint(line);

#endif

    metrics.udp_retry++;
    ++udpConsecutiveFails;
    udpBackoffUntilMs = now + 10u;
    if (udpConsecutiveFails >= UDP_SOCKET_RESET_FAILS) {
      debugPrint("UDP: reset socket after persistent error");
      startUdp();
    }
    return false;

  }



  if (static_cast<size_t>(sent) != pendingLen) {

    return false;

  }



  pendingLen = 0;

  metrics.udp_sent++;

  udpConsecutiveFails = 0;

  udpBackoffUntilMs = 0;

  return true;

}



void drainRingToUdp() {

  if (!remoteConfigured || !g_forwarding_enabled) {

    return;

  }



  if (pendingLen > 0) {

    (void)sendPendingUdp();

    if (pendingLen > 0) {

      return;

    }

  }



  for (unsigned i = 0; i < UDP_PACKETS_PER_LOOP; ++i) {

    if (buildPendingDatagram() == 0) {

      return;

    }

    if (!sendPendingUdp()) {

      return;

    }

    vTaskDelay(0);

  }

}



void bridgeTask(void * /*arg*/) {

  for (;;) {

    if (WiFi.status() != WL_CONNECTED) {
      debugPrint("WiFi: reconnect");
      g_forwarding_enabled = false;
      closeUdp();
      configureDefaultRemoteAfterWifiDown();
      while (!connectWiFi()) {
        pollMcuUart();
        processRawTelemetry();
        maintainStm32Handshake();
        vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS));
      }
      while (!startUdp()) {
        pollMcuUart();
        processRawTelemetry();
        maintainStm32Handshake();
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      beginForwarding(false);
    }

    if (!g_forwarding_enabled) {

      if (!g_forwarding_was_enabled) {

        drainMcuUartTrash();

      } else {

        pollMcuUart();

        processRawTelemetry();

      }

      taskYIELD();

      vTaskDelay(pdMS_TO_TICKS(10));

      continue;

    }

    pollMcuUart();

    processRawTelemetry();

    maintainStm32Handshake();

    drainRingToUdp();

    ingestUdpToRing();

    drainRingToUart();

    reportOverflows();

    if (uartToUdp.available() > 0 || frameQueueCount > 0 || pendingLen > 0) {

      taskYIELD();

    } else {

      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

    }

  }

}



void startBridgeTask() {

  if (bridge_task_handle != nullptr) {

    return;

  }

  xTaskCreatePinnedToCore(bridgeTask, "bridge", 8192, nullptr, BRIDGE_TASK_PRIORITY, &bridge_task_handle,

                          BRIDGE_TASK_CORE);

}



void beginForwarding(bool send_ready_signal) {

  if (!remoteConfigured) {

    g_forwarding_enabled = false;

    setUartRingCapture(false);

    return;

  }



  setUartRingCapture(true);



  if (send_ready_signal) {
    tmHandshakeActive = true;
    signalStm32Ready();

  } else {

    pollMcuUart();

    processRawTelemetry();

  }



  processRawTelemetry();



  g_forwarding_enabled = true;

  g_forwarding_was_enabled = true;

  debugPrint("forwarding enabled");

}



}  // namespace



void setup() {

#if DEBUG_USB

  Serial.begin(115200);
  {
    const uint32_t wait_start = millis();
    while (!Serial && (millis() - wait_start) < 1500) {
      delay(10);
    }
  }
  debugPrint("ESP32Telemetry: starting");

#endif



  mcuSerial.setRxBufferSize(UART_HW_RX_BUF_SIZE);

  mcuSerial.begin(SERIAL_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  setUartRingCapture(false);



  configureDefaultRemote();

  setUartRingCapture(true);

  tmHandshakeActive = true;

  signalStm32Ready();

  startBridgeTask();

}



void loop() {

  vTaskDelay(pdMS_TO_TICKS(100));

}


