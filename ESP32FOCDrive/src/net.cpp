/**
 * WiFi STA/AP fallback + OTA. Ported from
 * ESP32FOCHardwareCheck/src/main.cpp:187.
 */

#include "net.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "config.h"

/* Temporary seam, provided by main.cpp (task 2). At this position in the
 * execution order (task 8 runs third, right after the CLI) there is no FOC
 * task yet, so onStart only parks the motors via mainIdle; task 4 adds the
 * FOC timer pause and task 7 replaces this with the driveStop loop. */
void mainIdle(uint8_t axis_mask);

namespace {
bool s_ota_active = false;

void printOta() {
  /* The real mode drives the message, not a default fallthrough: after
   * netWifiOff() the mode is WIFI_MODE_NULL, and falling into the AP
   * branch would advertise an AP that no longer broadcasts, at 0.0.0.0. */
  const wifi_mode_t mode = WiFi.getMode();
  if (mode == WIFI_MODE_NULL) {
    Serial.println("ota: wifi off (reboot to restore OTA)");
    return;
  }
  if (mode == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED) {
    Serial.printf("ota: STA %s  ip=%s  host=%s\n", WIFI_SSID,
                  WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
    return;
  }
  Serial.printf("ota: AP %s  ip=%s  host=%s  (join this SSID, upload 192.168.4.1)\n",
                OTA_AP_SSID, WiFi.softAPIP().toString().c_str(), OTA_HOSTNAME);
}
}  // namespace

void netSetup() {
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  bool sta_ok = false;
  if (WIFI_SSID[0] != '\0') {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    const uint32_t t0 = millis();
    while (millis() - t0 < 8000u) {
      if (WiFi.status() == WL_CONNECTED) {
        sta_ok = true;
        break;
      }
      delay(100);
    }
  }
  if (!sta_ok) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_AP_SSID);
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setTimeout(60000);
  ArduinoOTA.onStart([]() {
    s_ota_active = true;
    mainIdle(0b11); /* task 7 swaps this for the driveStop loop */
    disableCore1WDT();
    Serial.println("ota: start - motors off");
    Serial.flush();
  });
  ArduinoOTA.onError([](ota_error_t err) {
    s_ota_active = false;
    /* An OTA failure returns control to normal operation, motor.move() and
     * cliPoll() included, so the safety net has to come back with it. */
    enableCore1WDT();
    Serial.printf("ota: err %u\n", (unsigned)err);
  });
  ArduinoOTA.begin();
  printOta();
}

void netLoop() { ArduinoOTA.handle(); }

bool netOtaActive() { return s_ota_active; }

void netPrintInfo() { printOta(); }

void netWifiOff() {
  WiFi.mode(WIFI_OFF);
  Serial.println("wifioff: WiFi off (reboot to restore OTA)");
}
