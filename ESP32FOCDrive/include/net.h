#pragma once

/**
 * WiFi STA (falls back to AP) + OTA. Core 1 only, like cli.h.
 * netSetup() blocks up to 8 s trying STA before falling back to AP.
 */
void netSetup();
/** Call every loop() iteration; pumps ArduinoOTA.handle(). */
void netLoop();
/** True from the OTA onStart callback until the transfer ends or errors. */
bool netOtaActive();
/** Prints the current STA/AP mode, IP and hostname. */
void netPrintInfo();
/** Turns the radio off until the next reboot. */
void netWifiOff();
