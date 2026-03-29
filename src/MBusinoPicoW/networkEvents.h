// Raspberry Pi Pico W WiFi status polling.
// The ESP32 uses FreeRTOS event callbacks (WiFi.onEvent).
// Pico W (arduino-pico core) has no equivalent, so we poll status each loop.
// Call checkWiFiStatus() from loop().

static uint8_t prevWiFiStatus = WL_IDLE_STATUS;

void checkWiFiStatus() {
  uint8_t status = WiFi.status();
  if (status == prevWiFiStatus) return;

  Serial.printf("[WiFi-status] %d\n", (int)status);
  prevWiFiStatus = status;

  switch (status) {
    case WL_CONNECTED:
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
      break;
    case WL_DISCONNECTED:
    case WL_CONNECTION_LOST:
      Serial.println("WiFi disconnected");
      wifiReconnect = true;
      timerWifiReconnect = millis();
      break;
    default:
      break;
  }
}
