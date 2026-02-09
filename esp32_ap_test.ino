#include <WiFi.h>
#include <esp_err.h>
#include <esp_task_wdt.h>

const char* kSsid = "ESP32-Temp-TEST";
const char* kPass = "12345678";

void disableTaskWDT();

void setup() {
  Serial.begin(115200);
  delay(1000);

  disableTaskWDT();

  Serial.println("\n=== ESP32 AP TEST ===");

  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);
  delay(200);

  WiFi.mode(WIFI_OFF);
  delay(200);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  delay(200);

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGW(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  bool ipOk = WiFi.softAPConfig(apIP, apGW, apMask);
  Serial.print("AP IP config: ");
  Serial.println(ipOk ? "OK" : "FALHA");

  bool apOk = WiFi.softAP(kSsid, kPass, 1, false, 4);
  Serial.print("softAP start: ");
  Serial.println(apOk ? "OK" : "FALHA");

  if (apOk) {
    Serial.print("SSID: ");
    Serial.println(kSsid);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

void loop() {
  delay(1000);
}

void disableTaskWDT() {
  esp_err_t err = esp_task_wdt_deinit();
  if (err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_STATE) {
    return;
  }
  if (err != ESP_OK) {
    Serial.print("WDT deinit falhou: ");
    Serial.println(err);
  }
}
