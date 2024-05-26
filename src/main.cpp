#include "esp_camera.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#define CAMERA_MODEL_AI_THINKER // Ima PSRAM
#include "camera_pins.h"
#include "esp_log.h"
#include <esp32-hal-psram.h>
#include <HardwareSerial.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
// Konfiguracija WiFi povezave
const char *ssid = "ESP32_AP";
const char *password = "12345678";
const char *server_ip = "192.168.4.1"; // Nadomestni z pravilnim IP naslovom
const int server_port = 81;

// WebSocketsClient objekt
WebSocketsClient webSocket;
// Funkcija za obdelavo WebSocket dogodkov
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
bool CAPTURE = false;
void setup()
{
  // Vklopimo UART
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Povezujem se na WiFi...");
  }
  Serial.println("Povezan na WiFi");

  // spremenljivka za začetno configuracijo kamere
  camera_config_t config;
  // Potrebne nastavitve na camera_config_t structu
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Če ima ESP32 MCU PSRAM potem lahko shranimo bolšjo kvaliteto
  if (psramFound())
  {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  // Inicializacija kamere
  esp_err_t err = esp_camera_init(&config);

  //  Nastavitve senzorja
  sensor_t *s = esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // vertikalno flipanje
    s->set_brightness(s, 1);  // nastavimo svetlost
    s->set_saturation(s, -2); // zmanjšamo saturacijo
  }

  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  webSocket.begin(server_ip, server_port, "/");
  webSocket.onEvent(webSocketEvent);
}

void loop()
{
  webSocket.loop();
  if (Serial.available() > 0)
  {
    String data = Serial.readString();
    if (data == "START\n")
    {
      CAPTURE = true;
    }
  }
  if (CAPTURE)
  {
    if (webSocket.isConnected())
    {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb)
      {
        Serial.println("Neuspešno zajemanje slike");
        return;
      }

      webSocket.sendBIN(fb->buf, fb->len);

      esp_camera_fb_return(fb);
    }
    CAPTURE = false;
  }
  delay(1);
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    Serial.println("Connected to server");
    break;
  case WStype_DISCONNECTED:
    Serial.println("Disconnected from server");
    break;
  case WStype_BIN:
    // Handle binary data received from server if needed
    break;
  default:
    break;
  }
}