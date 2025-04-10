#include "esp_camera.h"
#include <WiFi.h>
#include "time.h"
#include <WebServer.h>

#define CAMERA_MODEL_AI_THINKER

// Replace with your WiFi credentials
const char* ssid = "Alyssa wifi";
const char* password = "0826788787";

// NTP configuration
//const char* ntpServer = "pool.ntp.org";
const char* ntpServer = "time.windows.com";
const long gmtOffset_sec = 2 * 3600;    // GMT+2
const int daylightOffset_sec = 0;       // No DST in South Africa


// AI Thinker camera pin definition
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//WiFiServer server(80);
WebServer server(80);


void startCameraServer() {
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  Serial.println("Camera server started");
}

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Set resolution here (e.g., FRAMESIZE_QVGA = 320x240)
  config.frame_size = FRAMESIZE_QVGA;
  config.fb_count = 1;
  config.jpeg_quality = 12;

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Let serial come online

  // ---- FIRST WiFi ----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  
  // ---- THEN NTP ----
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println(&timeinfo, "Time synced: %Y-%m-%d %H:%M:%S");
  } else {
    Serial.println("Failed to obtain time");
  }

  // ---- THEN CAMERA ----
  startCamera();
  startCameraServer();


}


void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  // Get current time with milliseconds
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  struct tm* timeinfo = localtime(&tv_now.tv_sec);

  char timestamp[40];
  snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02d.%03ld",
           timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
           timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
           tv_now.tv_usec / 1000);

  // Set headers and send image
  //server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.sendHeader("X-Camera-Timestamp", timestamp);
  server.send_P(200, "image/jpeg", (char*)fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void loop() {
  server.handleClient();

/*  WiFiClient client = server.available();
  if (!client) return;

  // Wait for HTTP request
  while (client.connected()) {
    if (client.available()) {
      String req = client.readStringUntil('\r');
      client.readStringUntil('\n');
      if (req.indexOf("GET /capture") >= 0) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
          Serial.println("Camera capture failed");
          client.stop();
          return;
        }

        // Get current time
        struct tm timeinfo;
        getLocalTime(&timeinfo);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

        // Send headers
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: image/jpeg");
        client.println("X-Camera-Timestamp: " + String(timestamp));
        client.println("Content-Length: " + String(fb->len));
        client.println("Connection: close");
        client.println();

        // Send image
        client.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        break;
      }
    }
  }

  delay(10);
  client.stop();
  */
}
