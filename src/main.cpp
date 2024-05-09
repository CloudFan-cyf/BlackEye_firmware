#include <WiFi.h>
#include "esp_camera.h"
#include <vector>
#include <WebSocketsClient.h>
#include <driver/i2s.h>

const char *ssid = "CloudFan";              // wifi用户名
const char *password = "23333333";          // wifi密码
const IPAddress serverIP(49, 233, 216, 82); // 你自己的公网服务器ip地址
uint16_t serverPort = 80;                   // 服务器端口号(tcp协议)

#define maxcache 1430

WebSocketsClient webSocket; // 声明一个客户端对象，用于与服务器进行连接

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

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,

    .jpeg_quality = 16,
    .fb_count = 1,

    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};
void wifi_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // 关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
}
//摄像头启动
esp_err_t camera_init() {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        return err;
    }
    Serial.println("Camera Init OK!");
    return ESP_OK;
}
//摄像头休眠
void camera_deinit() {
    esp_camera_deinit();
    Serial.println("Camera Deinitialized");
}

// WebSocket event handling
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WebSocket] Disconnected!");
            break;
        case WStype_CONNECTED:
            Serial.println("[WebSocket] Connected!");
            webSocket.sendTXT("Hello, server!"); // Greeting message to server
            break;
        case WStype_TEXT:
            Serial.printf("[WebSocket] Received text: %s\n", payload);
            if (strcmp((char *)payload, "wake") == 0) { // Check if the message is a wake command
                Serial.println("Waking up camera...");
                camera_init(); // Initialize camera
            } else if (strcmp((char *)payload, "sleep") == 0) { // Check if the message is a sleep command
                Serial.println("Putting camera to sleep...");
                camera_deinit(); // Deinitialize camera
            }
            break;
        case WStype_BIN:
            Serial.printf("[WebSocket] Received binary data\n");
            break;
        case WStype_ERROR:
            Serial.println("[WebSocket] Error occurred!");
            break;
        case WStype_PING:
        case WStype_PONG:
            // Handle ping/pong packets
            break;
    }
}



void setup() {
    Serial.begin(9600);
    Serial.println("Starting Camera Client!");
    wifi_init();
    //不在setup中初始化摄像头，等待wake信号来初始化
    // camera_init();
    webSocket.begin("49.233.216.82", 5901, "/esp32");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
}

void loop() {
    webSocket.loop();

    if (webSocket.isConnected()) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            webSocket.sendBIN(fb->buf, fb->len);
            esp_camera_fb_return(fb);
        }
    }
}
