#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <vector>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include <MPU6050_6Axis_MotionApps20.h>

const char *ssid = "CloudFan";              // wifi用户名
const char *password = "23333333";          // wifi密码
const IPAddress serverIP(49, 233, 216, 82); // 你自己的公网服务器ip地址
uint16_t serverPort = 80;                   // 服务器端口号(tcp协议)

#define maxcache 1430

#define DATA_TYPE_VIDEO 0x01
#define DATA_TYPE_BATTERY 0x02
#define DATA_TYPE_IMU 0x03
struct DataPacket
{
    uint8_t dataType;
    std::vector<uint8_t> data;
};


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

#define BATTERY_PIN       33 // 假设电池电量检测连接到 GPIO 33 (ADC)


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

    .jpeg_quality = 10,
    .fb_count = 1,

    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

//定义舵机引脚
#define SERVO_PIN_H 13 // 水平舵机
#define SERVO_PIN_V 12 // 垂直舵机

// 定义舵机
Servo servoH; // 水平舵机对象
Servo servoV; // 垂直舵机对象

//定义I2C引脚
#define I2C_SDA 14
#define I2C_SCL 15
#define INPUT_PIN 2 // 接到MPU6050的INT引脚
//定义MPU6050
MPU6050 mpu(0x68); // 定义MPU6050对象，连接到地址为0x68的I2C总线

// MPU6050中断引脚和标志
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}
#define FIFO_BUFFER_SIZE 64 // 根据需要设置合适的大小
uint8_t fifoBuffer[FIFO_BUFFER_SIZE];
// IMU数据变量
int16_t ax, ay, az;
int16_t gx, gy, gz;

// 初始化舵机
void servo_init() {
    servoH.attach(SERVO_PIN_H);
    servoV.attach(SERVO_PIN_V);
    servoH.write(90); // 初始位置设置为90度
    servoV.write(90); // 初始位置设置为90度
    Serial.println("Servo Init OK!");
}

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

// 获取电池电量
float getBatteryLevel() {
    int adcValue = analogRead(BATTERY_PIN);
    float voltage = adcValue * (3.3 / 4096.0) * 2; // 假设使用的是3.3V参考电压，分压系数为2
    return voltage;
}

// 解析并处理控制命令
void processControlCommand(const char* payload) {
    float h_percent, v_percent;
    float h_angle, v_angle;
    // 解析控制信号中的角度
    if (sscanf(payload, "H:%f V:%f", &h_percent, &v_percent) == 2) {
        Serial.printf("Received control command: H:%f, V:%f\n", h_percent, v_percent);
        // 控制舵机转动
        h_angle = h_percent*180/100-90; // 映射到舵机控制范围
        v_angle = v_percent*180/100-90; // 映射到舵机控制范围
        if (h_angle >= -90 && h_angle <= 90) {
            servoH.write(h_angle + 90); // 将角度转换为舵机控制信号
        }
        if (v_angle >= -90 && v_angle <= 90) {
            servoV.write(v_angle + 90); // 将角度转换为舵机控制信号
        }
        Serial.printf("Controlled to H: %f, V: %f\n", h_angle, v_angle);
    }
    else {
        Serial.printf("Invalid control command: %s\n", payload);
    }
}


// WebSocket事件处理
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("[WebSocket] Disconnected!");
            break;
        case WStype_CONNECTED:
            Serial.println("[WebSocket] Connected!");
            //webSocket.sendTXT("Hello, server!"); // 向服务器发送问候消息
            break;
        case WStype_TEXT:
            Serial.printf("[WebSocket] Received text: %s\n", payload);
            if (strcmp((char *)payload, "wake") == 0) { // 检查消息是否为唤醒命令
                Serial.println("Waking up camera...");
                camera_init(); // 初始化摄像头
            } else if (strcmp((char *)payload, "sleep") == 0) { // 检查消息是否为休眠命令
                Serial.println("Putting camera to sleep...");
                camera_deinit(); // 停止摄像头
            } else { // 处理控制信号
                processControlCommand((char *)payload);
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
            // 处理ping/pong包
            break;
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Starting Camera Client!");
    wifi_init();
    
    // 舵机初始化
    servo_init();

    //IMU初始化
    Wire.begin(I2C_SDA, I2C_SCL); // 连接I2C总线
    Wire.setClock(100000);
    mpu.initialize(); // 初始化MPU6050
    pinMode(INPUT_PIN, INPUT); // MPU6050 INT引脚连接到ESP32的GPIO 2
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
    } else {
        Serial.println("MPU6050 connected");
    }
     // 初始化DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        Serial.println("DMP Initialized and Enabled");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }

    // 不在setup中初始化摄像头，等待wake信号来初始化
    // camera_init();
    webSocket.begin("49.233.216.82", 5901, "/esp32");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
}

void loop() {
    webSocket.loop();

    if (webSocket.isConnected()) {
        //获取电池电量并上传
        // float batteryLevel = getBatteryLevel();
        // char batteryMessage[50];
        // sprintf(batteryMessage, "battery:%.2fV", batteryLevel);
        
        // webSocket.sendTXT(batteryMessage);

        // 检查DMP数据准备状态
        if (mpuInterrupt) {
            mpuInterrupt = false;

            // 获取DMP数据包
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                Quaternion q;           // 四元数容器
                VectorFloat gravity;    // 重力容器
                float ypr[3];           // yaw/pitch/roll 容器

                // 从FIFO中提取四元数
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                // 格式化姿态数据
                DataPacket* imuPacket = new DataPacket;
                imuPacket->dataType = DATA_TYPE_IMU;
                String imuData = String(ypr[0] * 180 / M_PI) + "," +
                                 String(ypr[1] * 180 / M_PI) + "," +
                                 String(ypr[2] * 180 / M_PI);
                imuPacket->data.assign(imuData.begin(), imuData.end());
                std::vector<uint8_t> sendIMUData;

                sendIMUData.push_back(imuPacket->dataType);
                sendIMUData.insert(sendIMUData.end(), imuPacket->data.begin(), imuPacket->data.end());
                webSocket.sendBIN(sendIMUData.data(), sendIMUData.size());
                delete imuPacket;
            }
        }

       

        // 获取摄像头帧并上传
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            DataPacket* pkt = new DataPacket();
            
            pkt->dataType = DATA_TYPE_VIDEO;
            pkt->data.resize(fb->len);
            memcpy(pkt->data.data(), fb->buf, fb->len);
            std::vector<uint8_t> sendData;
            sendData.push_back(pkt->dataType);
            sendData.insert(sendData.end(), pkt->data.begin(), pkt->data.end());
            webSocket.sendBIN(sendData.data(), sendData.size());
            delete pkt;
            esp_camera_fb_return(fb);
        }
    }

    delay(10); // 每秒运行一次
}
