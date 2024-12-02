# 北京大学智能电子系统设计与实践课程项目：BlackEye
## 项目概述
本项目为北京大学智能电子系统设计与实践课程项目，本仓库为摄像头固件部分。BlackEye旨在复刻《RainbowSix:Siege》游戏中Valkyrie干员的技能：“黑眼”摄像头。
在游戏中，“黑眼”摄像头是一个信息获取道具（[技能演示视频](https://markdown.com.cn)）；在我们复刻的过程中，希望让它具有更生活化的功能，最终设定为家庭场景下的安防摄像头应用。
固件端使用ESP32-Cam模块作为主控MCU；使用2个SG90舵机作为旋转摄像头的机械装置；摄像头使用一个广角OV2640摄像头以实现广阔的视野和与游戏内相似的效果；附加一个MPU6050模块采集姿态数据用于在用户端可视化。与服务端的通讯协议采用WebSocket。
本仓库包括：ESP32固件代码；PCB设计图；外壳设计图。
## 使用说明
### ESP32固件代码
在使用时，请将`/src/main.cpp`中的<br>
```
const char *ssid = "";              // wifi用户名
const char *password = "";          // wifi密码
const IPAddress serverIP(xx, xxx, xxx, xx); // 你自己的公网服务器ip地址
uint16_t serverPort = xx;                   // 服务器端口号(tcp协议)
```
根据你自己的情况进行修改。确保你有一个公网服务器可用运行BlackEye的Nodejs服务端代码。<br>
外设模块的连接请参考`/src/main.cpp`中的定义接口部分：<br>
```
//定义舵机引脚
#define SERVO_PIN_H 13 // 水平舵机
#define SERVO_PIN_V 12 // 垂直舵机
//定义I2C引脚
#define I2C_SDA 14
#define I2C_SCL 15
#define INPUT_PIN 2 // 接到MPU6050的INT引脚
//定义MPU6050
MPU6050 mpu(0x68); // 定义MPU6050对象，连接到地址为0x68的I2C总线
```
### PCB设计
我们的电源板设计具有很大的漏洞，请勿采纳我们的设计。如有需要，请自行搜索可用的设计。只需保证有5V1.2A的输出和3.3V0.1A的输出即可。<br>
ESP32cam和外设的连接板可以使用。但其中的INMP441麦克风部分已经废弃，您在使用时可以删除。

### 外壳设计
外壳设计建议根据您的需求重新调整尺寸。我们的设计尺寸不符合市面上现售的亚克力透明半球尺寸，导致需要定制亚克力透明半球来完成上半球外壳。





