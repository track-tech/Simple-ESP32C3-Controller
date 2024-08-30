# 【开源/遥控改装】简单的ESPNOW遥控方案

## 0x01 简单介绍

<img src=".\pic\1.jpg" style="zoom:25%;" />



在淘宝电子电子垃圾店看到了这个小控，感觉之后拿来测试些东西很方便，就买来改造了



- **关于板载资源**

整个遥控有两个摇杆，四个肩键，其中左下肩键为旋钮

其次还板载一个无源蜂鸣器，上电无信号会响，感觉吵或者有需要可以刮断信号线接线改造



综上所述需要共计5个ADC IO，3个普通IO（无蜂鸣器，需要就再加一个）

最后选用esp32c3来做主控和射频单元



## 0x02 硬件连接及代码

**注：开发环境Arduino，ESPNOW通信相关API调用IDF** 

ESPIDF API在新版本进行了更新，导致网上的一些旧版代码无法运行，这个新旧区别在回调函数的第一个传参数，把uint8_t *改为esp_now_recv_info_t *就可以正常使用，具体看

https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32c3/api-reference/network/esp_now.html#_CPPv417esp_now_recv_cb_t



<img src=".\pic\2.jpg" style="zoom:25%;" />

硬件连接如上图所示，实际测量出对应脚口焊接即可



### Ⅰ发射端

```c
/*
--适用于淘宝9.9遥控改装--

Borad/Chip:ESP32C3 SuperMini

ESP32C3Controller.ino

*/


#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"

#include "espnow.h"
//#include "pitches.h"      //蜂鸣器用，标准音符频率

int sensorValue[5];     //原始数据
int sensorValue_ok[5];  //滤波数据
char buff[50];



//注意专用ADC接口GPIO0/1/2/3/4  共计5个
//摇杆GPIO设定-ADC
#define RX 2
#define RY 1
#define LX 4
#define LY 3
//按键GPIO设定
#define LB 6
#define LT 0  //ADC旋钮
#define RB 5
#define RT 7

#define BUZZER 8



// void Buzzer_Init()
// {

//  ledcAttach(BUZZER, 1000, 12);

//   for(int i = 0;i<3;i++)
//   {
//       ledcWriteTone(BUZZER, NOTE_C4);
//       delay(500);
//       ledcWriteTone(BUZZER, 0);
//       delay(500);
//   }
// }

/*
一阶互补滤波
方法：取a=0~1,本次滤波结果=（1-a）本次采样值+a上次滤波结果
优点：对周期性干扰具有良好的抑制作用适用于波动频率较高的场合
缺点：相位滞后，灵敏度低滞后程度取决于a值大小不能消除滤波频率高于采样频率的1/2的干扰信号
*/

int firstOrderFilter(int newValue, int oldValue, float a)
{
	return a * newValue + (1-a) * oldValue;
}


void setup() {

  Serial.begin(115200);

//ESP-NOW通信相关
  // 初始化 ESP-NOW
  Serial.println("Initializing ESP-NOW...");
  delay(100);
  WiFi.mode(WIFI_STA);
  Serial.println("InitESPNow");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  
  // 设置发送数据回调函数
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // 设置接收数据回调函数
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
//  esp_now_register_recv_cb(OnDataRecv);

  //配对连接pair with another ESP-NOW device
  pair_device();
  
  // resetData();  
  
//硬件读取  
  sensorValue_ok[0] = analogRead(LT);
  sensorValue_ok[1] = analogRead(RX);
  sensorValue_ok[2] = analogRead(RY);
  sensorValue_ok[3] = analogRead(LX);
  sensorValue_ok[4] = analogRead(LY);

  pinMode(RB, INPUT_PULLUP);
  pinMode(RT, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  pinMode(LT, INPUT_PULLUP);

  // Buzzer_Init();
}




void loop() {

  sensorValue[0] = analogRead(LT);
  sensorValue[1] = analogRead(RX);
  sensorValue[2] = analogRead(RY);
  sensorValue[3] = analogRead(LX);
  sensorValue[4] = analogRead(LY);

  for (int i = 0; i<5; i++) {
    sensorValue_ok[i] = firstOrderFilter(sensorValue[i] ,sensorValue_ok[i], 0.05);
  }


//写入结构体
  data.j1PotX = sensorValue_ok[1];
  data.j1PotY = sensorValue_ok[2];
  data.j2PotX = sensorValue_ok[3];
  data.j2PotY = sensorValue_ok[4];

  data.j1Button = 0;
  data.j2Button = 0;

  data.buttonLB = digitalRead(LB);
  data.buttonRB = digitalRead(RB);
  data.buttonRT = digitalRead(RT);
  data.buttonLT = sensorValue[0];

  data.tSwitch1 = 1;
  data.roll=127;
  data.pitch=127;
  data.buttonR3 = 1;  


  // 适配VOFA上位机
  // sprintf(buff,"samples:%d, %d, %d, %d, %d\n", sensorValue[0], sensorValue[1], sensorValue[2], sensorValue[3], sensorValue[4]);
  sprintf(buff,"location test:%d, %d, %d, %d, %d, %d, %d, %d\n", sensorValue_ok[1], sensorValue_ok[2], sensorValue_ok[3], sensorValue_ok[4]
                                                         , digitalRead(RB), digitalRead(RT), digitalRead(LB), sensorValue[0]);
  Serial.print(buff);

  
  // NowSerial.print(buff);

    sendData();




  delay(1);
}



```



```
//通信框架参考https://blog.csdn.net/Better_man1/article/details/120552196
//espnow.h-发射端

#ifndef ESPNOW_H
#define ESPNOW_H

#include <WiFi.h>
#include <esp_now.h>

//全0xFF的Mac地址时广播到附近所有ESPNOW设备
  // uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  // uint8_t broadcastAddress[] = {0x48, 0xCA, 0x43, 0xDA, 0xDD, 0xC8};

  uint8_t broadcastAddress[] = {0x60, 0x55, 0xF9, 0x75, 0xA8, 0x2C};
  
  esp_now_peer_info_t peerInfo = {}; 

// 设置数据结构体                        
typedef struct Data_Package {
  short j1PotX;  //左右摇杆
  short j1PotY;
  short j2PotX;
  short j2PotY;

  bool j1Button;  //左右摇杆按键
  bool j2Button;

  bool buttonRB;  //左右按键
  bool buttonLB;

  short buttonRT;  //左右扳机按键
  short buttonLT;

  bool tSwitch1;  //拨动开关   
  short roll;      //用于M5StackCore2或未来增加的陀螺仪功能
  short pitch;
  bool buttonR3;
};
Data_Package data; //Create a variable with the above structure

void resetData() {  //数据重置
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.buttonLT = 1;
  data.buttonRT = 1;
  data.j1Button = 1;
  data.j2Button = 1;
  data.buttonLB = 1;
  data.buttonRB = 1;
  data.tSwitch1 = 1;
  data.roll=127;
  data.pitch=127;
  data.buttonR3 = 1;   
}

/////////////////////////////////////////

    
// 数据发送回调函数
// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
//  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 数据接收回调函数
// callback when data is received from Slave to Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&data, incomingData, sizeof(data));
//  Serial.print("j1PotX: ");
//  Serial.print(data.j1PotX);
//  Serial.print("; j1PotY: ");
//  Serial.print(data.j1PotY); 
//  Serial.print("; j2PotX: ");
//  Serial.print(data.j2PotX);
//  Serial.print("; j2PotY: ");
//  Serial.println(data.j2PotY);
}


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) {       //ESP_OK=0
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

//配对连接pair with another ESP-NOW device
void pair_device(){     
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  } 
}

// 发送数据 send data
void sendData() {       //uint8_t data  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
}


#endif  

```





### Ⅱ 接收端

```
/*
ESP32C3Controller_Receiver.ino

接收端主程序

*/

#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"

#include "espnow.h"

char buff[50];
String buff_str;

int CH[12];




// Set the MAC address of the device that will receive the data
//设定对端通信设备MAC地址
// const MacAddress peer_mac({0x48, 0xCA, 0x43, 0xDA, 0xFB, 0x10});


void setup() {
  Serial.begin(115200);
/**********************************************/    
  // 初始化 ESP-NOW
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();

  // 设置接收数据回调函数

//  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);  
/**********************************************/ 

  
  ledcAttach(9, 50, 12);//servo

  ledcAttach(12, 50, 12);//LED1
  ledcAttach(13, 50, 12);//LED2


}

void loop() {

  //  esp_now_register_recv_cb(OnDataRecv);

  ledcWrite(12, data.j1PotX);
  ledcWrite(13, data.j1PotY);

  ledcWrite(9 ,map(data.j2PotY, 0, 4076, 100, 512)); // 输出PWM


  delay(1);
}

```



```
//通信框架参考https://blog.csdn.net/Better_man1/article/details/120552196
//espnow.h-接收端

#include <WiFi.h>
#include <esp_now.h>

typedef struct Data_Package {
  short j1PotX;  //左右摇杆
  short j1PotY;
  short j2PotX;
  short j2PotY;

  bool j1Button;  //左右摇杆按键
  bool j2Button;

  bool buttonRB;  //左右按键
  bool buttonLB;

  short buttonRT;  //左右扳机按键
  short buttonLT;

  bool tSwitch1;  //拨动开关   
  short roll;      //用于M5StackCore2或未来增加的陀螺仪功能
  short pitch;
  bool buttonR3;
};
Data_Package data; //Create a variable with the above structure


void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength)
{
  snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
}


// typedef void （*esp_now_recv_cb_t）（const esp_now_recv_info_t *esp_now_info， const uint8_t *data， int data_len)
// 数据接收回调函数
void OnDataRecv(const  esp_now_recv_info_t * mac, const uint8_t *incomingData, int len) {    
  char buff[100];
  
  memcpy(&data, incomingData, sizeof(data));
  // Serial.print("j1PotX: ");
  // Serial.print(data.j1PotX);
  // Serial.print("; j1PotY: ");
  // Serial.print(data.j1PotY); 
  // Serial.print("; j2PotX: ");
  // Serial.print(data.j2PotX);
  // Serial.print("; j2PotY: ");
  // Serial.print(data.j2PotY);
  // Serial.print("; ***buttonR1: ");
  // Serial.print(data.buttonR1);
  // Serial.print("; ***buttonR2: ");
  // Serial.print(data.buttonR2);
  // Serial.print("; buttonR3: ");
  // Serial.println(data.buttonR3);
  // Serial.print("; ***roll: ");
  // Serial.print(data.roll);
  // Serial.print("; pitch: ");
  // Serial.println(data.pitch);

  sprintf(buff,"ESPNOW-Controller:%d, %d, %d, %d, %d, %d, %d, %d\n", data.j1PotX, data.j1PotY, data.j2PotX, data.j2PotY
                                                         , data.buttonRB, data.buttonRT, data.buttonLB, data.buttonLT);
  Serial.print(buff);


}


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}


```







## 0x03 最终效果

最终可以通过在VOFA上位机和板载LED看到效果

演示视频见



