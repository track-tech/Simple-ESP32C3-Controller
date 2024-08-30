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
