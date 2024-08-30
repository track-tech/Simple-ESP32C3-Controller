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
