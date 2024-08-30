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
void OnDataRecv(const esp_now_recv_info_t * mac, const uint8_t *incomingData, int len) {    
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

