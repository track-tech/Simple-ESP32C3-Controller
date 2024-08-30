/*
--适用于淘宝9.9遥控改装--

Borad/Chip:ESP32C3 SuperMini

ESP32C3Controller.ino

*/


#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"

#include "espnow.h"
#include "pitches.h"      //蜂鸣器用，标准音符频率

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


