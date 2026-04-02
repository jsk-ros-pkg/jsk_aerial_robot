#include <Arduino.h>
#include <Wire.h>    //I2C
#include <EEPROM.h> //Needed to record user settings
#include <MsTimer2.h>            // タイマー割り込み
#include <ros.h>
#include <geometry_msgs/Vector3.h> 


//EEPROM locations to store 4-byte variables
#define LOCATION_CALIBRATION_FACTOR 0 //Float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 10 //Must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM

ros::NodeHandle nh;


//レギュレータ圧力指令値読み取り用変数
const int MPA_P1_PIN = A0;
const int MPA_P2_PIN = A1;
//センサー圧力読み取り変数

bool settingsDetected = false; //Used to prompt user to calibrate their scale

//ポテンショメータ用
int analogPin = 0; 
float val = 0.0;           //読み取った値を格納する変数
// 制御用変数
unsigned long time;
const float cycle = 5;

volatile float V1; //レギュレータへの電圧指令値
volatile float V2; //レギュレータへの電圧指令値
unsigned long nexttime;
unsigned long commandtime;//制御ループの時間監視
long currentReading1;
long currentReading2;
// データ格納用変数
float p1;//レギュレータからの読み取り値格納変数
float p2;//レギュレータからの読み取り値格納変数
int outputreg1;//int型で読み取る場合
int outputreg2;//int型で読み取る場合
float omega=2*PI/10;
volatile float T1=0.0; 

// コールバック関数
void mpaCmdCallback(const geometry_msgs::Vector3& msg) {
  noInterrupts();
  V1 = msg.x;
  V2 = msg.y;
  interrupts();
}

// ROSサブスクライバの宣言
ros::Subscriber<geometry_msgs::Vector3> sub_mpa_cmd("mpa_cmd", mpaCmdCallback);


void flash() {
   if(commandtime <= millis()){
    sei(); // 割り込み有効
    time = millis();//プログラム実行時からの経過時間
    T1 = (float)time/1000;  //float型に変換
    OutputRegulator((int)V1,1);
    OutputRegulator((int)V2,2);
    //データの計測
    p1 = (float)(analogRead(MPA_P1_PIN));
    p2 = (float)(analogRead(MPA_P2_PIN));
    outputreg1 = (int)p1;
    outputreg2 = (int)p2;

    //読み取り値出力
    Serial.print(T1);
    Serial.print(",");
    Serial.print(V1,2);
    Serial.print(",");
    Serial.print(V2,2);
    Serial.print(",");
    Serial.print(outputreg1);
    Serial.print(",");
    Serial.print(outputreg2);
    Serial.println();
    
    commandtime = commandtime +10;
  } 
}
void setup()
{
  Serial.begin(115200);
  //Serial.println("Qwiic Scale Example");

  Wire.begin();
  Wire.setClock(400000); //Qwiic Scale is capable of running at 400kHz if desired

  //レギュレータ入力初期化
  OutputRegulator(0, 1);
  OutputRegulator(0, 2);

  //myScale1.setSampleRate(NAU7802_SPS_320);
  delay(100);
  MsTimer2::set(10, flash);     // 10ms毎の割り込み、その時に処理する関数flash( )を呼び出し
  MsTimer2::start();           // タイマー割り込み開始
  commandtime = millis() + 5000;   

  

  nh.initNode();
  nh.subscribe(sub_mpa_cmd);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
