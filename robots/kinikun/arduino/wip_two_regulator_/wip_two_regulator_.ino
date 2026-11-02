#include <Arduino.h>
#include <Wire.h>    //I2C
#include <EEPROM.h>  //Needed to record user settings
#include <MsTimer2.h>            // タイマー割り込み

// ★rosserialの受信バッファ（Uno節約用、必要なら省略可）
// #define ROSLIB_SERIAL_BUFFER_SIZE 256
#include <ros.h>
#include <geometry_msgs/Vector3.h> 

//EEPROM locations to store 4-byte variables
#define LOCATION_CALIBRATION_FACTOR 0 //Float, requires 4 bytes of EEPROM
#define LOCATION_ZERO_OFFSET 10 //Must be more than 4 away from previous spot. Long, requires 4 bytes of EEPROM
#define MUX_PORT_1 0
#define MUX_PORT_2 1
#define MUX_PORT_3 2
#define MUX_PORT_4 3
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
unsigned long time_now;
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

// ===== 追加：Publish用バッファ =====
volatile float p1_mpa_buf = 0.0f;
volatile float p2_mpa_buf = 0.0f;
volatile bool  press_ready = false;

geometry_msgs::Vector3 msg_pressure;
ros::Publisher pub_mpa_pressure("mpa_pressure", &msg_pressure);

// コールバック関数
void mpaCmdCallback(const geometry_msgs::Vector3& msg) {
  noInterrupts();
  V1 = msg.x;
  V2 = msg.y;
  interrupts();
}

// ROSサブスクライバの宣言
ros::Subscriber<geometry_msgs::Vector3> sub_mpa_cmd("mpa_cmd", mpaCmdCallback);

// ★②③で提供されている関数の宣言だけ（定義は別スケッチにある想定）
void OutputRegulator(int data, int ch);

void flash() {
  if (commandtime <= millis()){
    sei(); // （元のまま）割り込み有効
    time_now = millis();//プログラム実行時からの経過時間
    T1 = (float)time_now/1000;  //float型に変換

    // レギュレータ更新（②③に依存）
    OutputRegulator((int)V1, MUX_PORT_1);
    OutputRegulator((int)V2, MUX_PORT_2);

    // センサ読み取り（MPa換算）
    int sensorValue1 = analogRead(MPA_P1_PIN);
    float p1_calc = ((sensorValue1 * (5.0f / 1023.0f)) - 1.0f) * (0.9f / 4.0f);
    if (p1_calc < 0) p1_calc = 0;

    int sensorValue2 = analogRead(MPA_P2_PIN);
    float p2_calc = ((sensorValue2 * (5.0f / 1023.0f)) - 1.0f) * (0.9f / 4.0f);
    if (p2_calc < 0) p2_calc = 0;

    // ★ISR内ではPublishしない。値だけバッファへ
    p1_mpa_buf = p1_calc;
    p2_mpa_buf = p2_calc;
    press_ready = true;

    // （任意）デバッグ出力は通信干渉しうるので止める
    // Serial.print(p1_calc); Serial.print(",,,"); Serial.println(p2_calc);

    commandtime = millis() + 10; // 10ms周期
  } 
}

void setup()
{
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000); // 400kHz

  // レギュレータ入力初期化
  OutputRegulator(0, MUX_PORT_1);
  OutputRegulator(0, MUX_PORT_2);

  delay(100);
  MsTimer2::set(10, flash);  // 10ms毎
  MsTimer2::start();         
  commandtime = millis() + 5000;

  // ★rosserialのボーレートを明示
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_mpa_cmd);
  nh.advertise(pub_mpa_pressure);
}

void loop() {
  // ★100Hzでセンサ値をPublish（ISRで準備完了時）
  if (press_ready) {
    noInterrupts();
    float p1_local = p1_mpa_buf;
    float p2_local = p2_mpa_buf;
    press_ready = false;
    interrupts();

    msg_pressure.x = p1_local;
    msg_pressure.y = p2_local;
    msg_pressure.z = 0.0f;
    pub_mpa_pressure.publish(&msg_pressure);
  }

  nh.spinOnce();
  delay(1);
}
