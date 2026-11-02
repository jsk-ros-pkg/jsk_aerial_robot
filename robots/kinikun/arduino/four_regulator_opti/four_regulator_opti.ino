// =======================================
// 4ch MPA Regulator Controller (rosserial)
// - 4つのレギュレータを MCP4725 + PCA9548A で制御
// - 4つの圧力センサ(0–0.9MPa, 1–5V 出力)を A0–A3 から読み取り
// - mpa_cmd    : std_msgs/Float32MultiArray (4要素, DACカウント想定)
// - mpa_pressure : std_msgs/Float32MultiArray (4要素, [MPa])
// =======================================

#include <Arduino.h>
#include <Wire.h>       // I2C
#include <MsTimer2.h>   // タイマー割り込み

// ★rosserialの受信バッファ（Uno節約用、必要なら省略可）
// #define ROSLIB_SERIAL_BUFFER_SIZE 256
#include <ros.h>
#include <geometry_msgs/Quaternion.h>

ros::NodeHandle nh;

// ---- ハード構成パラメータ ----
static const uint8_t NUM_MPA      = 4;    // 制御するレギュレータ数
static const uint8_t MCP4725_ADDR = 0x60; // DAC アドレス
static const uint8_t MUX_ADDR     = 0x70; // PCA9548A アドレス

// MUX の各ポートに1個ずつレギュレータがぶら下がっている想定
const uint8_t MPA_MUX_PORT[NUM_MPA]   = {0, 1, 2, 3};  // ポート番号 0〜3
// 各レギュレータに対応する圧力センサの A ピン
const uint8_t MPA_SENSOR_PIN[NUM_MPA] = {A0, A1, A2, A3};

// ---- 制御周期（ms） ----
static const uint16_t CONTROL_PERIOD_MS = 10; // 10ms = 100Hz

// ---- 制御用共有変数（ISR と loop 間で共有）----
volatile float g_dac_cmd[NUM_MPA]  = {0.0f};  // 各レギュレータへの DAC 指令 (0〜4095想定)
volatile float g_press_buf[NUM_MPA] = {0.0f}; // [MPa] センサ値バッファ
volatile bool  g_press_ready        = false;  // 新しいセンサ値が入ったフラグ

volatile unsigned long g_next_control_ms = 0; // 次に制御を実行する時刻

// ---- ROS: Publish 用バッファ ----
geometry_msgs::Quaternion msg_pressure;

// ---- 関数プロトタイプ ----
void selectMuxPort(uint8_t port);      // MUX ポート選択
void writeDacRaw(uint16_t value);      // MCP4725 に生の 12bit 値を書き込む
float adcToMpa(int adc_raw);           // ADC値 -> MPa 変換

// =======================================
//  コマンド受信コールバック
//  topic: "mpa_cmd", type: std_msgs/Float32MultiArray
// =======================================
void mpaCmdCallback(const geometry_msgs::Quaternion& msg)
{
  g_dac_cmd[0] = msg.x;
  g_dac_cmd[1] = msg.y;
  g_dac_cmd[2] = msg.z;
  g_dac_cmd[3] = msg.w;
}

// ROS サブスクライバ
ros::Subscriber<geometry_msgs::Quaternion> sub_mpa_cmd("mpa_cmd", &mpaCmdCallback);

// ROS パブリッシャ
ros::Publisher pub_mpa_pressure("mpa_pressure", &msg_pressure);

// =======================================
//  Timer2 ISR から呼ばれる 10ms タスク
//  - DAC 出力更新
//  - 圧力センサ読み取り
//  - バッファ更新のみ（Publish は loop 側）
// =======================================
void flash()
{
  // Timer 割り込みの中だが、I2C (TWI) が動くようにグローバル割り込みを許可
  sei();

  unsigned long now = millis();
  if (now < g_next_control_ms) {
    return;
  }
  g_next_control_ms = now + CONTROL_PERIOD_MS;

  // --- 1. DAC に指令値を書き込む ---
  for (uint8_t i = 0; i < NUM_MPA; ++i) {
    // コマンド値を 0〜4095 に制限
    int32_t cmd = (int32_t)g_dac_cmd[i];
    if (cmd < 0) cmd = 0;
    if (cmd > 4095) cmd = 4095;

    // 該当する MUX ポートを選択
    selectMuxPort(MPA_MUX_PORT[i]);

    // DAC へ書き込み
    writeDacRaw((uint16_t)cmd);
  }

  // --- 2. 圧力センサ読み取り (MPa に変換) ---
  for (uint8_t i = 0; i < NUM_MPA; ++i) {
    int adc = analogRead(MPA_SENSOR_PIN[i]);
    float p = adcToMpa(adc);
    if (p < 0.0f) p = 0.0f;
    g_press_buf[i] = p;
  }

  // 新しいセンサ値が用意できたことを通知
  g_press_ready = true;
}

// =======================================
//  セットアップ
// =======================================
void setup()
{
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  selectMuxPort(MPA_MUX_PORT[0]);
  writeDacRaw(0);

  MsTimer2::set(CONTROL_PERIOD_MS, flash);
  MsTimer2::start();
  g_next_control_ms = millis() + CONTROL_PERIOD_MS;

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_mpa_cmd);
  nh.advertise(pub_mpa_pressure);
}

// =======================================
//  メインループ
//  - ISR が準備したセンサ値を Publish
// =======================================
void loop()
{
  if (g_press_ready) {
    float local_press[NUM_MPA];

    // クリティカルセクション：バッファをローカルにコピー
    noInterrupts();
    for (uint8_t i = 0; i < NUM_MPA; ++i) {
      local_press[i] = g_press_buf[i];
    }
    g_press_ready = false;
    interrupts();

    // Quaternion に詰めて Publish
    msg_pressure.x = (NUM_MPA > 0) ? local_press[0] : 0.0f;
    msg_pressure.y = (NUM_MPA > 1) ? local_press[1] : 0.0f;
    msg_pressure.z = (NUM_MPA > 2) ? local_press[2] : 0.0f;
    msg_pressure.w = (NUM_MPA > 3) ? local_press[3] : 0.0f;

    pub_mpa_pressure.publish(&msg_pressure);
  }

  nh.spinOnce();
  delay(1);
}

// =======================================
//  ヘルパ関数群
// =======================================

// --- MUX ポート選択 ---
// PCA9548A の制御レジスタに 1<<port を書き込むだけのシンプル版
void selectMuxPort(uint8_t port)
{
  if (port > 7) port = 7;

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << port); // このポートのみ ON
  Wire.endTransmission();
}

// --- MCP4725 に 12bit 生値を書き込む ---
void writeDacRaw(uint16_t value)
{
  // 12bit 上限
  value &= 0x0FFF;

  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(0x40);               // Fast mode, write DAC input register
  Wire.write(value >> 4);         // 上位8bit
  Wire.write((value & 0x0F) << 4);// 下位4bitを上位へ
  Wire.endTransmission();
}

// --- ADC値 -> MPa 変換関数 ---
// センサ仕様:
//   出力電圧: 1〜5V が 0〜0.9MPa に対応
//   ADC: 10bit (0〜1023), 参照電圧 5V
float adcToMpa(int adc_raw)
{
  float v = (float)adc_raw * (5.0f / 1023.0f); // 0〜5V
  float p = (v - 1.0f) * (0.9f / 4.0f);        // 1〜5V -> 0〜0.9MPa
  return p;
}
