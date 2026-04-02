// アドレス
#define MCP4725_ADDR 0x60 //higher   
#define MUX_ADDR 0x70
void OutputRegulator(int data, int ch) {
  
  static int lastChannel = -1;  // 前回選択したチャネルを記録
  
  if (lastChannel != ch) {
    enableMuxPort(ch);  // 選択が変わった場合のみ有効化
    lastChannel = ch;
  }
  delay(1);
  Wire.beginTransmission(MCP4725_ADDR);
  Wire.write(64);                     // cmd to update the DAC
  Wire.write(data >> 4);        // the 4 least significant bits...//データ書き込み
  Wire.write((data & 15) << 4); // the 8 most significant bits...
  Wire.endTransmission();//通信の終了
  disableMuxPort(ch);
}
