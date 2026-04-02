// アドレス
#define MCP4725_ADDR1 0x60  //higher
#define MCP4725_ADDR2 0x61    
#define MUX_ADDR 0x70
void OutputRegulator(int data, int ch) {

  switch (ch) {
    case 1:
      Wire.beginTransmission(MCP4725_ADDR1);//通信開始
      break;
    case 2:
      Wire.beginTransmission(MCP4725_ADDR2);
      break;
      defalut:
      return;
  }

  Wire.write(64);                     // cmd to update the DAC
  Wire.write(data >> 4);        // the 4 least significant bits...//データ書き込み
  Wire.write((data & 15) << 4); // the 8 most significant bits...
  Wire.endTransmission();//通信の終了
}
