#define _BV(bit) (1 << (bit))
#define bit(b) (1UL << (b))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define lowWord(w) ((uint16_t)((w) & 0xffff))
#define highWord(w) ((uint16_t)((w) >> 16))
#define bytesToWord(hb, lb) ((uint16_t)((((uint16_t)(hb & 0xFF)) << 8) | ((uint16_t)lb)))
#define wordstoDWord(hw, lw) ((uint32_t)((((uint32_t)(hw & 0xFFFF)) << 16) | ((uint32_t)lw)))

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define swap(x,y) { x = x + y; y = x - y; x = x - y; }

#define abs(value) ((value)>0?(value):(-value))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define LV_MATH_ABS(x) ((x)>0?(x):(-(x)))
