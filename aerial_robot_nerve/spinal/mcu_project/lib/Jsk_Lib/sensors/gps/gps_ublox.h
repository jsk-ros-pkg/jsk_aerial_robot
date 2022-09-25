/*
******************************************************************************
* File Name          : gps_ublox.h
* Description        : UBLOX interface
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __GPS_UBLOX_H
#define __GPS_UBLOX_H

#include "sensors/gps/gps_backend.h"

#define UBLOX_PACKET_MAX_SIZE 110
#define PREAMBLE1 0xb5
#define PREAMBLE2 0x62

#define UBLOX_RXM_RAW_LOGGING 0
#define UBLOX_MAX_RXM_RAW_SATS 22
#define UBLOX_MAX_RXM_RAWX_SATS 32
#define UBLOX_GNSS_SETTINGS 1

#define UBLOX_MAX_GNSS_CONFIG_BLOCKS 7
#define UBX_MSG_TYPES 2

#define UBLOX_MAX_PORTS 6

#define RATE_PVT 1
#define RATE_POSLLH 0
#define RATE_STATUS 0
#define RATE_SOL 0
#define RATE_VELNED 0
#define RATE_HW 5
#define RATE_HW2 5

#define CONFIG_RATE_NAV      (1<<0)
#define CONFIG_RATE_POSLLH   (1<<1)
#define CONFIG_RATE_STATUS   (1<<2)
#define CONFIG_RATE_SOL      (1<<3)
#define CONFIG_RATE_VELNED   (1<<4)
#define CONFIG_RATE_DOP      (1<<5)
#define CONFIG_RATE_MON_HW   (1<<6)
#define CONFIG_RATE_MON_HW2  (1<<7)
#define CONFIG_RATE_RAW      (1<<8)
#define CONFIG_VERSION       (1<<9)
#define CONFIG_NAV_SETTINGS  (1<<10)
#define CONFIG_GNSS          (1<<11)
#define CONFIG_SBAS          (1<<12)

#define CONFIG_ALL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_SOL | CONFIG_RATE_VELNED \
                    | CONFIG_RATE_DOP | CONFIG_RATE_MON_HW | CONFIG_RATE_MON_HW2 | CONFIG_RATE_RAW | CONFIG_VERSION \
                    | CONFIG_NAV_SETTINGS | CONFIG_GNSS | CONFIG_SBAS)

//Configuration Sub-Sections
#define SAVE_CFG_IO     (1<<0)
#define SAVE_CFG_MSG    (1<<1)
#define SAVE_CFG_INF    (1<<2)
#define SAVE_CFG_NAV    (1<<3)
#define SAVE_CFG_RXM    (1<<4)
#define SAVE_CFG_RINV   (1<<9)
#define SAVE_CFG_ANT    (1<<10)
#define SAVE_CFG_ALL    (SAVE_CFG_IO|SAVE_CFG_MSG|SAVE_CFG_INF|SAVE_CFG_NAV|SAVE_CFG_RXM|SAVE_CFG_RINV|SAVE_CFG_ANT)

#define WARMUP_TIME 2000 //ms

class GPS : public GPS_Backend
{
public:
  GPS();
  void init(UART_HandleTypeDef *huart, ros::NodeHandle* nh,
            GPIO_TypeDef* led_port, uint16_t led_pin);
  void update() override;

private:
  // u-blox UBX protocol essentials
  struct PACKED ubx_header {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  };
#if UBLOX_GNSS_SETTINGS
  struct PACKED ubx_cfg_gnss {
    uint8_t msgVer;
    uint8_t numTrkChHw;
    uint8_t numTrkChUse;
    uint8_t numConfigBlocks;
    struct PACKED configBlock {
      uint8_t gnssId;
      uint8_t resTrkCh;
      uint8_t maxTrkCh;
      uint8_t reserved1;
      uint32_t flags;
    } configBlock[UBLOX_MAX_GNSS_CONFIG_BLOCKS];
  };
#endif
  struct PACKED ubx_cfg_nav_rate {
    uint16_t measure_rate_ms;
    uint16_t nav_rate;
    uint16_t timeref;
  };
  struct PACKED ubx_cfg_msg {
    uint8_t msg_class;
    uint8_t msg_id;
  };
  struct PACKED ubx_cfg_msg_rate {
    uint8_t msg_class;
    uint8_t msg_id;
    uint8_t rate;
  };
  struct PACKED ubx_cfg_msg_rate_6 {
    uint8_t msg_class;
    uint8_t msg_id;
    uint8_t rates[6];
  };
  struct PACKED ubx_cfg_nav_settings {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t res1;
    uint32_t res2;
    uint32_t res3;
    uint32_t res4;
  };
  struct PACKED ubx_cfg_prt {
    uint8_t portID;
  };
  struct PACKED ubx_cfg_sbas {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
  };
  struct PACKED ubx_nav_posllh {
    uint32_t time;                                  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
  };
  struct PACKED ubx_nav_status {
    uint32_t time;                                  // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;                                // milliseconds
  };
  struct PACKED ubx_nav_dop {
    uint32_t time;                                  // GPS msToW
    uint16_t gDOP;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t vDOP;
    uint16_t hDOP;
    uint16_t nDOP;
    uint16_t eDOP;
  };
  struct PACKED ubx_nav_solution {
    uint32_t time;
    int32_t time_nsec;
    uint16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
  };
  struct PACKED ubx_nav_pvt{
    uint32_t time;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t time_accuracy;
    int32_t  nano;
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t additional_flags;
    uint8_t satellites;
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
    uint16_t position_DOP;
    uint8_t res[6];
    int32_t heading;
    int16_t mag_dec;
    uint16_t mag_dec_acc;
  };
  struct PACKED ubx_nav_velned {
    uint32_t time;                                  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
  };

  // Lea6 uses a 60 byte message
  struct PACKED ubx_mon_hw_60 {
    uint32_t pinSel;
    uint32_t pinBank;
    uint32_t pinDir;
    uint32_t pinVal;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t aStatus;
    uint8_t aPower;
    uint8_t flags;
    uint8_t reserved1;
    uint32_t usedMask;
    uint8_t VP[17];
    uint8_t jamInd;
    uint16_t reserved3;
    uint32_t pinIrq;
    uint32_t pullH;
    uint32_t pullL;
  };
  // Neo7 uses a 68 byte message
  struct PACKED ubx_mon_hw_68 {
    uint32_t pinSel;
    uint32_t pinBank;
    uint32_t pinDir;
    uint32_t pinVal;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t aStatus;
    uint8_t aPower;
    uint8_t flags;
    uint8_t reserved1;
    uint32_t usedMask;
    uint8_t VP[25];
    uint8_t jamInd;
    uint16_t reserved3;
    uint32_t pinIrq;
    uint32_t pullH;
    uint32_t pullL;
  };
  struct PACKED ubx_mon_hw2 {
    int8_t ofsI;
    uint8_t magI;
    int8_t ofsQ;
    uint8_t magQ;
    uint8_t cfgSource;
    uint8_t reserved0[3];
    uint32_t lowLevCfg;
    uint32_t reserved1[2];
    uint32_t postStatus;
    uint32_t reserved2;
  };
  struct PACKED ubx_mon_ver {
    char swVersion[30];
    char hwVersion[10];
    char extension; // extensions are not enabled
  };
  struct PACKED ubx_nav_svinfo_header {
    uint32_t itow;
    uint8_t numCh;
    uint8_t globalFlags;
    uint16_t reserved;
  };
  struct PACKED ubx_ack_ack {
    uint8_t clsID;
    uint8_t msgID;
  };

  struct PACKED ubx_cfg_cfg {
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
  };

  enum ubs_protocol_bytes {
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_MON = 0x0A,
    CLASS_RXM = 0x02,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_DOP = 0x4,
    MSG_SOL = 0x6,
    MSG_PVT = 0x07,
    MSG_VELNED = 0x12,
    MSG_CFG_CFG = 0x09,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_MSG = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_SBAS = 0x16,
    MSG_CFG_GNSS = 0x3E,
    MSG_MON_HW = 0x09,
    MSG_MON_HW2 = 0x0B,
    MSG_MON_VER = 0x04,
    MSG_NAV_SVINFO = 0x30,
    MSG_RXM_RAW = 0x10,
    MSG_RXM_RAWX = 0x15
  };
  enum ubx_gnss_identifier {
    GNSS_GPS     = 0x00,
    GNSS_SBAS    = 0x01,
    GNSS_GALILEO = 0x02,
    GNSS_BEIDOU  = 0x03,
    GNSS_IMES    = 0x04,
    GNSS_QZSS    = 0x05,
    GNSS_GLONASS = 0x06
  };
  enum ubs_nav_fix_type {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
  };
  enum ubx_nav_status_bits {
    NAV_STATUS_FIX_VALID = 1,
    NAV_STATUS_DGPS_USED = 2
  };
  enum ubx_hardware_version {
    ANTARIS = 0,
    UBLOX_5,
    UBLOX_6,
    UBLOX_7,
    UBLOX_M8
  };
  enum ubs_pvt_valid_flags {
    VALID_DATE = 1,
    VALID_TIME = 2,
    VALID_FULLY_RESOLVED = 4,
    VALID_MAG = 8
  };

  // Receive buffer
  union PACKED{
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_dop dop;
    ubx_nav_solution solution;
    ubx_nav_pvt pvt;
    ubx_nav_velned velned;
    ubx_cfg_msg_rate msg_rate;
    ubx_cfg_msg_rate_6 msg_rate_6;
    ubx_cfg_nav_settings nav_settings;
    ubx_cfg_nav_rate nav_rate;
    ubx_cfg_prt prt;
    ubx_mon_hw_60 mon_hw_60;
    ubx_mon_hw_68 mon_hw_68;
    ubx_mon_hw2 mon_hw2;
    ubx_mon_ver mon_ver;
    ubx_cfg_gnss gnss;
    ubx_cfg_sbas sbas;
    ubx_nav_svinfo_header svinfo_header;
    ubx_ack_ack ack;
    uint8_t bytes[UBLOX_PACKET_MAX_SIZE];
  }raw_packet_;

  uint8_t step_ = 0;
  uint8_t msg_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_counter_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t class_ = 0;

  GPIO_TypeDef* led_port_;
  uint16_t led_pin_;

  void processMessage() override;
  void updateChecksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b);
  void sendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size);
  void configureRate(uint16_t rate);
  bool configureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
};

#endif // __GPS_UBLOX_H_
