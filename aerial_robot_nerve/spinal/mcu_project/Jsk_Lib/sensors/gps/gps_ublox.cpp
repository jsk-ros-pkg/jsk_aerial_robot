/*
******************************************************************************
* File Name          : gps_ublox.cpp
* Description        : Ublox interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/gps/gps_ublox.h"
#include "config.h"

#define UBLOX_SPEED_CHANGE 0

#define UBLOX_DEBUGGING 1
#define UBLOX_FAKE_3DLOCK 0

//the connection with FCU
//ublox: I2C SLC(RED,COMPASS), I2C SDA(COMPASS), VCC (5V) RX, TX, GNG

//reset:
//1. change to 9600 baud-rate,  uncomment the RESET_CONFIG
//2. comment the RESET_CONFIG, back to baudrate 57600, uncomment SAVE_CONFIG, change MEASURE_RATE to 10000(ms),  push reset button of board for several times
//3. comment SAVE_CONFIG, back MEASURE_RATE to 100ms
//#define RESET_CONFIG
//#define SAVE_CONFIG

//still have bug
//timer2 should have slower rate(e.g. 40[ms]= 40000, 20[ms]= 20000 may cause some freeze problem for some gps module, can not figure out the reason!)


GPS::GPS():GPS_Backend(){}

void GPS::init(UART_HandleTypeDef *huart, ros::NodeHandle* nh)
{
  baseInit (huart, nh);
  _step = 0;
  _msg_id = 0;
  _payload_length = 0;
  _payload_counter = 0;
  _class = 0;
  _cfg_saved = false;
  _last_cfg_sent_time = 0;
  _num_cfg_save_tries = 0;
  _last_config_time = 0;
  _delay_time = 0;
  _next_message = STEP_RATE_NAV;
  _ublox_port = 255;
  _have_version = false;
  _unconfigured_messages = CONFIG_ALL;
  _hardware_generation = 0;
  _new_position = 0;
  _new_speed = 0;
  _disable_counter = 0;
  next_fix = NO_FIX;
  _last_5hz_time = 0;
  _cfg_needs_save = false;
  noReceivedHdop = true;

  CLEAR_BIT(huart_->Instance->CR1, USART_CR1_RE);
  _last_5hz_time = millis();

  /* change the rate */
  //STEP_RATE_NAV:
  _configure_rate();
  HAL_Delay(100);
  //STEP_RATE_POSLLH:
  _configure_message_rate(CLASS_NAV, MSG_POSLLH, RATE_POSLLH);
  HAL_Delay(100);
  //STEP_RATE_VELNED:
  _configure_message_rate(CLASS_NAV, MSG_VELNED, RATE_VELNED);
  HAL_Delay(100);
  //STEP_RATE_SOL:
  _configure_message_rate(CLASS_NAV, MSG_SOL, RATE_SOL);
  HAL_Delay(100);
  //STEP_RATE_PVT:
  _configure_message_rate(CLASS_NAV, MSG_PVT, RATE_PVT);
  HAL_Delay(100);

  //DMA
  //start usart revceive dma interrupt
  HAL_UART_Receive_DMA(huart_, getRxPointer(), getRxSize());
  huart_->hdmarx->XferCpltCallback = UBLOX_UART_DMAReceiveCpltUBLOX; //change the registerred func
  __HAL_UART_DISABLE_IT(huart_, UART_IT_RXNE);
  SET_BIT(huart_->Instance->CR1, USART_CR1_RE);
}

void
GPS::_verify_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate) {
  switch(msg_class) {
  case CLASS_NAV:
    switch(msg_id) {
    case MSG_POSLLH:
      if(rate == RATE_POSLLH) {
        _unconfigured_messages &= ~CONFIG_RATE_POSLLH;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_POSLLH);
        _unconfigured_messages |= CONFIG_RATE_POSLLH;
        _cfg_needs_save = true;
      }
      break;
    case MSG_STATUS:
      if(rate == RATE_STATUS) {
        _unconfigured_messages &= ~CONFIG_RATE_STATUS;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_STATUS);
        _unconfigured_messages |= CONFIG_RATE_STATUS;
        _cfg_needs_save = true;
      }
      break;
    case MSG_SOL:
      if(rate == RATE_SOL) {
        _unconfigured_messages &= ~CONFIG_RATE_SOL;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_SOL);
        _unconfigured_messages |= CONFIG_RATE_SOL;
        _cfg_needs_save = true;
      }
      break;
    case MSG_VELNED:
      if(rate == RATE_VELNED) {
        _unconfigured_messages &= ~CONFIG_RATE_VELNED;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_VELNED);
        _unconfigured_messages |= CONFIG_RATE_VELNED;
        _cfg_needs_save = true;
      }
      break;
    }
    break;
  case CLASS_MON:
    switch(msg_id) {
    case MSG_MON_HW:
      if(rate == RATE_HW) {
        _unconfigured_messages &= ~CONFIG_RATE_MON_HW;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_HW);
        _unconfigured_messages |= CONFIG_RATE_MON_HW;
        _cfg_needs_save = true;
      }
      break;
    case MSG_MON_HW2:
      if(rate == RATE_HW2) {
        _unconfigured_messages &= ~CONFIG_RATE_MON_HW2;
      } else {
        _configure_message_rate(msg_class, msg_id, RATE_HW2);
        _unconfigured_messages |= CONFIG_RATE_MON_HW2;
        _cfg_needs_save = true;
      }
      break;
    }
    break;
  }
}

void
GPS::_request_port(void)
{
  _send_message(CLASS_CFG, MSG_CFG_PRT, NULL, 0);
}

void
GPS::update()
{
 uint8_t data = 0;
 while (available() > 0)
 {
	 pop(data);
	 read(data);
 }

#ifdef RESET_CONFIG
  _reset_config();
  return;
#endif

#ifdef SAVE_CONFIG

  _save_cfg(); 
  return;
#endif

}

bool
GPS::read(uint8_t data)
{
  bool parsed = false;
   
  //debug
  LED2_L;


#if  defined (RESET_CONFIG) || defined(SAVE_CONFIG) 
  return false;
#endif

 reset:
  switch(_step) {
  case 1:
    if (PREAMBLE2 == data) {
      _step++;
      break;
    }
    _step = 0;
    /* no break */
  case 0:
    if(PREAMBLE1 == data)
      _step++;
    break;

  case 2:
    _step++;
    _class = data;
    _ck_b = _ck_a = data;                               // reset the checksum accumulators
    break;
  case 3:
    _step++;
    _ck_b += (_ck_a += data);                   // checksum byte
    _msg_id = data;
    break;
  case 4:
    _step++;
    _ck_b += (_ck_a += data);                   // checksum byte
    _payload_length = data;                             // payload length low byte
    break;
  case 5:
    _step++;
    _ck_b += (_ck_a += data);                   // checksum byte

    _payload_length += (uint16_t)(data<<8);
    if (_payload_length > sizeof(_buffer)) {
      // assume any payload bigger then what we know about is noise
      _payload_length = 0;
      _step = 0;
      goto reset;
    }
    _payload_counter = 0;                               // prepare to receive payload
    break;

    // Receive message data
    //
  case 6:
    _ck_b += (_ck_a += data);                   // checksum byte
    if (_payload_counter < sizeof(_buffer)) {
      _buffer.bytes[_payload_counter] = data;
    }
    if (++_payload_counter == _payload_length)
      _step++;
    break;

    // Checksum and message processing
    //
  case 7:
    _step++;
    if (_ck_a != data) {
      _step = 0;
      goto reset;
    }
    break;
  case 8:
    _step = 0;
    if (_ck_b != data) {
      break;                                                  // bad checksum
    } 

    if (_parse_gps()) {
      parsed = true;
    }
    break;
  }
  //}
  return parsed;
}

void GPS::unexpected_message(void)
{
  if (++_disable_counter == 0) {
    _configure_message_rate(_class, _msg_id, 0);
  }
}

bool
GPS::_parse_gps(void)
{

  if (_class == CLASS_ACK) {
    if(_msg_id == MSG_ACK_ACK) {
      switch(_buffer.ack.clsID) {
      case CLASS_CFG:
        switch(_buffer.ack.msgID) {
        case MSG_CFG_CFG:
          _cfg_saved = true;
          _cfg_needs_save = false;
          break;
        case MSG_CFG_GNSS:
          _unconfigured_messages &= ~CONFIG_GNSS;
          break;
        case MSG_CFG_MSG:
          break;
        case MSG_CFG_NAV_SETTINGS:
          _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
          break;
        case MSG_CFG_RATE:
          _unconfigured_messages &= ~CONFIG_RATE_NAV;
          break;
        case MSG_CFG_SBAS:
          _unconfigured_messages &= ~CONFIG_SBAS;
          break;
        }
        break;
      case CLASS_MON:
        switch(_buffer.ack.msgID) {
        case MSG_MON_HW:
          _unconfigured_messages &= ~CONFIG_RATE_MON_HW;
          break;
        case MSG_MON_HW2:
          _unconfigured_messages &= ~CONFIG_RATE_MON_HW2;
          break;
        }
      }
    }
    return false;
  }

  if (_class == CLASS_CFG) {
    switch(_msg_id) {
    case  MSG_CFG_NAV_SETTINGS:
      _buffer.nav_settings.mask = 0;
      if (_navfilter != GPS_ENGINE_NONE &&
          _buffer.nav_settings.dynModel != _navfilter) {
        _buffer.nav_settings.dynModel = _navfilter;
        _buffer.nav_settings.mask |= 1;
      }
      if (_min_elevation != -100 &&
          _buffer.nav_settings.minElev != _min_elevation) {
        _buffer.nav_settings.minElev = _min_elevation;
        _buffer.nav_settings.mask |= 2;
      }
      if (_buffer.nav_settings.mask != 0) {
        _send_message(CLASS_CFG, MSG_CFG_NAV_SETTINGS,
                      &_buffer.nav_settings,
                      sizeof(_buffer.nav_settings));
        _unconfigured_messages |= CONFIG_NAV_SETTINGS;
        _cfg_needs_save = true;
      } else {
        _unconfigured_messages &= ~CONFIG_NAV_SETTINGS;
      }
      return false;

#if UBLOX_GNSS_SETTINGS
    case MSG_CFG_GNSS:
      if (_gnss_mode != 0) {
        struct ubx_cfg_gnss start_gnss = _buffer.gnss;
        uint8_t gnssCount = 0;

        for(int i = 0; i < UBLOX_MAX_GNSS_CONFIG_BLOCKS; i++) {
          if((_gnss_mode & (1 << i)) && i != GNSS_SBAS) {
            gnssCount++;
          }
        }

        for(int i = 0; i < _buffer.gnss.numConfigBlocks; i++) {
          // Reserve an equal portion of channels for all enabled systems
          if(_gnss_mode & (1 << _buffer.gnss.configBlock[i].gnssId)) {
            if(GNSS_SBAS !=_buffer.gnss.configBlock[i].gnssId) {
              _buffer.gnss.configBlock[i].resTrkCh = (_buffer.gnss.numTrkChHw - 3) / (gnssCount * 2);
              _buffer.gnss.configBlock[i].maxTrkCh = _buffer.gnss.numTrkChHw;
            } else {
              _buffer.gnss.configBlock[i].resTrkCh = 1;
              _buffer.gnss.configBlock[i].maxTrkCh = 3;
            }
            _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags | 0x00000001;
          } else {
            _buffer.gnss.configBlock[i].resTrkCh = 0;
            _buffer.gnss.configBlock[i].maxTrkCh = 0;
            _buffer.gnss.configBlock[i].flags = _buffer.gnss.configBlock[i].flags & 0xFFFFFFFE;
          }
        }
        if (!memcmp(&start_gnss, &_buffer.gnss, sizeof(start_gnss))) {
          _send_message(CLASS_CFG, MSG_CFG_GNSS, &_buffer.gnss, 4 + (8 * _buffer.gnss.numConfigBlocks));
          _unconfigured_messages |= CONFIG_GNSS;
          _cfg_needs_save = true;
        } else {
          _unconfigured_messages &= ~CONFIG_GNSS;
        }
      } else {
        _unconfigured_messages &= ~CONFIG_GNSS;
      }
      return false;
#endif

    case MSG_CFG_SBAS:
      if (_sbas_mode != 2) {
        if (_buffer.sbas.mode != _sbas_mode) {
          _buffer.sbas.mode = _sbas_mode;
          _send_message(CLASS_CFG, MSG_CFG_SBAS,
                        &_buffer.sbas,
                        sizeof(_buffer.sbas));
          _unconfigured_messages |= CONFIG_SBAS;
          _cfg_needs_save = true;
        } else {
          _unconfigured_messages &= ~CONFIG_SBAS;
        }
      } else {
        _unconfigured_messages &= ~CONFIG_SBAS;
      }
      return false;
    case MSG_CFG_MSG:
      if(_payload_length == sizeof(ubx_cfg_msg_rate_6)) {
        if(_ublox_port >= UBLOX_MAX_PORTS) {
          _request_port();
          return false;
        }
        _verify_rate(_buffer.msg_rate_6.msg_class, _buffer.msg_rate_6.msg_id,
                     _buffer.msg_rate_6.rates[_ublox_port]);
      } else {
        _verify_rate(_buffer.msg_rate.msg_class, _buffer.msg_rate.msg_id,
                     _buffer.msg_rate.rate);
      }
      return false;
    case MSG_CFG_PRT:
      _ublox_port = _buffer.prt.portID;
      return false;
    case MSG_CFG_RATE:
      if(_buffer.nav_rate.measure_rate_ms != MEASURE_RATE ||
         _buffer.nav_rate.nav_rate != 1 ||
         _buffer.nav_rate.timeref != 0) {
        _configure_rate();
        _unconfigured_messages |= CONFIG_RATE_NAV;
        _cfg_needs_save = true;
      } else {
        _unconfigured_messages &= ~CONFIG_RATE_NAV;
      }
      return false;
    }
  }

  if (_class == CLASS_MON) {
    switch(_msg_id) {
    case MSG_MON_VER:
      _have_version = true;
      break;
    default:
      unexpected_message();
    }
    return false;
  }

  if (_class != CLASS_NAV) {
    unexpected_message();
    return false;
  }

  switch (_msg_id) {
  case MSG_PVT:
	LED2_H;
    _last_pos_time        = _buffer.pvt.time;
    state.location.lng    = _buffer.pvt.longitude;
    state.location.lat    = _buffer.pvt.latitude;
    state.location.alt    = _buffer.pvt.altitude_msl / 10;

    state.status = _buffer.pvt.fix_type;

    _new_position = true;
    state.horizontal_accuracy = _buffer.pvt.horizontal_accuracy*1.0e-3f;
    state.vertical_accuracy = _buffer.pvt.vertical_accuracy*1.0e-3f;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;

    _new_speed= true;
    state.ground_speed     = _buffer.pvt.speed_2d*0.001f;          // m/s
    state.ground_course_cd = (_buffer.pvt.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
    state.have_vertical_velocity = true;
    state.velocity.x = _buffer.pvt.ned_north * 0.001f;
    state.velocity.y = _buffer.pvt.ned_east * 0.001f;
    state.velocity.z = _buffer.pvt.ned_down * 0.001f;
    state.have_speed_accuracy = true;
    state.speed_accuracy = _buffer.pvt.speed_accuracy*0.001f;
    state.hdop        = _buffer.pvt.position_DOP;
    state.num_sats    = _buffer.pvt.satellites;

    /* not synchronized update */
    update_ = true;

    return true;
  case MSG_POSLLH:

    _last_pos_time        = _buffer.posllh.time;
    state.location.lng    = _buffer.posllh.longitude;
    state.location.lat    = _buffer.posllh.latitude;
    state.location.alt    = _buffer.posllh.altitude_msl / 10;
    state.status          = next_fix;
    _new_position = true;
    state.horizontal_accuracy = _buffer.posllh.horizontal_accuracy*1.0e-3f;
    state.vertical_accuracy = _buffer.posllh.vertical_accuracy*1.0e-3f;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    /* not synchronized update */
    update_ = true;

    return true;
    //break;
  case MSG_SOL:

    if (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) {
      if( (_buffer.solution.fix_type == GPS::FIX_3D) &&
          (_buffer.solution.fix_status & GPS::NAV_STATUS_DGPS_USED)) {
        next_fix = GPS_OK_FIX_3D_DGPS;
      }else if( _buffer.solution.fix_type == GPS::FIX_3D) {
        next_fix = GPS_OK_FIX_3D;
      }else if (_buffer.solution.fix_type == GPS::FIX_2D) {
        next_fix = GPS_OK_FIX_2D;
      }else{
        next_fix = NO_FIX;
        state.status = NO_FIX;
      }
    }else{
      next_fix = NO_FIX;
      state.status = NO_FIX;
    }
    if(noReceivedHdop) {
      state.hdop = _buffer.solution.position_DOP;
    }
    state.num_sats    = _buffer.solution.satellites;
    if (next_fix >= GPS_OK_FIX_2D) {
      state.last_gps_time_ms = millis();
      if (state.time_week == _buffer.solution.week &&
          state.time_week_ms + 200 == _buffer.solution.time) {
        _last_5hz_time = state.last_gps_time_ms;
      }
      state.time_week_ms    = _buffer.solution.time;
      state.time_week       = _buffer.solution.week;
    }
    return true;
    //break;
  case MSG_VELNED:

    _last_vel_time         = _buffer.velned.time;
    state.ground_speed     = _buffer.velned.speed_2d*0.01f;          // m/s
    state.ground_course_cd = (_buffer.velned.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
    state.have_vertical_velocity = true;
    state.velocity.x = _buffer.velned.ned_east * 0.01f;
    state.velocity.y = _buffer.velned.ned_east * 0.01f;
    state.velocity.z = _buffer.velned.ned_down * 0.01f;
    state.have_speed_accuracy = true;
    state.speed_accuracy = _buffer.velned.speed_accuracy*0.01f;
#if UBLOX_FAKE_3DLOCK
    state.speed_accuracy = 0;
#endif
    _new_speed = true;
    return true;
    // break;
  case MSG_NAV_SVINFO:
    {
      static const uint8_t HardwareGenerationMask = 0x07;
      _hardware_generation = _buffer.svinfo_header.globalFlags & HardwareGenerationMask;
      switch (_hardware_generation) {
      case UBLOX_5:
      case UBLOX_6:
        // only 7 and newer support CONFIG_GNSS
        _unconfigured_messages &= ~CONFIG_GNSS;
        break;
      case UBLOX_7:
      case UBLOX_M8:
        break;
      default:
        break;
      };
      _unconfigured_messages &= ~CONFIG_VERSION;
      _configure_message_rate(CLASS_NAV, MSG_NAV_SVINFO, 0);
      break;
    }
  default:
    if (++_disable_counter == 0) {
      _configure_message_rate(CLASS_NAV, _msg_id, 0);
    }
    return false;
  }
  if (_new_position && _new_speed && _last_vel_time == _last_pos_time) {
    _new_speed = _new_position = false;

    return true;
  }

  return false;
}

void
GPS::_update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
  while (len--) {
    ck_a += *data;
    ck_b += ck_a;
    data++;
  }
}

void
GPS::_send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size)
{
  struct ubx_header header;
  uint8_t ck_a=0, ck_b=0;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id    = msg_id;
  header.length    = size;

  _update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
  _update_checksum((uint8_t *)msg, size, ck_a, ck_b);

  write((const uint8_t *)&header, sizeof(header));
  write((const uint8_t *)msg, size);
  write((const uint8_t *)&ck_a, 1);
  write((const uint8_t *)&ck_b, 1);
}

bool
GPS::_request_message_rate(uint8_t msg_class, uint8_t msg_id)
{
  if(_ublox_port >= UBLOX_MAX_PORTS) {
    _request_port();
    return false;
  } else {
    struct ubx_cfg_msg msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
    return true;
  }
}

bool
GPS::_configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  struct ubx_cfg_msg_rate msg;
  msg.msg_class = msg_class;
  msg.msg_id    = msg_id;
  msg.rate          = rate;
  _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
  return true;
}

void
GPS::_request_navigation_rate(void)
{
  _send_message(CLASS_CFG, MSG_CFG_RATE, 0, 0);
}

void
GPS::_save_cfg()
{
  ubx_cfg_cfg save_cfg;
  save_cfg.clearMask = 0;
  save_cfg.saveMask = SAVE_CFG_ALL;
  save_cfg.loadMask = 0;
  _send_message(CLASS_CFG, MSG_CFG_CFG, &save_cfg, sizeof(save_cfg));
  _last_cfg_sent_time = millis();
  _num_cfg_save_tries++;
}

void
GPS::_request_version(void)
{
  _send_message(CLASS_MON, MSG_MON_VER, NULL, 0);
}

void
GPS::_configure_rate(void)
{
  struct ubx_cfg_nav_rate msg;
  msg.measure_rate_ms = MEASURE_RATE;
  msg.nav_rate        = 1;
  msg.timeref         = 0;     // UTC time
  _send_message(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}

void
GPS::_reset_config(void)
{
  char reset_config[] = UBLOX_SET_BINARY;
  write((const uint8_t *)reset_config, sizeof(reset_config));
  _save_cfg();
}


void
GPS::UBLOX_UART_DMAReceiveCpltUBLOX(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* DMA Normal mode */
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
    {
      huart->RxXferCount = 0;

      /* Disable the DMA transfer for the receiver request by setting the DMAR bit 
         in the UART CR3 register */
      huart->Instance->CR3 &= (uint32_t)~((uint32_t)USART_CR3_DMAR);

      /* Check if a transmit Process is ongoing or not */
      if(huart->State == HAL_UART_STATE_BUSY_TX_RX)
        {
          huart->State = HAL_UART_STATE_BUSY_TX;
        }
      else
        {
          huart->State = HAL_UART_STATE_READY;
        }
    }

  for(std::size_t i = 0; i < gps_rx_size_; i++)
    {
      if(!gps_rx_buf_.push(gps_rx_value_[i]))
        return;
    }
}
