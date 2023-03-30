#include <kondo.h>

void setPosition(kondo_servo_t* const this, uint16_t target_position){
	int tx_size = 3, rx_size = 3;
	uint8_t tx_buff[tx_size], rx_buff[rx_size];
	uint8_t ret, id;
	uint16_t current_position;

	tx_buff[0] = 0x80 + this->id_;
	tx_buff[1] = (uint8_t)((target_position&0x3f80) >> 7);
	tx_buff[2] = (uint8_t)(target_position&0x007f);

	HAL_HalfDuplex_EnableTransmitter(&this->port_);
	ret = HAL_UART_Transmit(&this->port_, tx_buff, tx_size, 1);
	if(ret == HAL_OK){
		HAL_HalfDuplex_EnableReceiver(&this->port_);
	}
	ret = HAL_UART_Receive(&this->port_, rx_buff, rx_size, 1);

	this->target_position_ = target_position;
	id = rx_buff[0] & 0x1f;
	current_position =  (uint16_t)((0x7f & rx_buff[1]) << 7) + (uint16_t)(0x7f & rx_buff[2]);
	if(id == this->id_){
		this->current_position_ = current_position;
	}
}

uint16_t readPosition(kondo_servo_t* const this){
	int tx_size = 2, rx_size = 4;
	uint8_t tx_buff[tx_size], rx_buff[rx_size];
	uint8_t ret, id;
	tx_buff[0] = 0xA0 + this->id_;
	tx_buff[1] = 0x05;

	HAL_HalfDuplex_EnableTransmitter(&this->port_);
	ret = HAL_UART_Transmit(&this->port_, tx_buff, tx_size, 1);
	if(ret == HAL_OK){
		HAL_HalfDuplex_EnableReceiver(&this->port_);
	}
	ret = HAL_UART_Receive(&this->port_, rx_buff, rx_size, 1);

	id = rx_buff[0] & 0x1f;
	if(id == this->id_){
		uint16_t current_position = (uint16_t)((0x7f & rx_buff[2]) << 7) + (uint16_t)(0x7f & rx_buff[3]);
		this->current_position_ = current_position;
		return current_position;
	} else return this->current_position_;
}


void free(kondo_servo_t* const this){
	setPosition(this, 0);
}

void hold(kondo_servo_t* const this){
	setPosition(this, readPosition(this));
}


uint8_t getId(kondo_servo_t* const this) {return this->id_;}
uint32_t getBaudrate(kondo_servo_t* const this) {return this->baudrate_;}
uint8_t getReverse(kondo_servo_t* const this) {return this->reverse_;}
UART_HandleTypeDef getPort(kondo_servo_t* const this) {return this->port_;}
uint8_t getCurrentAngle(kondo_servo_t* const this) {return this->current_angle_;}
uint8_t getTargetAngle(kondo_servo_t* const this) {return this->target_angle_;}
