/*
 serial interface between the basic controller board for attitude control and the superior controller for flight control.

 lev.0 is inplemented in C using termios.h + fcntl.h

*/



//* the relative header file 
#include "jsk_quadcopter/basic_controller_superior_controller_interface_lev0.h"
//* additional header file from C to support the C++ interface code
extern "C" {
  #include <unistd.h>
  #include <fcntl.h>
}


SerialInterface::SerialInterface (std::string port, uint32_t speed, int every_byte_interval)
  :serialPortName (port), serialPortSpeed (speed), everyByteInterval(every_byte_interval)
  {
    struct termios tio;
      status = false;
      serialPortBaud = bitrate (serialPortSpeed);
      ROS_INFO ("Initializing serial port...");

      serialDev = open(serialPortName.c_str (),O_RDWR | O_NOCTTY | O_NDELAY);
      ROS_DEBUG ("dev: %d", serialDev);
      ROS_ASSERT_MSG (serialDev != -1, "Failed to open serial port: %s %s", serialPortName.c_str (), strerror (errno));

      ROS_ASSERT_MSG (tcgetattr (serialDev, &tio) == 0, "Unknown Error: %s", strerror (errno));

      cfsetispeed (&tio, serialPortBaud);
      cfsetospeed (&tio, serialPortBaud);

      tio.c_iflag = 0;
      tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
      tio.c_iflag |= IGNBRK;

      tio.c_oflag = 0;
      tio.c_oflag &= ~(OPOST | ONLCR);

      tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
      tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

      tio.c_lflag = 0;
      tio.c_lflag |= NOFLSH;
      tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

      ROS_ASSERT_MSG (tcsetattr (serialDev, TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));

      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;

      tcflush (serialDev, TCIOFLUSH);

      ROS_ASSERT_MSG (&serialDev != NULL, "Could not open serial port %s", serialPortName.c_str ());
      ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialPortName.c_str (), serialPortSpeed);
  }


SerialInterface::~SerialInterface ()
{
  ROS_DEBUG ("  deleted Serial Interface");
  txRxFlush ();
  close (serialDev);
}


void SerialInterface::txRxFlush ()
{
  tcflush (serialDev, TCIOFLUSH);
}

void SerialInterface::txFlush ()
{
  tcflush (serialDev, TCOFLUSH);
}

void SerialInterface::rxFlush ()
{
  tcflush (serialDev, TCIFLUSH);
}

void SerialInterface::drain ()
{
  ROS_ASSERT_MSG (tcdrain (serialDev) == 0, "Drain Error: %s", strerror (errno));
}


int SerialInterface::wait (int bytes_requested, float timeout)
{
  int bytes_available=0;
  double begin_secs =ros::Time::now().toSec();

  while (bytes_available < bytes_requested)
    {
      ioctl(serialDev, FIONREAD, &bytes_available);
      //usleep(1);
      double timeout_ms =ros::Time::now().toSec() - begin_secs;
      if (timeout_ms  > timeout && bytes_available < bytes_requested)
	{
	  // ROS_WARN("Timeout: %d bytes available %d bytes requested", bytes_available,bytes_requested);

	  // double diff_secs =ros::Time::now().toSec() - begin_secs;
	  // ROS_INFO("            latency is %f", diff_secs);
	  return 0; 
	}
    }
  return 1; //変更
}


speed_t SerialInterface::bitrate (int Bitrate)
{
  switch (Bitrate)
    {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:                 // invalid bitrate
      return B57600;         //あとで直す
    }
}

void SerialInterface::output (uint8_t *output, int len)
{
  int i;
  ROS_DEBUG ("SerialInterface::output()");

  //txFlush
  //ROS_INFO ("Writing %d element(s): %s", len, output);
  //ROS_DEBUG ("dev: %zd", (size_t) serialDev);
  //ROS_DEBUG ("FOO");
  i = write (serialDev, output, len);
  if (i != len)
    {
      //ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      ROS_BREAK ();
    }
}




void SerialInterface::sendPacket (const int& packet_type, Navigator* navigator, Estimator* estimator, FlightCtrlInput* flight_ctrl_input)
{
  uint8_t tx_buffer[256]; //15 + 1('\0')

  if(packet_type == FLIGHT_CONTROL)
    {
      tx_buffer[0] = '>';
      tx_buffer[1] = '*';
      tx_buffer[2] = '>';

      if(navigator->getNaviCommand() == navigator->START_COMMAND)
	{ 
	  uint16_t posZ = 1000 * estimator->getStatePosZ();
	  //*** バッファーを綺麗にする
	  txRxFlush();
	  tx_buffer[3] = 2; // height
	  tx_buffer[4] = PD_START_COMMAND;  
	  memcpy(&tx_buffer[5], (uint8_t*) &posZ, 2);
	  uint16_t chk = crc16(&tx_buffer[3], 4); // packet size & packet type & packet data
	  tx_buffer[7] = (uint8_t)(chk & 0xff);
	  tx_buffer[8] = (uint8_t)(chk >> 8);
	  output(tx_buffer, 9); //3 + 2 + 2 + 2 
	  //debug
	  ROS_INFO("START_COMMAND");
	}
      else if(navigator->getNaviCommand() == navigator->STOP_COMMAND)
	{ 
	  uint16_t posZ = 1000 * estimator->getStatePosZ();
	  //*** バッファーを綺麗にする
	  //txRxFlush();
	  tx_buffer[3] = 2; // height
	  tx_buffer[4] = PD_STOP_COMMAND;  
	  memcpy(&tx_buffer[5], (uint8_t*) &posZ, 2);
	  uint16_t chk = crc16(&tx_buffer[3], 4); 
	  tx_buffer[7] = (uint8_t)(chk & 0xff);
	  tx_buffer[8] = (uint8_t)(chk >> 8);
	  output(tx_buffer, 9); //3 + 2 + 2 + 2 
	  //debug
	  ROS_INFO("STOP_COMMAND");
	}
      else if(navigator->getNaviCommand() == navigator->TAKEOFF_COMMAND ||
	      navigator->getNaviCommand() == navigator->LAND_COMMAND ||
	      navigator->getNaviCommand() == navigator->CTRL_COMMAND ||
	      navigator->getNaviCommand() == navigator->CTRL_ACK_COMMAND ||
	      navigator->getNaviCommand() == navigator->HOVER_COMMAND)
	{
	  tx_buffer[3] = 8;
	  tx_buffer[4] = PD_CTRL_COMMAND;
	  memcpy(&tx_buffer[5], flight_ctrl_input->getCtrlInputArray(), 
		 flight_ctrl_input->getCtrlInputSize()); 
	  uint16_t chk = crc16(&tx_buffer[3], 10); // packet size & packet type & packet dat
	  tx_buffer[13] = (uint8_t)(chk & 0xff);
	  tx_buffer[14] = (uint8_t)(chk >> 8);
	  output(tx_buffer, 15); //3 + 2 + 8 + 2 
          
          //ROS_INFO("Flight_COMMAND");
	}
      else
	{
	  //ROS_ERROR("ERROR PISITION COMMAND, CAN NOT BE SEND TO Quadcopter");
	}
    }
}


bool SerialInterface::getPacket (uint8_t* spacket, uint8_t* packet_type,  uint16_t* packet_size, ros::Time& packet_time_stamp)
{
  char end_bytes[3]; // ompare with <#<
  uint8_t start_byte[1];
  uint8_t ssize[2];
  uint8_t stype[1];
  uint8_t scrc[2];

  int i,syncstate=0;

  uint16_t packet_crc;
  //uint16_t packet_size;

  //txRxFlush();

  while(1){//this is the simple way of serial communication
    drain();
    // get beginning (">*>")
    if(syncstate == 0){
      if(wait(1)){
	i = read (serialDev, start_byte, 1);

	if (start_byte[0]=='>'){
	  syncstate++;
	}else{
	  syncstate=0;
	  //ROS_WARN (" Error Reading Packet Of First Header: %s", strerror (errno));
	}
      }else{
	//ROS_WARN ("  Critical Error Reading Packet Of First Header: %s", strerror (errno));
	return false;
      }
    }

    if(syncstate == 1){
      if(wait(1)){
	i = read (serialDev,start_byte, 1);

	if (start_byte[0]=='*' && i!=0){
	  syncstate++;
	}else{
	  syncstate=0;
	  ROS_WARN ("Error Reading Packet Of Second Header: %s", strerror (errno));
	}
      }else{
	ROS_WARN ("    Critical Error Reading Packet Of Second Header: %s", strerror (errno));
	return false;
      }
    }


    if(syncstate == 2){
      if(wait(1)){
	i = read (serialDev,start_byte, 1);

	if (start_byte[0]=='>' && i!=0){

	  syncstate++;
	}else{
	  syncstate=0;
	  ROS_WARN ("    Error Reading Packet Of Third Header: %s", strerror (errno));
	}
      }else{
	ROS_WARN ("     Critical Error Reading Packet Of Third Header: %s", strerror (errno));
	return false;
      }
    }


    // get packet size
    if(syncstate==3){
      if(wait(sizeof(*packet_size))){
	i = read (serialDev,ssize, sizeof(*packet_size));
	if (*(unsigned short*)ssize >0){
	  syncstate++;
	  memcpy (packet_size, ssize, sizeof(*packet_size));
	}else{
	  syncstate=0;
	  ROS_WARN("   Error Reading Packet Size: %s", strerror (errno));
	}
      }else{
	return false;
	ROS_WARN ("    Critical Error Reading Packet Size: %s", strerror (errno));
      }
    }

    // get packet size
    if(syncstate==4){
      if(wait(sizeof(*packet_type))){
	i = read (serialDev,stype, sizeof(*packet_type));
	if (*(unsigned uint8_t*)stype >0){
	  syncstate++;
	  memcpy (packet_type, stype, sizeof(*packet_type));
	}else{
	  syncstate=0;
	  ROS_WARN ("   Error Reading Packet Type: %s", strerror (errno));
	}
      }else{
	return false;
	ROS_WARN ("    Critical Error Reading Packet Type: %s", strerror (errno));
      }
    }

    // get packet
    if(syncstate==5){
      if(wait(*packet_size)){
	i = read (serialDev, spacket, *packet_size);
	if(i !=0){
	  syncstate++;
	}else{
	  syncstate=0;
	  ROS_WARN ("   Error Reading Packet Packet Data: %s", strerror (errno));
	}
      }else{
	return false;	
	ROS_WARN ("   Critical Error Reading Packet Packet Data: %s", strerror (errno));
      }
    }
       
    // get packet crc
    if(syncstate==6){
      if(wait(2)){
	i = read (serialDev, scrc, 2);
	if (i != 0){
	  memcpy (&packet_crc, scrc, sizeof (packet_crc));
	  syncstate++;
	}else{
	  syncstate=0;
	  ROS_WARN ("   Error Reading Packet CRC: %s", strerror (errno));
	}
      }else{
	ROS_WARN ("   Critical Error Reading Packet CRC: %s", strerror (errno));
	return false;
      }
    }

    // get closing ("<#<")
    if(syncstate==7){
      if(wait(3)){
	i = read (serialDev, end_bytes, 3);
	if (i !=0 && strncmp (end_bytes, "<#<", 3) == 0){
          packet_time_stamp = ros::Time::now();
	  break;
	}else{
	  ROS_WARN ("Error Reading Packet Footer: %s", strerror (errno));
	  ROS_WARN (" stop string is %s", end_bytes);
	  syncstate = 0;
	}
      }else{
	ROS_WARN ("Critical Error Reading Packet Footer: %s", strerror (errno));
	return false;
      }
    }
  }



  return (crc_valid(packet_crc, spacket, *packet_size));
}

