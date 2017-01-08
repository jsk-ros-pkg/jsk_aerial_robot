#include "uav_comm/xbee/xbee.h"

namespace xbee
{

  XBeeResponse::XBeeResponse() {
  }

  uint8_t XBeeResponse::getApiId() {
    return _apiId;
  }

  void XBeeResponse::setApiId(uint8_t apiId) {
    _apiId = apiId;
  }

  uint8_t XBeeResponse::getMsbLength() {
    return _msbLength;
  }

  void XBeeResponse::setMsbLength(uint8_t msbLength) {
    _msbLength = msbLength;
  }

  uint8_t XBeeResponse::getLsbLength() {
    return _lsbLength;
  }

  void XBeeResponse::setLsbLength(uint8_t lsbLength) {
    _lsbLength = lsbLength;
  }

  uint8_t XBeeResponse::getChecksum() {
    return _checksum;
  }

  void XBeeResponse::setChecksum(uint8_t checksum) {
    _checksum = checksum;
  }

  uint8_t XBeeResponse::getFrameDataLength() {
    return _frameLength;
  }

  void XBeeResponse::setFrameLength(uint8_t frameLength) {
    _frameLength = frameLength;
  }

  bool XBeeResponse::isAvailable() {
    return _complete;
  }

  void XBeeResponse::setAvailable(bool complete) {
    _complete = complete;
  }

  bool XBeeResponse::isError() {
    return _errorCode > 0;
  }

  uint8_t XBeeResponse::getErrorCode() {
    return _errorCode;
  }

  void XBeeResponse::setErrorCode(uint8_t errorCode) {
    _errorCode = errorCode;
  }

  // copy common fields from xbee response to target response
  void XBeeResponse::setCommon(XBeeResponse &target) {
    target.setApiId(getApiId());
    target.setAvailable(isAvailable());
    target.setChecksum(getChecksum());
    target.setErrorCode(getErrorCode());
    target.setFrameLength(getFrameDataLength());
    target.setMsbLength(getMsbLength());
    target.setLsbLength(getLsbLength());
  }

  ZBTxStatusResponse::ZBTxStatusResponse() : FrameIdResponse() {

  }

  uint16_t ZBTxStatusResponse::getRemoteAddress() {
    return  (getFrameData()[1] << 8) + getFrameData()[2];
  }

  uint8_t ZBTxStatusResponse::getTxRetryCount() {
    return getFrameData()[3];
  }

  uint8_t ZBTxStatusResponse::getDeliveryStatus() {
    return getFrameData()[4];
  }

  uint8_t ZBTxStatusResponse::getDiscoveryStatus() {
    return getFrameData()[5];
  }

  bool ZBTxStatusResponse::isSuccess() {
    return getDeliveryStatus() == SUCCESS;
  }

  void XBeeResponse::getZBTxStatusResponse(XBeeResponse &zbXBeeResponse) {

    // way off?
    ZBTxStatusResponse* zb = static_cast<ZBTxStatusResponse*>(&zbXBeeResponse);
    // pass pointer array to subclass
    zb->setFrameData(getFrameData());
    setCommon(zbXBeeResponse);
  }

  ZBRxResponse::ZBRxResponse(): RxDataResponse() {
    _remoteAddress64 = XBeeAddress64();
  }

  uint16_t ZBRxResponse::getRemoteAddress16() {
    return 	(getFrameData()[8] << 8) + getFrameData()[9];
  }

  uint8_t ZBRxResponse::getOption() {
    return getFrameData()[10];
  }

  // markers to read data from packet array.  this is the index, so the 12th item in the array
  uint8_t ZBRxResponse::getDataOffset() {
    return 11;
  }

  uint8_t ZBRxResponse::getDataLength() {
    return getPacketLength() - getDataOffset() - 1;
  }

  XBeeAddress64& ZBRxResponse::getRemoteAddress64() {
    return _remoteAddress64;
  }

  void XBeeResponse::getZBRxResponse(XBeeResponse &rxResponse) {

    ZBRxResponse* zb = static_cast<ZBRxResponse*>(&rxResponse);

    //TODO verify response api id matches this api for this response

    // pass pointer array to subclass
    zb->setFrameData(getFrameData());
    setCommon(rxResponse);

    zb->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
    zb->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + (getFrameData()[7]));
  }


  ZBRxIoSampleResponse::ZBRxIoSampleResponse() : ZBRxResponse() {

  }

  // 64 + 16 addresses, sample size, option = 12 (index 11), so this starts at 12
  uint8_t ZBRxIoSampleResponse::getDigitalMaskMsb() {
    return getFrameData()[12] & 0x1c;
  }

  uint8_t ZBRxIoSampleResponse::getDigitalMaskLsb() {
    return getFrameData()[13];
  }

  uint8_t ZBRxIoSampleResponse::getAnalogMask() {
    return getFrameData()[14] & 0x8f;
  }

  bool ZBRxIoSampleResponse::containsAnalog() {
    return getAnalogMask() > 0;
  }

  bool ZBRxIoSampleResponse::containsDigital() {
    return getDigitalMaskMsb() > 0 || getDigitalMaskLsb() > 0;
  }

  bool ZBRxIoSampleResponse::isAnalogEnabled(uint8_t pin) {
    return ((getAnalogMask() >> pin) & 1) == 1;
  }

  bool ZBRxIoSampleResponse::isDigitalEnabled(uint8_t pin) {
    if (pin <= 7) {
      // added extra parens to calm avr compiler
      return ((getDigitalMaskLsb() >> pin) & 1) == 1;
    } else {
      return ((getDigitalMaskMsb() >> (pin - 8)) & 1) == 1;
    }
  }

  uint16_t ZBRxIoSampleResponse::getAnalog(uint8_t pin) {
    // analog starts 13 bytes after sample size, if no dio enabled
    uint8_t start = 15;

    if (containsDigital()) {
      // make room for digital i/o
      start+=2;
    }

    //	std::cout << "spacing is " << static_cast<unsigned int>(spacing) << std::endl;

    // start depends on how many pins before this pin are enabled
    for (int i = 0; i < pin; i++) {
      if (isAnalogEnabled(i)) {
	start+=2;
      }
    }

    return (uint16_t)((getFrameData()[start] << 8) + getFrameData()[start + 1]);
  }

  bool ZBRxIoSampleResponse::isDigitalOn(uint8_t pin) {
    if (pin <= 7) {
      // D0-7
      // DIO LSB is index 5
      return ((getFrameData()[16] >> pin) & 1) == 1;
    } else {
      // D10-12
      // DIO MSB is index 4
      return ((getFrameData()[15] >> (pin - 8)) & 1) == 1;
    }
  }

  void XBeeResponse::getZBRxIoSampleResponse(XBeeResponse &response) {
    ZBRxIoSampleResponse* zb = static_cast<ZBRxIoSampleResponse*>(&response);


    // pass pointer array to subclass
    zb->setFrameData(getFrameData());
    setCommon(response);

    zb->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
    zb->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + (getFrameData()[7]));
  }


  RemoteAtCommandResponse::RemoteAtCommandResponse() : AtCommandResponse() {

  }

  uint8_t* RemoteAtCommandResponse::getCommand() {
    return getFrameData() + 11;
  }

  uint8_t RemoteAtCommandResponse::getStatus() {
    return getFrameData()[13];
  }

  bool RemoteAtCommandResponse::isOk() {
    // weird c++ behavior.  w/o this method, it calls AtCommandResponse::isOk(), which calls the AtCommandResponse::getStatus, not this.getStatus!!!
    return getStatus() == AT_OK;
  }

  uint8_t RemoteAtCommandResponse::getValueLength() {
    return getFrameDataLength() - 14;
  }

  uint8_t* RemoteAtCommandResponse::getValue() {
    if (getValueLength() > 0) {
      // value is only included for query commands.  set commands does not return a value
      return getFrameData() + 14;
    }

    return NULL;
  }

  uint16_t RemoteAtCommandResponse::getRemoteAddress16() {
    return uint16_t((getFrameData()[9] << 8) + getFrameData()[10]);
  }

  XBeeAddress64& RemoteAtCommandResponse::getRemoteAddress64() {
    return _remoteAddress64;
  }

  void XBeeResponse::getRemoteAtCommandResponse(XBeeResponse &response) {

    // TODO no real need to cast.  change arg to match expected class
    RemoteAtCommandResponse* at = static_cast<RemoteAtCommandResponse*>(&response);

    // pass pointer array to subclass
    at->setFrameData(getFrameData());
    setCommon(response);

    at->getRemoteAddress64().setMsb((uint32_t(getFrameData()[1]) << 24) + (uint32_t(getFrameData()[2]) << 16) + (uint16_t(getFrameData()[3]) << 8) + getFrameData()[4]);
    at->getRemoteAddress64().setLsb((uint32_t(getFrameData()[5]) << 24) + (uint32_t(getFrameData()[6]) << 16) + (uint16_t(getFrameData()[7]) << 8) + (getFrameData()[8]));

  }

  RxDataResponse::RxDataResponse() : XBeeResponse() {

  }

  uint8_t RxDataResponse::getData(int index) {
    return getFrameData()[getDataOffset() + index];
  }

  uint8_t* RxDataResponse::getData() {
    return getFrameData() + getDataOffset();
  }

  FrameIdResponse::FrameIdResponse() {

  }

  uint8_t FrameIdResponse::getFrameId() {
    return getFrameData()[0];
  }


  ModemStatusResponse::ModemStatusResponse() {

  }

  uint8_t ModemStatusResponse::getStatus() {
    return getFrameData()[0];
  }

  void XBeeResponse::getModemStatusResponse(XBeeResponse &modemStatusResponse) {

    ModemStatusResponse* modem = static_cast<ModemStatusResponse*>(&modemStatusResponse);

    // pass pointer array to subclass
    modem->setFrameData(getFrameData());
    setCommon(modemStatusResponse);

  }

  AtCommandResponse::AtCommandResponse() {

  }

  uint8_t* AtCommandResponse::getCommand() {
    return getFrameData() + 1;
  }

  uint8_t AtCommandResponse::getStatus() {
    return getFrameData()[3];
  }

  uint8_t AtCommandResponse::getValueLength() {
    return getFrameDataLength() - 4;
  }

  uint8_t* AtCommandResponse::getValue() {
    if (getValueLength() > 0) {
      // value is only included for query commands.  set commands does not return a value
      return getFrameData() + 4;
    }

    return NULL;
  }

  bool AtCommandResponse::isOk() {
    return getStatus() == AT_OK;
  }

  void XBeeResponse::getAtCommandResponse(XBeeResponse &atCommandResponse) {

    AtCommandResponse* at = static_cast<AtCommandResponse*>(&atCommandResponse);

    // pass pointer array to subclass
    at->setFrameData(getFrameData());
    setCommon(atCommandResponse);
  }

  uint16_t XBeeResponse::getPacketLength() {
    return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
  }

  uint8_t* XBeeResponse::getFrameData() {
    return _frameDataPtr;
  }

  void XBeeResponse::setFrameData(uint8_t* frameDataPtr) {
    _frameDataPtr = frameDataPtr;
  }

  void XBeeResponse::init() {
    _complete = false;
    _errorCode = NO_ERROR;
    _checksum = 0;
  }

  void XBeeResponse::reset() {
    init();
    _apiId = 0;
    _msbLength = 0;
    _lsbLength = 0;
    _checksum = 0;
    _frameLength = 0;

    _errorCode = NO_ERROR;

    for (int i = 0; i < MAX_FRAME_DATA_SIZE; i++) {
      getFrameData()[i] = 0;
    }
  }

  XBeeRequest::XBeeRequest(uint8_t apiId, uint8_t frameId) {
    _apiId = apiId;
    _frameId = frameId;
  }

  void XBeeRequest::setFrameId(uint8_t frameId) {
    _frameId = frameId;
  }

  uint8_t XBeeRequest::getFrameId() {
    return _frameId;
  }

  uint8_t XBeeRequest::getApiId() {
    return _apiId;
  }

  void XBeeRequest::setApiId(uint8_t apiId) {
    _apiId = apiId;
  }

  PayloadRequest::PayloadRequest(uint8_t apiId, uint8_t frameId, uint8_t *payload, uint8_t payloadLength) : XBeeRequest(apiId, frameId) {
    _payloadPtr = payload;
    _payloadLength = payloadLength;
  }

  uint8_t* PayloadRequest::getPayload() {
    return _payloadPtr;
  }

  void PayloadRequest::setPayload(uint8_t* payload) {
    _payloadPtr = payload;
  }

  uint8_t PayloadRequest::getPayloadLength() {
    return _payloadLength;
  }

  void PayloadRequest::setPayloadLength(uint8_t payloadLength) {
    _payloadLength = payloadLength;
  }


  XBeeAddress::XBeeAddress() {

  }

  XBeeAddress64::XBeeAddress64() : XBeeAddress() {

  }

  XBeeAddress64::XBeeAddress64(uint32_t msb, uint32_t lsb) : XBeeAddress() {
    _msb = msb;
    _lsb = lsb;
  }

  uint32_t XBeeAddress64::getMsb() {
    return _msb;
  }

  void XBeeAddress64::setMsb(uint32_t msb) {
    _msb = msb;
  }

  uint32_t XBeeAddress64::getLsb() {
    return _lsb;
  }


  void XBeeAddress64::setLsb(uint32_t lsb) {
    _lsb = lsb;
  }

  ZBTxRequest::ZBTxRequest() : PayloadRequest(ZB_TX_REQUEST, DEFAULT_FRAME_ID, NULL, 0) {

  }

  ZBTxRequest::ZBTxRequest(XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *data, uint8_t dataLength, uint8_t frameId): PayloadRequest(ZB_TX_REQUEST, frameId, data, dataLength) {
    _addr64 = addr64;
    _addr16 = addr16;
    _broadcastRadius = broadcastRadius;
    _option = option;
  }

  ZBTxRequest::ZBTxRequest(uint8_t *data, uint8_t dataLength): PayloadRequest(ZB_TX_REQUEST, NO_RESPONSE_FRAME_ID, data, dataLength) {
    _addr64 = RemoteAtCommandRequest::broadcastAddress64;
    _addr16 = ZB_BROADCAST_ADDRESS;
    _broadcastRadius = ZB_BROADCAST_RADIUS_MAX_HOPS;
    _option = DISABLE_ACK_OPTION;
  }

  ZBTxRequest::ZBTxRequest(XBeeAddress64 &addr64, uint16_t addr16 , uint8_t *data, uint8_t dataLength): PayloadRequest(ZB_TX_REQUEST, NO_RESPONSE_FRAME_ID, data, dataLength) {
    _addr64 = RemoteAtCommandRequest::broadcastAddress64;
    _addr16 = addr16;
    _broadcastRadius = ZB_BROADCAST_RADIUS_MAX_HOPS;
    _option = DISABLE_ACK_OPTION;
  }


  uint8_t ZBTxRequest::getFrameData(uint8_t pos) {
    if (pos == 0) {
      return (_addr64.getMsb() >> 24) & 0xff;
    } else if (pos == 1) {
      return (_addr64.getMsb() >> 16) & 0xff;
    } else if (pos == 2) {
      return (_addr64.getMsb() >> 8) & 0xff;
    } else if (pos == 3) {
      return _addr64.getMsb() & 0xff;
    } else if (pos == 4) {
      return (_addr64.getLsb() >> 24) & 0xff;
    } else if (pos == 5) {
      return  (_addr64.getLsb() >> 16) & 0xff;
    } else if (pos == 6) {
      return (_addr64.getLsb() >> 8) & 0xff;
    } else if (pos == 7) {
      return _addr64.getLsb() & 0xff;
    } else if (pos == 8) {
      return (_addr16 >> 8) & 0xff;
    } else if (pos == 9) {
      return _addr16 & 0xff;
    } else if (pos == 10) {
      return _broadcastRadius;
    } else if (pos == 11) {
      return _option;
    } else {
      return getPayload()[pos - ZB_TX_API_LENGTH];
    }
  }

  uint8_t ZBTxRequest::getFrameDataLength() {
    return ZB_TX_API_LENGTH + getPayloadLength();
  }

  XBeeAddress64& ZBTxRequest::getAddress64() {
    return _addr64;
  }

  uint16_t ZBTxRequest::getAddress16() {
    return _addr16;
  }

  uint8_t ZBTxRequest::getBroadcastRadius() {
    return _broadcastRadius;
  }

  uint8_t ZBTxRequest::getOption() {
    return _option;
  }

  void ZBTxRequest::setAddress64(XBeeAddress64& addr64) {
    _addr64 = addr64;
  }

  void ZBTxRequest::setAddress16(uint16_t addr16) {
    _addr16 = addr16;
  }

  void ZBTxRequest::setBroadcastRadius(uint8_t broadcastRadius) {
    _broadcastRadius = broadcastRadius;
  }

  void ZBTxRequest::setOption(uint8_t option) {
    _option = option;
  }

  CreateSourceRoute::CreateSourceRoute() : PayloadRequest(ZB_TX_REQUEST, 0, NULL, 0) {

  }

  CreateSourceRoute::CreateSourceRoute(XBeeAddress64 &addr64, uint16_t addr16 , uint8_t *data, uint8_t dataLength): PayloadRequest(CREATE_SOURCE_ROUTE, 0, data, dataLength) {
    _addr64 = addr64;
    _addr16 = addr16;
    _option = 0;
  }


  uint8_t CreateSourceRoute::getFrameData(uint8_t pos) {
    if (pos == 0) {
      return (_addr64.getMsb() >> 24) & 0xff;
    } else if (pos == 1) {
      return (_addr64.getMsb() >> 16) & 0xff;
    } else if (pos == 2) {
      return (_addr64.getMsb() >> 8) & 0xff;
    } else if (pos == 3) {
      return _addr64.getMsb() & 0xff;
    } else if (pos == 4) {
      return (_addr64.getLsb() >> 24) & 0xff;
    } else if (pos == 5) {
      return  (_addr64.getLsb() >> 16) & 0xff;
    } else if (pos == 6) {
      return (_addr64.getLsb() >> 8) & 0xff;
    } else if (pos == 7) {
      return _addr64.getLsb() & 0xff;
    } else if (pos == 8) {
      return (_addr16 >> 8) & 0xff;
    } else if (pos == 9) {
      return _addr16 & 0xff;
    } else if (pos == 10) {
      return _option;
    } else {
      // ROS_INFO("pos - length is %d", pos - CREATE_SOURCE_ROUTE_API_LENGTH);
      // ROS_INFO("data is %d", getPayload()[pos - CREATE_SOURCE_ROUTE_API_LENGTH]);
      return getPayload()[pos - CREATE_SOURCE_ROUTE_API_LENGTH];
    }
  }

  uint8_t CreateSourceRoute::getFrameDataLength() {
    // ROS_INFO(" frame data length is %d", CREATE_SOURCE_ROUTE_API_LENGTH + getPayloadLength());
    return  CREATE_SOURCE_ROUTE_API_LENGTH + getPayloadLength();
  }

  XBeeAddress64& CreateSourceRoute::getAddress64() {
    return _addr64;
  }

  uint16_t CreateSourceRoute::getAddress16() {
    return _addr16;
  }

  uint8_t CreateSourceRoute::getOption() {
    return _option;
  }

  void CreateSourceRoute::setAddress64(XBeeAddress64& addr64) {
    _addr64 = addr64;
  }

  void CreateSourceRoute::setAddress16(uint16_t addr16) {
    _addr16 = addr16;
  }

  void CreateSourceRoute::setOption(uint8_t option) {
    _option = option;
  }

  AtCommandRequest::AtCommandRequest() : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
    _command = NULL;
    clearCommandValue();
  }

  AtCommandRequest::AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
    _command = command;
    _commandValue = commandValue;
    _commandValueLength = commandValueLength;
  }

  AtCommandRequest::AtCommandRequest(uint8_t *command) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
    _command = command;
    clearCommandValue();
  }

  uint8_t* AtCommandRequest::getCommand() {
    return _command;
  }

  uint8_t* AtCommandRequest::getCommandValue() {
    return _commandValue;
  }

  uint8_t AtCommandRequest::getCommandValueLength() {
    return _commandValueLength;
  }

  void AtCommandRequest::setCommand(uint8_t* command) {
    _command = command;
  }

  void AtCommandRequest::setCommandValue(uint8_t* value) {
    _commandValue = value;
  }

  void AtCommandRequest::setCommandValueLength(uint8_t length) {
    _commandValueLength = length;
  }

  uint8_t AtCommandRequest::getFrameData(uint8_t pos) {

    if (pos == 0) {
      return _command[0];
    } else if (pos == 1) {
      return _command[1];
    } else {
      return _commandValue[pos - AT_COMMAND_API_LENGTH];
    }
  }

  void AtCommandRequest::clearCommandValue() {
    _commandValue = NULL;
    _commandValueLength = 0;
  }

  //void AtCommandRequest::reset() {
  //	 XBeeRequest::reset();
  //}

  uint8_t AtCommandRequest::getFrameDataLength() {
    // command is 2 byte + length of value
    return AT_COMMAND_API_LENGTH + _commandValueLength;
  }

  XBeeAddress64 RemoteAtCommandRequest::broadcastAddress64 = XBeeAddress64(0x0, BROADCAST_ADDRESS);

  RemoteAtCommandRequest::RemoteAtCommandRequest() : AtCommandRequest(NULL, NULL, 0) {
    _remoteAddress16 = 0;
    _applyChanges = false;
    setApiId(REMOTE_AT_REQUEST);
  }

  RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength) {
    _remoteAddress64 = broadcastAddress64;
    _remoteAddress16 = remoteAddress16;
    _applyChanges = true;
    setApiId(REMOTE_AT_REQUEST);
  }

  RemoteAtCommandRequest::RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint16_t remoteAddress16, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength) {
    _remoteAddress64 = remoteAddress64;
    _remoteAddress16 = remoteAddress16;
    _applyChanges = true;
    setApiId(REMOTE_AT_REQUEST);
  }


  RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command) : AtCommandRequest(command, NULL, 0) {
    _remoteAddress64 = broadcastAddress64;
    _remoteAddress16 = remoteAddress16;
    _applyChanges = false;
    setApiId(REMOTE_AT_REQUEST);
  }

  RemoteAtCommandRequest::RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength) {
    _remoteAddress64 = remoteAddress64;
    _remoteAddress16 = ZB_BROADCAST_ADDRESS;
    _applyChanges = true;
    setApiId(REMOTE_AT_REQUEST);
  }

  RemoteAtCommandRequest::RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command) : AtCommandRequest(command, NULL, 0) {
    _remoteAddress64 = remoteAddress64;
    _remoteAddress16 = ZB_BROADCAST_ADDRESS;
    _applyChanges = false;
    setApiId(REMOTE_AT_REQUEST);
  }

  uint16_t RemoteAtCommandRequest::getRemoteAddress16() {
    return _remoteAddress16;
  }

  void RemoteAtCommandRequest::setRemoteAddress16(uint16_t remoteAddress16) {
    _remoteAddress16 = remoteAddress16;
  }

  XBeeAddress64& RemoteAtCommandRequest::getRemoteAddress64() {
    return _remoteAddress64;
  }

  void RemoteAtCommandRequest::setRemoteAddress64(XBeeAddress64 &remoteAddress64) {
    _remoteAddress64 = remoteAddress64;
  }

  bool RemoteAtCommandRequest::getApplyChanges() {
    return _applyChanges;
  }

  void RemoteAtCommandRequest::setApplyChanges(bool applyChanges) {
    _applyChanges = applyChanges;
  }

  uint8_t RemoteAtCommandRequest::getFrameData(uint8_t pos) {
    if (pos == 0) {
      return (_remoteAddress64.getMsb() >> 24) & 0xff;
    } else if (pos == 1) {
      return (_remoteAddress64.getMsb() >> 16) & 0xff;
    } else if (pos == 2) {
      return (_remoteAddress64.getMsb() >> 8) & 0xff;
    } else if (pos == 3) {
      return _remoteAddress64.getMsb() & 0xff;
    } else if (pos == 4) {
      return (_remoteAddress64.getLsb() >> 24) & 0xff;
    } else if (pos == 5) {
      return (_remoteAddress64.getLsb() >> 16) & 0xff;
    } else if (pos == 6) {
      return(_remoteAddress64.getLsb() >> 8) & 0xff;
    } else if (pos == 7) {
      return _remoteAddress64.getLsb() & 0xff;
    } else if (pos == 8) {
      return (_remoteAddress16 >> 8) & 0xff;
    } else if (pos == 9) {
      return _remoteAddress16 & 0xff;
    } else if (pos == 10) {
      return _applyChanges ? 2: 0;
    } else if (pos == 11) {
      return getCommand()[0];
    } else if (pos == 12) {
      return getCommand()[1];
    } else {
      return getCommandValue()[pos - REMOTE_AT_COMMAND_API_LENGTH];
    }
  }

  uint8_t RemoteAtCommandRequest::getFrameDataLength() {
    return REMOTE_AT_COMMAND_API_LENGTH + getCommandValueLength();
  }

XBee::XBee (std::string port, uint32_t baudrate): port_(port), baudrate_(baudrate)
  {
    serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
    if(!serial_->isOpen())
      {
        ROS_FATAL("can't open serial port of %s", port_.c_str());
      }
    else
      {
        ROS_INFO("open the serial port");
      }

    pos_ = 0;
    escape_ = false;
    checksum_total_ = 0;
    next_frame_id_ = 0;

    response_.init();
    response_.setFrameData(response_frame_data_);
  }

  XBee::~XBee ()
  {
    delete serial_;
  }

  void XBee::resetResponse() {
    pos_ = 0;
    escape_ = false;
    response_.reset();
  }

  XBeeResponse& XBee::getResponse() {
    return response_;
  }

  void XBee::getResponse(XBeeResponse &response) {

    response.setMsbLength(response_.getMsbLength());
    response.setLsbLength(response_.getLsbLength());
    response.setApiId(response_.getApiId());
    response.setFrameLength(response_.getFrameDataLength());

    response.setFrameData(response_.getFrameData());
  }

  uint8_t XBee::getNextFrameId() {

    next_frame_id_++;

    if (next_frame_id_ == 0) {
      // can't send 0 because that disables status response
      next_frame_id_ = 1;
    }

    return next_frame_id_;
  }

  void XBee::send(XBeeRequest &request) {

    sendByte(START_BYTE, false);

    // send length
    uint8_t msbLen = ((request.getFrameDataLength() + 2) >> 8) & 0xff;
    uint8_t lsbLen = (request.getFrameDataLength() + 2) & 0xff;

    sendByte(msbLen, true);
    sendByte(lsbLen, true);

    // api id
    sendByte(request.getApiId(), true);
    sendByte(request.getFrameId(), true);

    uint8_t checksum = 0;

    // compute checksum, start at api id
    checksum+= request.getApiId();
    checksum+= request.getFrameId();

    for (int i = 0; i < request.getFrameDataLength(); i++) {

      sendByte(request.getFrameData(i), true);
      checksum+= request.getFrameData(i);
    }

    // perform 2s complement
    checksum = 0xff - checksum;

    // send checksum
    sendByte(checksum, true);

    //serial_->flushOutput (); // do we need to fresh the buffer?
  }

  void XBee::sendByte(uint8_t b, bool escape) {

    if (escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)) {

      uint8_t esc = ESCAPE;
      output(esc , 1);
      uint8_t esced = b^0x20;
      output(esced , 1);
    } else {
      output(b , 1);
    }
  }

  void XBee::output (uint8_t &output, int len)
  {
    int i;
    ROS_DEBUG ("SerialInterface::output()");
    i = serial_->write(&output, len);
    if (i != len)
      {
        ROS_ERROR ("Error wrote %d out of %d element(s): %s", i, len, strerror (errno));
      }
  }

  void XBee::readPacketUntilAvailable()
  {
    while (!(getResponse().isAvailable() || getResponse().isError()))
      {
        // read some more
        readPacket();
      }
  }

  bool XBee::readPacket(double timeout)
  {
    if (timeout < 0) return false;

    //unsigned long start = millis();
    double begin_secs =ros::Time::now().toSec();

    while (ros::Time::now().toSec() - begin_secs < timeout)
      {
        readPacket();

        if (getResponse().isAvailable()) return true;
        else if (getResponse().isError()) return false;
      }

    // timed out
    return false;
  }

  void XBee::readPacket()
  {
    // reset previous response
    if (response_.isAvailable() || response_.isError()) {
      // discard previous packet and start over
      resetResponse();
    }

    while(serial_->available() > 0)
      {
        uint8_t b = 0;
        if(serial_->read(&b, 1) == 1)
          {
            if (pos_ > 0 && b == START_BYTE && ATAP == 2)
              {
                ROS_WARN("XBEE Serial RX: new packet start before previous packeted completed -- discard previous packet and start over");

                response_.setErrorCode(UNEXPECTED_START_BYTE);
                return;
              }

            if (pos_ > 0 && b == ESCAPE)
              {
                if (serial_->available()  > 0)
                  {
                    if(serial_->read(&b, 1) == 1) b = 0x20 ^ b;
                    else
                      {
                        ROS_ERROR("read 1 bit error");
                        break;
                      }
                  }
                else
                  {// escape byte.  next byte will be
                    escape_ = true;
                    continue;
                  }
              }

            if (escape_ == true)
              {
                b = 0x20 ^ b;
                escape_ = false;
              }

            /* checksum includes all bytes starting with api id */
            if (pos_ >= API_ID_INDEX)  checksum_total_ += b;

            switch(pos_)
              {
              case 0:
                if (b == START_BYTE) pos_++;
                break;
              case 1:
                // length msb
                response_.setMsbLength(b);
                pos_++;
                break;
              case 2:
                // length lsb
                response_.setLsbLength(b);
                pos_++;
                break;
              case 3:
                response_.setApiId(b);
                pos_++;
                break;
              default:
                // starts at fifth byte
                if (pos_ > MAX_FRAME_DATA_SIZE) {
                  ROS_WARN("XBEE Serial RX: exceed max size.  should never occur");
                  response_.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
                  return;
                }

                // check if we're at the end of the packet
                // packet length does not include start, length, or checksum bytes, so add 3
                if (pos_ == (response_.getPacketLength() + 3))
                  {
                    // verify checksum

                    if ((checksum_total_ & 0xff) == 0xff) {
                      response_.setChecksum(b);
                      response_.setAvailable(true);

                      response_.setErrorCode(NO_ERROR);
                      ROS_INFO("XBEE Serial RX DEBUG: receive the packet");
                    } else {
                      // checksum failed
                      response_.setErrorCode(CHECKSUM_FAILURE);
                    }
                    // minus 4 because we start after start,msb,lsb,api and up to but not including checksum
                    // e.g. if frame was one byte, pos_=4 would be the byte, pos=5 is the checksum, where end stop reading
                    response_.setFrameLength(pos_ - 4);

                    // reset state vars
                    pos_ = 0;

                    checksum_total_ = 0;

                    return;
                  }
                else
                  {
                    //add to packet array, starting with the fourth byte of the apiFrame
                    response_.getFrameData()[pos_ - 4] = b;
                    pos_++;
                  }
              }
          }
        else
          {
            ROS_ERROR("read 1 bit error");
            break;
          }
      }
  }
}

