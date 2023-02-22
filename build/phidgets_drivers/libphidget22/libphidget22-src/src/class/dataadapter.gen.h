#ifndef _DATAADAPTER_H_
#define _DATAADAPTER_H_

/* Generated by WriteClassHeaderVisitor: Mon Jan 09 2023 15:24:31 GMT-0700 (Mountain Standard Time) */

#ifdef INCLUDE_PRIVATE
typedef struct _PhidgetDataAdapter *PhidgetDataAdapterHandle;

/* Methods */
API_PRETURN_HDR PhidgetDataAdapter_create(PhidgetDataAdapterHandle *ch);
API_PRETURN_HDR PhidgetDataAdapter_delete(PhidgetDataAdapterHandle *ch);
API_PRETURN_HDR PhidgetDataAdapter_setI2CFormat(PhidgetDataAdapterHandle ch, const char *format);
API_PRETURN_HDR PhidgetDataAdapter_getLastData(PhidgetDataAdapterHandle ch, uint8_t *data,
  size_t *dataLen, PhidgetDataAdapter_PacketErrorCode *lastDataError);
API_PRETURN_HDR PhidgetDataAdapter_setEndOfLine(PhidgetDataAdapterHandle ch, const char *endOfLine);
API_PRETURN_HDR PhidgetDataAdapter_readLine(PhidgetDataAdapterHandle ch, char *data, size_t *dataLen);
API_PRETURN_HDR PhidgetDataAdapter_sendPacket(PhidgetDataAdapterHandle ch, const uint8_t *data,
  size_t dataLen);
API_VRETURN_HDR PhidgetDataAdapter_sendPacket_async(PhidgetDataAdapterHandle ch, const uint8_t *data,
  size_t dataLen, Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetDataAdapter_sendPacketWaitResponse(PhidgetDataAdapterHandle ch,
  const uint8_t *sendData, size_t sendDataLen, uint8_t *recvData, size_t *recvDataLen, PhidgetDataAdapter_PacketErrorCode *error);
API_PRETURN_HDR PhidgetDataAdapter_write(PhidgetDataAdapterHandle ch, const char *sendData);
API_VRETURN_HDR PhidgetDataAdapter_write_async(PhidgetDataAdapterHandle ch, const char *sendData,
  Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetDataAdapter_writeLine(PhidgetDataAdapterHandle ch, const char *sendData);
API_VRETURN_HDR PhidgetDataAdapter_writeLine_async(PhidgetDataAdapterHandle ch, const char *sendData,
  Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetDataAdapter_writeLineWaitResponse(PhidgetDataAdapterHandle ch,
  const char *sendData, char *recvData, size_t *recvDataLen, PhidgetDataAdapter_PacketErrorCode *error);

/* Properties */
API_PRETURN_HDR PhidgetDataAdapter_setBaudRate(PhidgetDataAdapterHandle ch, uint32_t baudRate);
API_PRETURN_HDR PhidgetDataAdapter_getBaudRate(PhidgetDataAdapterHandle ch, uint32_t *baudRate);
API_PRETURN_HDR PhidgetDataAdapter_getMinBaudRate(PhidgetDataAdapterHandle ch, uint32_t *minBaudRate);
API_PRETURN_HDR PhidgetDataAdapter_getMaxBaudRate(PhidgetDataAdapterHandle ch, uint32_t *maxBaudRate);
API_PRETURN_HDR PhidgetDataAdapter_setDataBits(PhidgetDataAdapterHandle ch, uint32_t dataBits);
API_PRETURN_HDR PhidgetDataAdapter_getDataBits(PhidgetDataAdapterHandle ch, uint32_t *dataBits);
API_PRETURN_HDR PhidgetDataAdapter_getMinDataBits(PhidgetDataAdapterHandle ch, uint32_t *minDataBits);
API_PRETURN_HDR PhidgetDataAdapter_getMaxDataBits(PhidgetDataAdapterHandle ch, uint32_t *maxDataBits);
API_PRETURN_HDR PhidgetDataAdapter_setDeviceAddress(PhidgetDataAdapterHandle ch,
  uint32_t deviceAddress);
API_PRETURN_HDR PhidgetDataAdapter_getDeviceAddress(PhidgetDataAdapterHandle ch,
  uint32_t *deviceAddress);
API_PRETURN_HDR PhidgetDataAdapter_setHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_HandshakeMode handshakeMode);
API_PRETURN_HDR PhidgetDataAdapter_getHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_HandshakeMode *handshakeMode);
API_PRETURN_HDR PhidgetDataAdapter_setEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness endianness);
API_PRETURN_HDR PhidgetDataAdapter_getEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness *endianness);
API_PRETURN_HDR PhidgetDataAdapter_setIOVoltage(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_IOVoltage IOVoltage);
API_PRETURN_HDR PhidgetDataAdapter_getIOVoltage(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_IOVoltage *IOVoltage);
API_PRETURN_HDR PhidgetDataAdapter_getNewDataAvailable(PhidgetDataAdapterHandle ch,
  int *newDataAvailable);
API_PRETURN_HDR PhidgetDataAdapter_setParity(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Parity parity);
API_PRETURN_HDR PhidgetDataAdapter_getParity(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Parity *parity);
API_PRETURN_HDR PhidgetDataAdapter_setProtocol(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Protocol protocol);
API_PRETURN_HDR PhidgetDataAdapter_getProtocol(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Protocol *protocol);
API_PRETURN_HDR PhidgetDataAdapter_getMaxReceivePacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxReceivePacketLength);
API_PRETURN_HDR PhidgetDataAdapter_setResponseTimeout(PhidgetDataAdapterHandle ch,
  uint32_t responseTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getResponseTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *responseTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getMinResponseTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *minResponseTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getMaxResponseTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *maxResponseTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getMaxSendPacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxSendPacketLength);
API_PRETURN_HDR PhidgetDataAdapter_getMaxSendWaitPacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxSendWaitPacketLength);
API_PRETURN_HDR PhidgetDataAdapter_setSPIMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_SPIMode SPIMode);
API_PRETURN_HDR PhidgetDataAdapter_getSPIMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_SPIMode *SPIMode);
API_PRETURN_HDR PhidgetDataAdapter_setStopBits(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_StopBits stopBits);
API_PRETURN_HDR PhidgetDataAdapter_getStopBits(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_StopBits *stopBits);
API_PRETURN_HDR PhidgetDataAdapter_setTransmitTimeout(PhidgetDataAdapterHandle ch,
  uint32_t transmitTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getTransmitTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *transmitTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getMinTransmitTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *minTransmitTimeout);
API_PRETURN_HDR PhidgetDataAdapter_getMaxTransmitTimeout(PhidgetDataAdapterHandle ch,
  uint32_t *maxTransmitTimeout);

/* Events */
typedef void (CCONV *PhidgetDataAdapter_OnPacketCallback)(PhidgetDataAdapterHandle ch, void *ctx,
  const uint8_t *data, size_t dataLen, PhidgetDataAdapter_PacketErrorCode error);

API_PRETURN_HDR PhidgetDataAdapter_setOnPacketHandler(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_OnPacketCallback fptr, void *ctx);

#endif /* INCLUDE_PRIVATE */
#endif /* _DATAADAPTER_H_ */
