/* Generated: Wed Aug 05 2020 10:53:40 GMT-0600 (Mountain Daylight Time) */

#include "device/dataadapterdevice.h"
static void CCONV PhidgetDataAdapter_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetDataAdapter_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetDataAdapter {
	struct _PhidgetChannel phid;
	uint16_t responseID;
	uint8_t lastData[8192];
	uint32_t lastDataIndex;
	uint8_t eventData[8192];
	uint32_t eventDataLen;
	uint32_t eventDataError;
	uint32_t lastDataLen;
	uint32_t lastDataError;
	char I2CFormat[2049];
	int lastDataRead;
	uint32_t baudRate;
	uint32_t minBaudRate;
	uint32_t maxBaudRate;
	uint32_t dataBits;
	uint32_t minDataBits;
	uint32_t maxDataBits;
	uint32_t deviceAddress;
	PhidgetDataAdapter_RTSMode handshakeMode;
	PhidgetDataAdapter_Endianness endianness;
	PhidgetDataAdapter_IOVoltage IOVoltage;
	int newDataAvailable;
	PhidgetDataAdapter_Parity parity;
	PhidgetDataAdapter_Protocol protocol;
	uint32_t maxReceivePacketLength;
	uint32_t maxSendPacketLength;
	uint32_t maxSendWaitPacketLength;
	PhidgetDataAdapter_SPIMode SPIMode;
	PhidgetDataAdapter_StopBits stopBits;
	uint32_t timeout;
	uint32_t minTimeout;
	uint32_t maxTimeout;
	PhidgetDataAdapter_OnPacketCallback Packet;
	void *PacketCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	int version;

	ch = (PhidgetDataAdapterHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 3) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 3 - functionality may be limited.", phid, version);
	}

	if(version >= 3)
		ch->responseID = getBridgePacketUInt16ByName(bp, "responseID");
	if(version >= 3)
		memcpy(&ch->lastData, getBridgePacketUInt8ArrayByName(bp, "lastData"), sizeof (uint8_t) * 8192);
	if(version >= 3)
		ch->lastDataIndex = getBridgePacketUInt32ByName(bp, "lastDataIndex");
	if(version >= 3)
		memcpy(&ch->eventData, getBridgePacketUInt8ArrayByName(bp, "eventData"), sizeof (uint8_t) * 8192);
	if(version >= 3)
		ch->eventDataLen = getBridgePacketUInt32ByName(bp, "eventDataLen");
	if(version >= 3)
		ch->eventDataError = getBridgePacketUInt32ByName(bp, "eventDataError");
	if(version >= 3)
		ch->lastDataLen = getBridgePacketUInt32ByName(bp, "lastDataLen");
	if(version >= 3)
		ch->lastDataError = getBridgePacketUInt32ByName(bp, "lastDataError");
	if(version >= 3)
		memcpy(&ch->I2CFormat, getBridgePacketUInt8ArrayByName(bp, "I2CFormat"), sizeof (char) * 2049);
	if(version >= 3)
		ch->lastDataRead = getBridgePacketInt32ByName(bp, "lastDataRead");
	if(version >= 3)
		ch->baudRate = getBridgePacketUInt32ByName(bp, "baudRate");
	if(version >= 3)
		ch->minBaudRate = getBridgePacketUInt32ByName(bp, "minBaudRate");
	if(version >= 3)
		ch->maxBaudRate = getBridgePacketUInt32ByName(bp, "maxBaudRate");
	if(version >= 3)
		ch->dataBits = getBridgePacketUInt32ByName(bp, "dataBits");
	if(version >= 3)
		ch->minDataBits = getBridgePacketUInt32ByName(bp, "minDataBits");
	if(version >= 3)
		ch->maxDataBits = getBridgePacketUInt32ByName(bp, "maxDataBits");
	if(version >= 3)
		ch->deviceAddress = getBridgePacketUInt32ByName(bp, "deviceAddress");
	if(version >= 3)
		ch->handshakeMode = getBridgePacketInt32ByName(bp, "handshakeMode");
	if(version >= 3)
		ch->endianness = getBridgePacketInt32ByName(bp, "endianness");
	if(version >= 3)
		ch->IOVoltage = getBridgePacketInt32ByName(bp, "IOVoltage");
	if(version >= 3)
		ch->newDataAvailable = getBridgePacketInt32ByName(bp, "newDataAvailable");
	if(version >= 3)
		ch->parity = getBridgePacketInt32ByName(bp, "parity");
	if(version >= 3)
		ch->protocol = getBridgePacketInt32ByName(bp, "protocol");
	if(version >= 3)
		ch->maxReceivePacketLength = getBridgePacketUInt32ByName(bp, "maxReceivePacketLength");
	if(version >= 3)
		ch->maxSendPacketLength = getBridgePacketUInt32ByName(bp, "maxSendPacketLength");
	if(version >= 3)
		ch->maxSendWaitPacketLength = getBridgePacketUInt32ByName(bp, "maxSendWaitPacketLength");
	if(version >= 3)
		ch->SPIMode = getBridgePacketInt32ByName(bp, "SPIMode");
	if(version >= 3)
		ch->stopBits = getBridgePacketInt32ByName(bp, "stopBits");
	if(version >= 3)
		ch->timeout = getBridgePacketUInt32ByName(bp, "timeout");
	if(version >= 3)
		ch->minTimeout = getBridgePacketUInt32ByName(bp, "minTimeout");
	if(version >= 3)
		ch->maxTimeout = getBridgePacketUInt32ByName(bp, "maxTimeout");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDataAdapterHandle ch;

	ch = (PhidgetDataAdapterHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",responseID=uh"
	  ",lastData=%8192R"
	  ",lastDataIndex=%u"
	  ",eventData=%8192R"
	  ",eventDataLen=%u"
	  ",eventDataError=%u"
	  ",lastDataLen=%u"
	  ",lastDataError=%u"
	  ",I2CFormat=%2049R"
	  ",lastDataRead=%d"
	  ",baudRate=%u"
	  ",minBaudRate=%u"
	  ",maxBaudRate=%u"
	  ",dataBits=%u"
	  ",minDataBits=%u"
	  ",maxDataBits=%u"
	  ",deviceAddress=%u"
	  ",handshakeMode=%d"
	  ",endianness=%d"
	  ",IOVoltage=%d"
	  ",newDataAvailable=%d"
	  ",parity=%d"
	  ",protocol=%d"
	  ",maxReceivePacketLength=%u"
	  ",maxSendPacketLength=%u"
	  ",maxSendWaitPacketLength=%u"
	  ",SPIMode=%d"
	  ",stopBits=%d"
	  ",timeout=%u"
	  ",minTimeout=%u"
	  ",maxTimeout=%u"
	  ,3 /* class version */
	  ,ch->responseID
	  ,ch->lastData
	  ,ch->lastDataIndex
	  ,ch->eventData
	  ,ch->eventDataLen
	  ,ch->eventDataError
	  ,ch->lastDataLen
	  ,ch->lastDataError
	  ,ch->I2CFormat
	  ,ch->lastDataRead
	  ,ch->baudRate
	  ,ch->minBaudRate
	  ,ch->maxBaudRate
	  ,ch->dataBits
	  ,ch->minDataBits
	  ,ch->maxDataBits
	  ,ch->deviceAddress
	  ,ch->handshakeMode
	  ,ch->endianness
	  ,ch->IOVoltage
	  ,ch->newDataAvailable
	  ,ch->parity
	  ,ch->protocol
	  ,ch->maxReceivePacketLength
	  ,ch->maxSendPacketLength
	  ,ch->maxSendWaitPacketLength
	  ,ch->SPIMode
	  ,ch->stopBits
	  ,ch->timeout
	  ,ch->minTimeout
	  ,ch->maxTimeout
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDataAdapterHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETI2CFORMAT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DATAOUT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DATAEXCHANGE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETBAUDRATE:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minBaudRate,
		  ch->maxBaudRate);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->baudRate = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "BaudRate");
		break;
	case BP_SETDATABITS:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataBits,
		  ch->maxDataBits);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->dataBits = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "DataBits");
		break;
	case BP_SETADDRESS:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->deviceAddress = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "DeviceAddress");
		break;
	case BP_SETHANDSHAKEMODE:
		if (!supportedRTS_CTS_Mode(phid, (PhidgetDataAdapter_RTSMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified RTS_CTS_Mode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->handshakeMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "HandshakeMode");
		break;
	case BP_SETENDIANNESS:
		if (!supportedEndianness(phid, (PhidgetDataAdapter_Endianness)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified Endianness is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->endianness = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Endianness");
		break;
	case BP_SETIOVOLTAGE:
		if (!supportedIOVoltage(phid, (PhidgetDataAdapter_IOVoltage)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified IOVoltage is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->IOVoltage = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "IOVoltage");
		break;
	case BP_SETPARITY:
		if (!supportedParity(phid, (PhidgetDataAdapter_Parity)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified Parity is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->parity = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Parity");
		break;
	case BP_SETPROTOCOL:
		if (!supportedProtocol(phid, (PhidgetDataAdapter_Protocol)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified Protocol is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->protocol = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Protocol");
		break;
	case BP_SETSPIMODE:
		if (!supportedSPIMode(phid, (PhidgetDataAdapter_SPIMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified SPIMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->SPIMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "SPIMode");
		break;
	case BP_SETSTOPBITS:
		if (!supportedStopBits(phid, (PhidgetDataAdapter_StopBits)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified StopBits is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->stopBits = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "StopBits");
		break;
	case BP_SETTIMEOUT:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minTimeout,
		  ch->maxTimeout);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->timeout = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Timeout");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetDataAdapterHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		ch->newDataAvailable = 0;
		break;
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		ch->maxSendPacketLength = 10000000;
		ch->maxReceivePacketLength = 8192;
		ch->maxSendWaitPacketLength = 1024;
		ch->baudRate = 9600;
		ch->minBaudRate = 800;
		ch->maxBaudRate = 2500000;
		ch->parity = PARITY_MODE_NONE;
		ch->stopBits = STOP_BITS_ONE;
		ch->dataBits = 8;
		ch->minDataBits = 7;
		ch->maxDataBits = 8;
		ch->handshakeMode = RTS_CTS_MODE_READY_TO_RECEIVE;
		ch->protocol = PROTOCOL_RS232;
		ch->newDataAvailable = 0;
		break;
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		ch->maxSendPacketLength = 1024;
		ch->maxReceivePacketLength = 8192;
		ch->maxSendWaitPacketLength = 1024;
		ch->baudRate = 9600;
		ch->minBaudRate = 800;
		ch->maxBaudRate = 2500000;
		ch->parity = PARITY_MODE_NONE;
		ch->stopBits = STOP_BITS_ONE;
		ch->dataBits = 8;
		ch->minDataBits = 7;
		ch->maxDataBits = 8;
		ch->protocol = PUNK_ENUM;
		ch->endianness = ENDIANNESS_LSB_FIRST;
		ch->timeout = 1000;
		ch->minTimeout = 10;
		ch->maxTimeout = 60000;
		ch->newDataAvailable = 0;
		break;
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100:
		ch->maxSendPacketLength = 1024;
		ch->maxReceivePacketLength = 8192;
		ch->maxSendWaitPacketLength = 1024;
		ch->baudRate = 9600;
		ch->minBaudRate = 800;
		ch->maxBaudRate = 2500000;
		ch->parity = PARITY_MODE_NONE;
		ch->stopBits = STOP_BITS_ONE;
		ch->dataBits = 8;
		ch->minDataBits = 4;
		ch->maxDataBits = 8;
		ch->handshakeMode = RTS_CTS_MODE_READY_TO_RECEIVE;
		ch->protocol = PUNK_ENUM;
		ch->SPIMode = SPI_MODE_0;
		ch->deviceAddress = 0;
		ch->endianness = ENDIANNESS_MSB_FIRST;
		ch->timeout = 1000;
		ch->minTimeout = 10;
		ch->maxTimeout = 60000;
		ch->IOVoltage = IO_VOLTAGE_EXTERN;
		ch->newDataAvailable = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->responseID = 0;
	memset(ch->lastData, '\0', 8192);
	ch->lastDataIndex = 0;
	memset(ch->eventData, '\0', 8192);
	ch->eventDataLen = 0;
	ch->eventDataError = 0;
	ch->lastDataLen = 0;
	ch->lastDataError = 0;
	memset(ch->I2CFormat, '\0', 2049);
	ch->lastDataRead = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetDataAdapterHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		break;
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		ret = bridgeSendToDevice(phid, BP_SETBAUDRATE, NULL, NULL, "%u", ch->baudRate);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		ret = bridgeSendToDevice(phid, BP_SETBAUDRATE, NULL, NULL, "%u", ch->baudRate);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100:
		ret = bridgeSendToDevice(phid, BP_SETBAUDRATE, NULL, NULL, "%u", ch->baudRate);
		if (ret != EPHIDGET_OK) {
			break;
		}
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
PhidgetDataAdapter_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDataAdapter));
}

API_PRETURN
PhidgetDataAdapter_create(PhidgetDataAdapterHandle *phidp) {

	CHANNELCREATE_BODY(DataAdapter, PHIDCHCLASS_DATAADAPTER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_delete(PhidgetDataAdapterHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDataAdapter_setI2CFormat(PhidgetDataAdapterHandle ch, const char *format) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETI2CFORMAT, NULL, NULL, "%s", format);
}

API_PRETURN
PhidgetDataAdapter_setBaudRate(PhidgetDataAdapterHandle ch, uint32_t baudRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBAUDRATE, NULL, NULL, "%u", baudRate));
}

API_PRETURN
PhidgetDataAdapter_getBaudRate(PhidgetDataAdapterHandle ch, uint32_t *baudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(baudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*baudRate = ch->baudRate;
	if (ch->baudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinBaudRate(PhidgetDataAdapterHandle ch, uint32_t *minBaudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minBaudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minBaudRate = ch->minBaudRate;
	if (ch->minBaudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxBaudRate(PhidgetDataAdapterHandle ch, uint32_t *maxBaudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxBaudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxBaudRate = ch->maxBaudRate;
	if (ch->maxBaudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setDataBits(PhidgetDataAdapterHandle ch, uint32_t dataBits) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATABITS, NULL, NULL, "%u", dataBits));
}

API_PRETURN
PhidgetDataAdapter_getDataBits(PhidgetDataAdapterHandle ch, uint32_t *dataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*dataBits = ch->dataBits;
	if (ch->dataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinDataBits(PhidgetDataAdapterHandle ch, uint32_t *minDataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minDataBits = ch->minDataBits;
	if (ch->minDataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxDataBits(PhidgetDataAdapterHandle ch, uint32_t *maxDataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxDataBits = ch->maxDataBits;
	if (ch->maxDataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setDeviceAddress(PhidgetDataAdapterHandle ch, uint32_t deviceAddress) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETADDRESS, NULL, NULL, "%u",
	  deviceAddress));
}

API_PRETURN
PhidgetDataAdapter_getDeviceAddress(PhidgetDataAdapterHandle ch, uint32_t *deviceAddress) {

	TESTPTR_PR(ch);
	TESTPTR_PR(deviceAddress);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*deviceAddress = ch->deviceAddress;
	if (ch->deviceAddress == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_RTSMode handshakeMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETHANDSHAKEMODE, NULL, NULL, "%d",
	  handshakeMode));
}

API_PRETURN
PhidgetDataAdapter_getHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_RTSMode *handshakeMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(handshakeMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*handshakeMode = ch->handshakeMode;
	if (ch->handshakeMode == (PhidgetDataAdapter_RTSMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness endianness) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENDIANNESS, NULL, NULL, "%d",
	  endianness));
}

API_PRETURN
PhidgetDataAdapter_getEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness *endianness) {

	TESTPTR_PR(ch);
	TESTPTR_PR(endianness);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*endianness = ch->endianness;
	if (ch->endianness == (PhidgetDataAdapter_Endianness)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setIOVoltage(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_IOVoltage IOVoltage) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETIOVOLTAGE, NULL, NULL, "%d",
	  IOVoltage));
}

API_PRETURN
PhidgetDataAdapter_getIOVoltage(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_IOVoltage *IOVoltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(IOVoltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*IOVoltage = ch->IOVoltage;
	if (ch->IOVoltage == (PhidgetDataAdapter_IOVoltage)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getNewDataAvailable(PhidgetDataAdapterHandle ch, int *newDataAvailable) {

	TESTPTR_PR(ch);
	TESTPTR_PR(newDataAvailable);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*newDataAvailable = ch->newDataAvailable;
	if (ch->newDataAvailable == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setParity(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Parity parity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPARITY, NULL, NULL, "%d", parity));
}

API_PRETURN
PhidgetDataAdapter_getParity(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Parity *parity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(parity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*parity = ch->parity;
	if (ch->parity == (PhidgetDataAdapter_Parity)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setProtocol(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Protocol protocol) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPROTOCOL, NULL, NULL, "%d", protocol));
}

API_PRETURN
PhidgetDataAdapter_getProtocol(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Protocol *protocol) {

	TESTPTR_PR(ch);
	TESTPTR_PR(protocol);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*protocol = ch->protocol;
	if (ch->protocol == (PhidgetDataAdapter_Protocol)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxReceivePacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxReceivePacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxReceivePacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxReceivePacketLength = ch->maxReceivePacketLength;
	if (ch->maxReceivePacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxSendPacketLength(PhidgetDataAdapterHandle ch, uint32_t *maxSendPacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSendPacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxSendPacketLength = ch->maxSendPacketLength;
	if (ch->maxSendPacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxSendWaitPacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxSendWaitPacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSendWaitPacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxSendWaitPacketLength = ch->maxSendWaitPacketLength;
	if (ch->maxSendWaitPacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setSPIMode(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_SPIMode SPIMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPIMODE, NULL, NULL, "%d", SPIMode));
}

API_PRETURN
PhidgetDataAdapter_getSPIMode(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_SPIMode *SPIMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(SPIMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*SPIMode = ch->SPIMode;
	if (ch->SPIMode == (PhidgetDataAdapter_SPIMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setStopBits(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_StopBits stopBits) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTOPBITS, NULL, NULL, "%d", stopBits));
}

API_PRETURN
PhidgetDataAdapter_getStopBits(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_StopBits *stopBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(stopBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*stopBits = ch->stopBits;
	if (ch->stopBits == (PhidgetDataAdapter_StopBits)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setTimeout(PhidgetDataAdapterHandle ch, uint32_t timeout) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTIMEOUT, NULL, NULL, "%u", timeout));
}

API_PRETURN
PhidgetDataAdapter_getTimeout(PhidgetDataAdapterHandle ch, uint32_t *timeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(timeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*timeout = ch->timeout;
	if (ch->timeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinTimeout(PhidgetDataAdapterHandle ch, uint32_t *minTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minTimeout = ch->minTimeout;
	if (ch->minTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxTimeout(PhidgetDataAdapterHandle ch, uint32_t *maxTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
	case PHIDCHUID_ADP_RS232_DATAADAPTER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxTimeout = ch->maxTimeout;
	if (ch->maxTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setOnPacketHandler(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_OnPacketCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);

	ch->Packet = fptr;
	ch->PacketCtx = ctx;

	return (EPHIDGET_OK);
}
