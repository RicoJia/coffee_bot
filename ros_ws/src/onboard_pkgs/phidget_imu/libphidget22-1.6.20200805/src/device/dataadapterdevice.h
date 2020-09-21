/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef __CPHIDGETDATAADAPTERDEVICE
#define __CPHIDGETDATAADAPTERDEVICE

//In
#define DATAADAPTER_PACKET_DATA 0x20
#define DATAADAPTER_PACKET_DATA_ERROR 0x60
#define DATAADAPTER_PACKET_DATA_END 0x30
#define DATAADAPTER_PACKET_TIMEOUT 0x61
#define DATAADAPTER_PACKET_DROPPED 0x70
#define DATAADAPTER_VOLTAGE_ERROR 0x50
#define DATAADAPTER_VOLTAGE_ERROR 0x50
#define DATAADAPTER_PACKET_ACK 0x40
//Out
#define DATAADAPTER_TX_DATA 0x01
#define DATAADAPTER_BAUD_RATE 0x02
#define DATAADAPTER_RTS_CTS_MODE 0x03
#define DATAADAPTER_DTR_DSR_MODE 0x04
#define DATAADAPTER_PARITY_MODE 0x05
#define DATAADAPTER_STOP_BITS 0x06
#define DATAADAPTER_DATA_BITS 0x07
#define DATAADAPTER_MSB_ORDER 0x08
#define DATAADAPTER_RESET 0x09
#define DATAADAPTER_ENABLE 0x0A
#define DATAADAPTER_PROTOCOL 0x0B
#define DATAADAPTER_SPIMODE 0x0C
#define DATAADAPTER_ADDRESS 0x0D
#define DATAADAPTER_ENDIANNESS 0x0E
#define DATAADAPTER_TIMEOUT 0x0F
#define DATAADAPTER_IOVOLTAGE 0x10


//DIGITAL OUTPUT IN
#define STATE_CHANGE 0x0C
#define STATE_INVALID 0x07

//Constants
#define DATAADAPTER_MAX_PACKET_LENGTH 8192
#define USB_OUT_PACKET_LENGTH 64

#define DATAADAPTER_MAXINPUTS 8

#define USB_OUT_PACKET_OVERHEAD 5
#define USB_IN_PACKET_OVERHEAD 5
#define NEW_PACKET_FLAG 0x8000
#define WAIT_RESP_FLAG 0x4000

#define NEW_RX_READY 0xFFFF
#define NO_ACTIVE_PACKET 0xFFFF
#define ANONYMOUS_PACKET_ID 0x0000

typedef struct _PhidgetDataAdapterDevice *PhidgetDataAdapterDeviceHandle;
PhidgetReturnCode PhidgetDataAdapterDevice_create(PhidgetDataAdapterDeviceHandle *phid);

PhidgetReturnCode sendData(PhidgetDataAdapterDeviceHandle phid, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendI2CData(PhidgetDataAdapterDeviceHandle phid, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendDataBuffer(PhidgetDataAdapterDeviceHandle phid, size_t len, const uint8_t *buffer, BridgePacket* bp, int waitResponse);

PhidgetReturnCode parseI2CFormat(PhidgetDataAdapterDeviceHandle phid, const char *string);

struct _PhidgetDataAdapterDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.dataadapter
	
	PhidgetDevice phid;

	/* Public Members */
	double baudRate;
	uint8_t lastData[DATAADAPTER_MAX_PACKET_LENGTH];
	size_t lastDataLength;

	uint8_t inputState[DATAADAPTER_MAXINPUTS];

	/* Private Members */
	uint16_t usbInPacketCount;
	uint32_t packetID;
	uint16_t rxPacketID;

	uint16_t ackID;

	uint8_t completeRXPacket[8192];
	uint16_t completeRXPacketID;
	uint16_t completeRXPacketLength;
	uint16_t rxPacketError;

	uint16_t droppedPacketID;
	//uint16_t activePacket;

	uint8_t storedPacket[8192];	
	uint32_t storedPacketLength;

	PhidgetDataAdapter_Protocol protocol;
	uint8_t i2cFormatList[128];
	char i2cFormatCount;
	uint32_t address;

} typedef PhidgetDataAdapterDeviceInfo;

#endif
