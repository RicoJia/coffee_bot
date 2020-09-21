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

#include "phidgetbase.h"
#include "gpp.h"
#include "device/dataadapterdevice.h"
#include "class/dataadapter.gen.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_initAfterOpen(PhidgetDeviceHandle device) {
	int i;

	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)device;
	//PhidgetReturnCode ret;
	assert(phid);

	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232:
		phid->usbInPacketCount = 0;
		phid->packetID = 0;
		phid->rxPacketID = NEW_RX_READY;
		phid->baudRate = 9600;
		phid->protocol = PUNK_ENUM;
		for (i = 0; i < DATAADAPTER_MAXINPUTS; i++)
			phid->inputState[i] = PUNK_BOOL;
		break;
	case PHIDUID_ADP_RS485_422:
		phid->usbInPacketCount = 0;
		phid->packetID = 0;
		phid->storedPacketLength = 0;
		phid->rxPacketID = NEW_RX_READY;
		phid->baudRate = 9600;
		phid->protocol = PUNK_ENUM;
		break;
	case PHIDUID_ADP_SERIAL:
		phid->usbInPacketCount = 0;
		phid->packetID = 0;
		phid->baudRate = 9600;
		phid->storedPacketLength = 0;
		phid->protocol = PROTOCOL_SPI;
		phid->rxPacketID = NEW_RX_READY;
		phid->protocol = PUNK_ENUM;
		for (i = 0; i < DATAADAPTER_MAXINPUTS; i++)
			phid->inputState[i] = PUNK_BOOL;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)device;
	PhidgetChannelHandle channel;
	uint8_t lastInputState[DATAADAPTER_MAXINPUTS];
	uint16_t receivedPacketCount;
	PhidgetReturnCode ret;
	int endPacket = 0;
	int packetOverrun = 0;
	int error = 0;
	int packetDropped = 0;
	int newResponsePacket = 0;
	uint16_t packetInfo;
	uint16_t packetID;
	uint32_t totalPacketLength;
	int i, j;

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232:
		if (length > 0) {
			switch (buffer[0]) {
			case DATAADAPTER_PACKET_DATA_ERROR:
				error = 1;  //we intend to fall through to the next here
			case DATAADAPTER_PACKET_DATA_END:
				endPacket = 1;
			case DATAADAPTER_PACKET_DATA:
				//Verify the validity of the packet
				receivedPacketCount = unpack16(&buffer[1]);
				packetInfo = unpack16(&buffer[3]);
				packetID = packetInfo & 0x3FFF;
				newResponsePacket = ((packetInfo & NEW_PACKET_FLAG) != 0);

				if (phid->usbInPacketCount != receivedPacketCount || (newResponsePacket && (phid->storedPacketLength != 0))) {
					//Send old data to channel
					if (!newResponsePacket) // we missed the start of this packet, it is in error
						error = 1;
					if ((channel = getChannel(phid, 0)) != NULL) {
						ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, error, (phid->rxPacketID != ANONYMOUS_PACKET_ID), phid->rxPacketID);
						phid->storedPacketLength = 0;

						PhidgetRelease(&channel);
					}
				}

				if (phid->rxPacketID == NEW_RX_READY) {
					phid->rxPacketID = packetID;
					phid->rxPacketError = 0;
				}

				phid->rxPacketError |= error;

				if (packetID != phid->rxPacketID && !packetDropped && !newResponsePacket) {
					MOS_PANIC("TODO: Something got out of sequence");
				}

				//handle the packet
				phid->usbInPacketCount = receivedPacketCount + 1;
				phid->lastDataLength = length - USB_IN_PACKET_OVERHEAD;
				memcpy(phid->lastData, buffer + USB_IN_PACKET_OVERHEAD, phid->lastDataLength);

				if (phid->storedPacketLength + phid->lastDataLength <= DATAADAPTER_MAX_PACKET_LENGTH) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint16_t)phid->lastDataLength;
					if ((endPacket == 0))// && (phid->protocol != PROTOCOL_RS422 && phid->protocol != PROTOCOL_RS485)) //send async packets immediately, store up syncronous ones
						return EPHIDGET_OK;
				} else
					packetOverrun = 1;

				totalPacketLength = phid->storedPacketLength;

				//Send data to channel
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, phid->rxPacketError, packetDropped, packetID);

					if (ret != EPHIDGET_NOSPC) {
						phid->storedPacketLength = 0;
					}

					PhidgetRelease(&channel);
				}

				PhidgetLock(phid);
				phid->completeRXPacketLength = totalPacketLength;
				memcpy(phid->completeRXPacket, phid->storedPacket, phid->completeRXPacketLength);
				phid->completeRXPacketID = phid->rxPacketID;
				phid->rxPacketID = NEW_RX_READY;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);

				if (packetOverrun && ret == EPHIDGET_OK) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint32_t)phid->lastDataLength;
				}

				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_TIMEOUT:
				packetInfo = unpack16(&buffer[1]);
				packetID = packetInfo & 0x3FFF;
				phid->rxPacketID = NEW_RX_READY;
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", 0, phid->storedPacket, 1, 0, packetID);

					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_DROPPED:
				PhidgetLock(phid);
				phid->droppedPacketID = unpack16(&buffer[1]);
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				if ((channel = getChannel(phid, 0)) != NULL) {
					//Could add packet ID here for faster acknowledgement of rejection, but for now the corresponding send will time out.
					//SEND_ERROR_EVENT(channel, EEPHIDGET_PACKETLOST, "One or more of the transmitted packets were lost.");
					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_ACK:
				PhidgetLock(phid);
				phid->ackID = (unpack16(&buffer[1])) & 0x3FFF;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				return (EPHIDGET_OK);
			case STATE_CHANGE:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
	//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = phid->inputState[j];
					phid->inputState[j] = ((buffer[1] & (1 << j)) != 0);
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
							bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
						}
						PhidgetRelease(&channel);
					}
				}

				return (EPHIDGET_OK);
			case STATE_INVALID:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = PUNK_BOOL;
					phid->inputState[j] = PUNK_BOOL;
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_INVALIDSTATE, "Invalid State Detected");
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	case PHIDUID_ADP_RS485_422:
		if (length > 0) {
			switch (buffer[0]) {
			case DATAADAPTER_PACKET_DATA_ERROR:
				error = 1;  //we intend to fall through to the next here
			case DATAADAPTER_PACKET_DATA_END:
				endPacket = 1;
			case DATAADAPTER_PACKET_DATA:
				//Verify the validity of the packet
				receivedPacketCount = unpack16(&buffer[1]);
				packetInfo = unpack16(&buffer[3]);
				packetID = packetInfo & 0x3FFF;
				newResponsePacket = ((packetInfo & NEW_PACKET_FLAG) != 0);

				if (phid->usbInPacketCount != receivedPacketCount || (newResponsePacket && (phid->storedPacketLength != 0))) {
					//Send old data to channel
					if (!newResponsePacket) // we missed the start of this packet, it is in error
						error = 1;
					if ((channel = getChannel(phid, 0)) != NULL) {
						ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, error, (phid->rxPacketID != ANONYMOUS_PACKET_ID), phid->rxPacketID);
						phid->storedPacketLength = 0;

						PhidgetRelease(&channel);
					}
				}

				if (phid->rxPacketID == NEW_RX_READY) {
					phid->rxPacketID = packetID;
					phid->rxPacketError = 0;
				}

				phid->rxPacketError |= error;

				if (packetID != phid->rxPacketID && !packetDropped && !newResponsePacket) {
					MOS_PANIC("TODO: Something got out of sequence");
				}

				//handle the packet
				phid->usbInPacketCount = receivedPacketCount + 1;
				phid->lastDataLength = length - USB_IN_PACKET_OVERHEAD;
				memcpy(phid->lastData, buffer + USB_IN_PACKET_OVERHEAD, phid->lastDataLength);

				if (phid->storedPacketLength + phid->lastDataLength <= DATAADAPTER_MAX_PACKET_LENGTH) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint16_t)phid->lastDataLength;
					if ((endPacket == 0))// && (phid->protocol != PROTOCOL_RS422 && phid->protocol != PROTOCOL_RS485)) //send async packets immediately, store up syncronous ones
						return EPHIDGET_OK;
				} else
					packetOverrun = 1;

				totalPacketLength = phid->storedPacketLength;

				//Send data to channel
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, phid->rxPacketError, packetDropped, packetID);

					if (ret != EPHIDGET_NOSPC) {
						phid->storedPacketLength = 0;
					}

					PhidgetRelease(&channel);
				}

				PhidgetLock(phid);
				phid->completeRXPacketLength = totalPacketLength;
				memcpy(phid->completeRXPacket, phid->storedPacket, phid->completeRXPacketLength);
				phid->completeRXPacketID = phid->rxPacketID;
				phid->rxPacketID = NEW_RX_READY;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);

				if (packetOverrun && ret == EPHIDGET_OK) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint32_t)phid->lastDataLength;
				}

				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_TIMEOUT:
				packetInfo = unpack16(&buffer[1]);
				packetID = packetInfo & 0x3FFF;
				phid->rxPacketID = NEW_RX_READY;
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", 0, phid->storedPacket, 1, 0, packetID);

					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_DROPPED:
				PhidgetLock(phid);
				phid->droppedPacketID = unpack16(&buffer[1]);
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				if ((channel = getChannel(phid, 0)) != NULL) {
					//Could add packet ID here for faster acknowledgement of rejection, but for now the corresponding send will time out.
					//SEND_ERROR_EVENT(channel, EEPHIDGET_PACKETLOST, "One or more of the transmitted packets were lost.");
					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_ACK:
				PhidgetRunLock(phid);
				PhidgetLock(phid);
				phid->ackID = (unpack16(&buffer[1])) & 0x3FFF;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				PhidgetRunUnlock(phid);
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	case PHIDUID_ADP_SERIAL:
		if (length > 0) {
			switch (buffer[0]) {
			case DATAADAPTER_PACKET_DATA_ERROR:
				error = 1;  //we intend to fall through to the next here
			case DATAADAPTER_PACKET_DATA_END:
				endPacket = 1;
			case DATAADAPTER_PACKET_DATA:
				//Verify the validity of the packet
				receivedPacketCount = unpack16(&buffer[1]);
				packetInfo = unpack16(&buffer[3]);
				packetID = packetInfo & 0x3FFF;
				newResponsePacket = ((packetInfo & NEW_PACKET_FLAG) != 0);

				ret = EPHIDGET_OK;

				if (phid->usbInPacketCount != receivedPacketCount || (newResponsePacket && (phid->storedPacketLength != 0))) {
					//Send old data to channel
					if (!newResponsePacket) // we missed the start of this packet, it is in error
						error = 1;
					if ((channel = getChannel(phid, 0)) != NULL) {
						ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, error, (phid->rxPacketID != ANONYMOUS_PACKET_ID), phid->rxPacketID);
						phid->storedPacketLength = 0;

						PhidgetRelease(&channel);
					}
				}

				if (phid->rxPacketID == NEW_RX_READY) {
					phid->rxPacketID = packetID;
					phid->rxPacketError = 0;
				}

				phid->rxPacketError |= error;

				if (packetID != phid->rxPacketID && !packetDropped && !newResponsePacket) {
					MOS_PANIC("TODO: Something got out of sequence");
				}

				//handle the packet
				phid->usbInPacketCount = receivedPacketCount + 1;
				phid->lastDataLength = length - USB_IN_PACKET_OVERHEAD;
				memcpy(phid->lastData, buffer + USB_IN_PACKET_OVERHEAD, phid->lastDataLength);

				if (phid->storedPacketLength + phid->lastDataLength <= DATAADAPTER_MAX_PACKET_LENGTH) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint16_t)phid->lastDataLength;
					if ((endPacket == 0))// && (phid->protocol != PROTOCOL_RS422 && phid->protocol != PROTOCOL_RS485)) //send async packets immediately, store up syncronous ones
						return EPHIDGET_OK;
				} else
					packetOverrun = 1;

				totalPacketLength = phid->storedPacketLength;

				//Send data to channel
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", phid->storedPacketLength, phid->storedPacket, phid->rxPacketError, packetDropped, packetID);

					if (ret != EPHIDGET_NOSPC) {
						phid->storedPacketLength = 0;
					}

					PhidgetRelease(&channel);
				}

				PhidgetLock(phid);
				phid->completeRXPacketLength = totalPacketLength;
				memcpy(phid->completeRXPacket, phid->storedPacket, phid->completeRXPacketLength);
				phid->completeRXPacketID = phid->rxPacketID;
				phid->rxPacketID = NEW_RX_READY;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);

				if (packetOverrun && ret == EPHIDGET_OK) {
					memcpy(&phid->storedPacket[phid->storedPacketLength], phid->lastData, phid->lastDataLength);
					phid->storedPacketLength += (uint32_t)phid->lastDataLength;
				}

				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_TIMEOUT:
				packetInfo = unpack16(&buffer[1]);
				packetID = packetInfo & 0x3FFF;
				phid->rxPacketID = NEW_RX_READY;
				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = bridgeSendToChannel(channel, BP_DATAIN, "%*R%u%u%uh", 0, phid->storedPacket, 1, 0, packetID);

					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_DROPPED:
				PhidgetLock(phid);
				phid->droppedPacketID = unpack16(&buffer[1]);
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				if ((channel = getChannel(phid, 0)) != NULL) {
					//Could add packet ID here for faster acknowledgement of rejection, but for now the corresponding send will time out.
					//SEND_ERROR_EVENT(channel, EEPHIDGET_PACKETLOST, "One or more of the transmitted packets were lost.");
					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case DATAADAPTER_PACKET_ACK:
				PhidgetLock(phid);
				phid->ackID = (unpack16(&buffer[1])) & 0x3FFF;
				PhidgetBroadcast(phid);
				PhidgetUnlock(phid);
				return (EPHIDGET_OK);
			case DATAADAPTER_VOLTAGE_ERROR:
				if ((channel = getChannel(phid, 0)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected");
					PhidgetRelease(&channel);
				}
				return (EPHIDGET_OK);
			case STATE_CHANGE:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = phid->inputState[j];
					phid->inputState[j] = ((buffer[1] & (1 << j)) != 0);
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
							bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
						}
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			case STATE_INVALID:
				for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
					//				inputState[j] = PUNK_BOOL;
					lastInputState[j] = PUNK_BOOL;
					phid->inputState[j] = PUNK_BOOL;
				}

				for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
					int chIndex = i + phid->devChannelCnts.numDataAdapters;
					if ((channel = getChannel(phid, chIndex)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_INVALIDSTATE, "Invalid State Detected");
						PhidgetRelease(&channel);
					}
				}
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		}
		MOS_PANIC("Unexpected packet type");
	default:
		MOS_PANIC("Unexpected device");
	}

}

static PhidgetReturnCode CCONV
PhidgetDataAdapterDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetDataAdapterDeviceHandle phid = (PhidgetDataAdapterDeviceHandle)ch->parent;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	PhidgetReturnCode ret;
	double dutyCycle;
	int32_t state;
	size_t len;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_DATAADAPTER);


	switch (((PhidgetDeviceHandle)phid)->deviceInfo.UDD->uid) {
	case PHIDUID_ADP_RS232:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
				ret = sendData(phid, bp, 0);
				return ret;
			case BP_DATAEXCHANGE:
				ret = sendData(phid, bp, 1);
				return ret;
			case BP_SETBAUDRATE:
				len = 4;
				pack32(buffer, getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
			case BP_SETPARITY:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
			case BP_SETSTOPBITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
			case BP_SETDATABITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
			case BP_SETHANDSHAKEMODE:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RTS_CTS_MODE, 0, buffer, &len, 100);
			case BP_SETTIMEOUT:
				len = 2;
				pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
			case BP_OPENRESET:
				len = 0;
				phid->protocol = PROTOCOL_RS232;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RESET, 0, buffer, &len, 100);
			case BP_CLOSERESET:
				len = 0;
				phid->protocol = PUNK_ENUM;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RESET, 0, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ENABLE, 0, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALINPUT:
			switch (bp->vpkt) {
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALOUTPUT:
			switch (bp->vpkt) {
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0 && dutyCycle != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)dutyCycle;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_SETSTATE:
				if (phid->protocol != PROTOCOL_RS232)
					return EPHIDGET_NOTCONFIGURED;
				state = getBridgePacketInt32(bp, 0);
				if (state != 0 && state != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)state;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
	case PHIDUID_ADP_RS485_422:
		switch (bp->vpkt) {
		case BP_DATAOUT:
			ret = sendData(phid, bp, 0);
			return ret;
		case BP_DATAEXCHANGE:
			ret = sendData(phid, bp, 1);
			return ret;
		case BP_SETBAUDRATE:
			len = 4;
			pack32(buffer, getBridgePacketUInt32(bp, 0));
			ret =  PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
			if (ret == EPHIDGET_OK)
				phid->baudRate = getBridgePacketUInt32(bp, 0);
			return ret;
		case BP_SETPARITY:
			len = 1;
			if (phid->protocol == PROTOCOL_DMX512)
				return EPHIDGET_NOTCONFIGURED;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
		case BP_SETSTOPBITS:
			len = 1;
			if (phid->protocol == PROTOCOL_MODBUS_RTU || phid->protocol == PROTOCOL_DMX512)
				return EPHIDGET_NOTCONFIGURED;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
		case BP_SETDATABITS:
			len = 1;
			if (phid->protocol == PROTOCOL_MODBUS_RTU || phid->protocol == PROTOCOL_DMX512)
				return EPHIDGET_NOTCONFIGURED;
			buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			len = 0;
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RESET, 0, buffer, &len, 100);
		case BP_ENABLE:
			len = 0;
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ENABLE, 0, buffer, &len, 100);

		case BP_SETPROTOCOL:
			phid->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_PROTOCOL, 0, buffer, &len, 100);
		case BP_SETTIMEOUT:
			len = 2;
			pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
		case BP_SETENDIANNESS:
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ENDIANNESS, 0, buffer, &len, 100);
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDUID_ADP_SERIAL:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
				if(phid->protocol == PROTOCOL_I2C)
					ret = sendI2CData(phid, bp, 1);
				else if(phid->protocol == PROTOCOL_UART)
					ret = sendData(phid, bp, 0);
				else
					ret = sendData(phid, bp, 1);

				return ret;
			case BP_DATAEXCHANGE:
				if (phid->protocol == PROTOCOL_I2C)
					ret = sendI2CData(phid, bp, 1);
				else
					ret = sendData(phid, bp, 1);
				return ret;
			case BP_SETBAUDRATE:
				len = 4;
				pack32(buffer, getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				phid->protocol = PUNK_ENUM;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RESET, 0, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ENABLE, 0, buffer, &len, 100);

			case BP_SETDATABITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
			case BP_SETPROTOCOL:
				phid->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_PROTOCOL, 0, buffer, &len, 100);
			case BP_SETHANDSHAKEMODE:
				len = 1;
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_RTS_CTS_MODE, 0, buffer, &len, 100);
			case BP_SETSPIMODE:
				len = 1;
				if (phid->protocol != PROTOCOL_SPI)
					return EPHIDGET_NOTCONFIGURED;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_SPIMODE, 0, buffer, &len, 100);
			case BP_SETADDRESS:
				if (phid->protocol != PROTOCOL_SPI || phid->protocol != PROTOCOL_I2C)
					return EPHIDGET_NOTCONFIGURED;
				if (phid->protocol != PROTOCOL_I2C) {
					len = 1;
					buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
					return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ADDRESS, 0, buffer, &len, 100);
				}
				else {
					phid->address = getBridgePacketUInt32(bp, 0);
					return EPHIDGET_OK;
				}
			case BP_SETENDIANNESS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_ENDIANNESS, 0, buffer, &len, 100);
			case BP_SETIOVOLTAGE:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_IOVOLTAGE, 0, buffer, &len, 100);
			case BP_SETTIMEOUT:
				len = 2;
				pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
			case BP_SETPARITY:
				len = 1;
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
			case BP_SETSTOPBITS:
				len = 1;
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
			case BP_SETI2CFORMAT:
				ret = parseI2CFormat(phid, getBridgePacketString(bp, 0));
				if (phid->protocol != PROTOCOL_I2C)
					return EPHIDGET_NOTCONFIGURED;
				return ret;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALINPUT:
			switch (bp->vpkt) {
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_DIGITALOUTPUT:
			switch (bp->vpkt) {
			case BP_SETDUTYCYCLE:
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0 && dutyCycle != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)dutyCycle;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_SETSTATE:
				if (phid->protocol != PROTOCOL_UART)
					return EPHIDGET_NOTCONFIGURED;
				state = getBridgePacketInt32(bp, 0);
				if (state != 0 && state != 1)
					return EPHIDGET_INVALIDARG;
				buffer[0] = (uint8_t)state;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, ch->uniqueIndex, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
	default:
		MOS_PANIC("Unexpected device");
	}
}

static void CCONV
PhidgetDataAdapterDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetDataAdapterDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetDataAdapterDevice_create(PhidgetDataAdapterDeviceHandle *phidp) {
	DEVICECREATE_BODY(DataAdapterDevice, PHIDCLASS_DATAADAPTER);
	return (EPHIDGET_OK);
}


PhidgetReturnCode sendData( PhidgetDataAdapterDeviceHandle phid, BridgePacket* bp, int waitResponse) {
	return sendDataBuffer(phid, bp->entry[0].len, (const uint8_t *)getBridgePacketUInt8Array(bp, 0) , bp, waitResponse);
}

PhidgetReturnCode parseI2CFormat(PhidgetDataAdapterDeviceHandle phid, const char *string) {
	int index = 0;
	int count = 0;
	int stopped = 0;
	int mode = 0; //0 transmit, 1 receive
	int totalFormatCount = 0;
	size_t i;

	uint8_t tmpFormatList[128];

	for (i = 0; i < mos_strlen(string); i++) {
		if (stopped)
			return EPHIDGET_INVALIDARG;
		switch (string[i]) {
		case 's':
			if (i == 0)
				continue;
			if (count != 0) {
				tmpFormatList[index] = count;
				if (mode)
					tmpFormatList[index] |= 0x80;

				index++;
				count = 0;
			} else
				return EPHIDGET_INVALIDARG;
			break;
		case 'T':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count == 0) {
				mode = 0;
			}
			if (mode != 0) {
				return EPHIDGET_INVALIDARG;
			}
			count++;
			totalFormatCount++;
			if (count > 127 || totalFormatCount > 512)
				return EPHIDGET_INVALIDARG;
			break;
		case 'R':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count == 0) {
				mode = 1;
			}
			if (mode != 1) {
				return EPHIDGET_INVALIDARG;
			}
			count++;
			totalFormatCount++;
			if (count > 127 || totalFormatCount > 512)
				return EPHIDGET_INVALIDARG;
			break;
		case 'p':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count != 0) {
				tmpFormatList[index] = count;
				if (mode)
					tmpFormatList[index] |= 0x80;



			} else
				return EPHIDGET_INVALIDARG;
			stopped = 1;
			break;
		default:
			return EPHIDGET_INVALIDARG;
		}
	}
	if (!stopped)
		return EPHIDGET_INVALIDARG;

	phid->i2cFormatCount = index + 1;
	memcpy(phid->i2cFormatList, tmpFormatList, phid->i2cFormatCount);

	return EPHIDGET_OK;
}

PhidgetReturnCode sendI2CData(PhidgetDataAdapterDeviceHandle phid, BridgePacket* bp, int waitResposne) {
	uint8_t buffer[1024];
	int transmitCount = 0;
	int i;
	uint8_t dataSize = bp->entry[0].len;

	uint8_t totalCount = dataSize + phid->i2cFormatCount + 2;

	if (totalCount > 1024)
		return EPHIDGET_INVALIDARG;

	for (i = 0; i < phid->i2cFormatCount; i++) {
		if (!(phid->i2cFormatList[i] & 0x80)) //if transmit segment
			transmitCount += phid->i2cFormatList[i] & 0x7F;
	}

	if (transmitCount != dataSize)
		return EPHIDGET_INVALIDARG;

	buffer[0] = phid->address;
	buffer[1] = phid->i2cFormatCount;
	memcpy(&buffer[2], phid->i2cFormatList, phid->i2cFormatCount);
	memcpy(&buffer[phid->i2cFormatCount+2], (const uint8_t *)getBridgePacketUInt8Array(bp, 0), dataSize);

	return sendDataBuffer(phid, totalCount, buffer, bp, waitResposne);
}

PhidgetReturnCode sendDataBuffer(PhidgetDataAdapterDeviceHandle phid, size_t len, const uint8_t *buffer, BridgePacket* bp, int waitResposne) {
	PhidgetReturnCode ret;
	size_t packetLen;
	uint32_t packetCount = 0;
	uint16_t packetID;
	uint16_t packetInfo;
	size_t i = 0;

	uint8_t buf[USB_OUT_PACKET_LENGTH];

	mostime_t duration;
	mostime_t start;

	if (phid->baudRate == 0)
		return EPHIDGET_NOTCONFIGURED;

	start = mos_gettime_usec();

	//PhidgetRunLock(phid);
	//Assign unique packet ID
	PhidgetLock(phid);
	phid->packetID++;
	phid->packetID &= 0x3FFF; //14-bit packet ID
	if (phid->packetID == ANONYMOUS_PACKET_ID)
		phid->packetID = 0x0001;
	packetID = phid->packetID;
	PhidgetUnlock(phid);
	//PhidgetRunUnlock(phid);

	//Bridge packet reply is packet ID
	bp->reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
	memset(bp->reply_bpe, 0, sizeof(BridgePacketEntry));

	bp->reply_bpe->type = BPE_UI8ARRAY;
	bp->reply_bpe->bpe_len = (uint16_t)2;
	bp->reply_bpe->bpe_ptr = mos_malloc(2);
	bp->reply_bpe->bpe_ui8array = bp->reply_bpe->bpe_ptr;
	bp->reply_bpe->bpe_cnt = (uint16_t)1;

	bp->reply_bpe->bpe_ui8array[0] = (phid->packetID & 0xFF00) >> 8;
	int tmp = (phid->packetID & 0xFF00) >> 8;
	tmp = tmp;
	bp->reply_bpe->bpe_ui8array[1] = phid->packetID & 0xFF;


	//Wait until previous packets are dealt with, ensures the device will accept the packet

	ret = EPHIDGET_OK;

	//Transfer the packet

	if (len > 0) {
		if (len <= (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD)) {
			packetLen = len + USB_OUT_PACKET_OVERHEAD;
			packetInfo = NEW_PACKET_FLAG | packetID;
			if (waitResposne)
				packetInfo |= WAIT_RESP_FLAG;
			pack16(&buf[0], packetInfo);
			buf[2] = (uint8_t)(len >> 16); //pack 24
			buf[3] = (uint8_t)(len >> 8);
			buf[4] = (uint8_t)(len & 0xFF);
			memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer, len);
			ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, 5000);
			if (ret != EPHIDGET_OK) {
//				PhidgetRunUnlock(phid);
				return ret;
			}

		} else {
			for (i = 0; i + (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD) < len; i += (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD)) {
				packetLen = USB_OUT_PACKET_LENGTH;
				if (i == 0) {
					packetInfo = NEW_PACKET_FLAG | packetID;
					if (waitResposne)
						packetInfo |= WAIT_RESP_FLAG;
					pack16(&buf[0], packetInfo);
					buf[2] = (uint8_t)(len >> 16); //pack 24
					buf[3] = (uint8_t)(len >> 8);
					buf[4] = (uint8_t)(len & 0xFF);
				} else {
					pack16(&buf[0], packetID);
					buf[2] = (uint8_t)(packetCount >> 16); //pack 24
					buf[3] = (uint8_t)(packetCount >> 8);
					buf[4] = (uint8_t)(packetCount & 0xFF);
				}

				if (phid->droppedPacketID == packetID) {
					return EPHIDGET_INTERRUPTED;
				}
				memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
				ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, 5000);
				if (ret != EPHIDGET_OK) {
//					PhidgetRunUnlock(phid);
					return ret;
				}
				packetCount++;
			}
			if (i != len) {
				packetLen = len - i + USB_OUT_PACKET_OVERHEAD;
				pack16(&buf[0], packetID);
				buf[2] = (uint8_t)(packetCount >> 16); //pack 24
				buf[3] = (uint8_t)(packetCount >> 8);
				buf[4] = (uint8_t)(packetCount & 0xFF);

				if (phid->droppedPacketID == packetID) {
					return EPHIDGET_INTERRUPTED;
				}
				memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
				ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, 5000);
				if (ret != EPHIDGET_OK) {
//					PhidgetRunUnlock(phid);
					return ret;
				}
			}
		}
	}

	//Wait to see if the device accepted the packet, this makes the return code from sendPacket mean something
	start = mos_gettime_usec();

	//(no need to lock here - already locked)
	for (;;) {
		if (phid->ackID == packetID) { // a direct comparison should work here, as there is only ever one active packet at a time
//			PhidgetRunUnlock(phid);
			return (EPHIDGET_OK);
		}

		if (phid->droppedPacketID == packetID) {
			return EPHIDGET_INTERRUPTED;
		}

		if (!(ISATTACHED(phid))) {
//			PhidgetRunUnlock(phid);
			return (EPHIDGET_CLOSED);
		}

		duration = (mos_gettime_usec() - start) / 1000;
		if (duration >= ((640000 / phid->baudRate) + 50)) {
		//	phid->activePacket = NO_ACTIVE_PACKET; // transmission failed, the packet is no longer active
//			PhidgetRunUnlock(phid);
			return (EPHIDGET_TIMEOUT);
		}
		PhidgetLock(phid);
		if (phid->baudRate != 0)
			PhidgetTimedWait(phid, ((640000 / phid->baudRate) + 50) - (uint32_t)duration);
		else
			return EPHIDGET_NOTCONFIGURED;
		PhidgetUnlock(phid);
	}

}
