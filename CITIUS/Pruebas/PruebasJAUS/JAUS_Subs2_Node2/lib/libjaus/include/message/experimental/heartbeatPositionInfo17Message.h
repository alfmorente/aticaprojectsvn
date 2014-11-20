/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *  
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD 
 *  license.  See the LICENSE file for details.
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: heartbeatPositionInfo17Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a HeartbeatPositionInfo17Message

#ifndef HEARTBEAT_POSITION_INFO_17_MESSAGE_H
#define HEARTBEAT_POSITION_INFO_17_MESSAGE_H

#include "jaus.h"

#ifndef   JAUS_17_PV 
#define  JAUS_17_PV_LATITUDE_BIT    		0
#define  JAUS_17_PV_LONGITUDE_BIT    		1
#define  JAUS_17_PV_ALTITUDE_BIT    		2
#define  JAUS_17_PV_HEADING_BIT    		3
#define  JAUS_17_PV_SPEED_BIT                   4
#endif


typedef struct
{
	// Include all parameters from a JausMessage structure:
	// Header Properties
	struct
	{
		// Properties by bit fields
		#ifdef JAUS_BIG_ENDIAN
			JausUnsignedShort reserved:2;
			JausUnsignedShort version:6;
			JausUnsignedShort expFlag:1;
			JausUnsignedShort scFlag:1;
			JausUnsignedShort ackNak:2;
			JausUnsignedShort priority:4; 
		#elif JAUS_LITTLE_ENDIAN
			JausUnsignedShort priority:4; 
			JausUnsignedShort ackNak:2;
			JausUnsignedShort scFlag:1; 
			JausUnsignedShort expFlag:1;
			JausUnsignedShort version:6; 
			JausUnsignedShort reserved:2;
		#else
			#error "Please define system endianess (see jaus.h)"
		#endif
	}properties;

	JausUnsignedShort commandCode; 

	JausAddress destination;

	JausAddress source;

	JausUnsignedInteger dataSize;

	JausUnsignedInteger dataFlag;
	
	JausUnsignedShort sequenceNumber;

 // PRESENCE VECTOR
  JausByte presenceVector; 

  // MESSAGE DATA MEMBERS GO HERE

  JausDouble latitude;			// Scaled Int (-90, 90), Res: 4e-8
  JausDouble longitude;			// Scaled Int (-180, 180), Res: 8e-8
  JausDouble altitude;			// Scaled Int (-10000, 35000), Res: 1e-5
  JausDouble heading;			// Scaled Short (-JAUS_PI, JAUS_PI), Res: 1e-5
  JausDouble speed;      	// Scaled Short (0,10000), Res: 0.1526
  
}HeartbeatPositionInfo17MessageStruct;

typedef HeartbeatPositionInfo17MessageStruct* HeartbeatPositionInfo17Message;

JAUS_EXPORT HeartbeatPositionInfo17Message heartbeatPositionInfo17MessageCreate(void);
JAUS_EXPORT void heartbeatPositionInfo17MessageDestroy(HeartbeatPositionInfo17Message);

JAUS_EXPORT JausBoolean heartbeatPositionInfo17MessageFromBuffer(HeartbeatPositionInfo17Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean heartbeatPositionInfo17MessageToBuffer(HeartbeatPositionInfo17Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT HeartbeatPositionInfo17Message heartbeatPositionInfo17MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage heartbeatPositionInfo17MessageToJausMessage(HeartbeatPositionInfo17Message message);

JAUS_EXPORT unsigned int heartbeatPositionInfo17MessageSize(HeartbeatPositionInfo17Message message);

JAUS_EXPORT char* heartbeatPositionInfo17MessageToString(HeartbeatPositionInfo17Message message);
#endif // HEARTBEAT_POSITION_INFO_17_MESSAGE_H
