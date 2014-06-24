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
// File Name: reportScientificOperations13Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a ReportScientificOperations13Message

#ifndef REPORT_SCIENTIFIC_OPERATIONS_13_MESSAGE_H
#define REPORT_SCIENTIFIC_OPERATIONS_13_MESSAGE_H

#include "jaus.h"

#ifndef   JAUS_13_PV 
#define  JAUS_13_PV_SOUND_SPEED_BIT                		0
#define  JAUS_13_PV_FLOW_VALUE_BIT                              1
#define  JAUS_13_PV_FLOW_DEPTH_BIT                              2
#define  JAUS_13_PV_FLOW_DIRECTION_BIT                          3
#define  JAUS_13_PV_WATER_SOUND_SPEED_BIT               	4
#define  JAUS_13_PV_WATER_SOUND_DEPTH_BIT    			5
#define  JAUS_13_PV_CONTACT_ID_BIT                              6
#define  JAUS_13_PV_CONTACT_DELAY_BIT    			7
#define  JAUS_13_PV_CONTACT_DISTANCE_BIT    			8
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
	
  JausDouble sound_speed;			// Scaled Short (0,3000), Res: 0.05
  JausDouble flow_value;				// Scaled Short (0,250), Res: 1.0
  JausDouble flow_depth;			// Scaled Short (0,10000), Res: 0.15
  JausDouble flow_direction;			// Scaled Short (0,360), 0.005
  JausDouble water_sound_speed;			// Scaled Short (0,10000), Res: 0.15
  JausDouble water_sound_depth;			// Scaled Short (0,10000), Res: 0.15
  JausByte contact_id;				        // ENUM NO DEFINIDO!!
  JausDouble contact_delay;				// Scaled Short (-180,180), Res: 0.005
  JausDouble contact_distance;				// Scaled Short (0,10000), Res: 0.15
  
}ReportScientificOperations13MessageStruct;

typedef ReportScientificOperations13MessageStruct* ReportScientificOperations13Message;

JAUS_EXPORT ReportScientificOperations13Message reportScientificOperations13MessageCreate(void);
JAUS_EXPORT void reportScientificOperations13MessageDestroy(ReportScientificOperations13Message);

JAUS_EXPORT JausBoolean reportScientificOperations13MessageFromBuffer(ReportScientificOperations13Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean reportScientificOperations13MessageToBuffer(ReportScientificOperations13Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT ReportScientificOperations13Message reportScientificOperations13MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage reportScientificOperations13MessageToJausMessage(ReportScientificOperations13Message message);

JAUS_EXPORT unsigned int reportScientificOperations13MessageSize(ReportScientificOperations13Message message);

JAUS_EXPORT char* reportScientificOperations13MessageToString(ReportScientificOperations13Message message);
#endif // REPORT_SCIENTIFIC_OPERATIONS_13_MESSAGE_H
