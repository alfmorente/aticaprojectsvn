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
// File Name: usvInfo3Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a USVInfo3Message

#ifndef USV_INFO_3_MESSAGE_H
#define USV_INFO_3_MESSAGE_H

#include "jaus.h"

#ifndef  JAUS_3_PV 
#define  JAUS_3_PV_ACTIVE_RUDDER_ANGLE_BIT                        0
#define  JAUS_3_PV_ACTIVE_RPM_M1_BIT                              1
#define  JAUS_3_PV_ACTIVE_RPM_M2_BIT                              2
#define  JAUS_3_PV_FUEL_LEVEL_BIT                                 3
#define  JAUS_3_PV_PRESSURE_M1_BIT                                4
#define  JAUS_3_PV_PRESSURE_M2_BIT                                5
#define  JAUS_3_PV_TEMPERATURE_M1_BIT                             6
#define  JAUS_3_PV_TEMPERATURE_M2_BIT                             7
#define  JAUS_3_PV_VOLTAGE_M1_BIT                                 8
#define  JAUS_3_PV_VOLTAGE_M2_BIT                                 9
#define  JAUS_3_PV_ALARMS_BIT                                    10
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

	// MESSAGE DATA MEMBERS GO HERE
        
  // PRESENCE VECTOR
  JausShort presenceVector; 
  
  JausDouble active_rudder_angle;     // Scaled Short (-90,90), Res:0.003
  JausDouble active_rpm_m1;             // Scaled Short (-5000,5000), Res: 0.15
  JausDouble active_rpm_m2;             // Scaled Short (-5000,5000), Res: 0.15
  JausDouble fuel_level;        // Scaled Byte (0,100), Res: 0.3945
  JausDouble pressure_m1;               // Scaled Byte (0,100), Res: 0.4
  JausDouble pressure_m2;               // Scaled Byte (0,100), Res: 0.4
  JausDouble temperature_m1;           // Scaled Short (0,400), Res: 0.006
  JausDouble temperature_m2;           // Scaled Short (0,400), Res: 0.006
  JausDouble voltage_m1;        // Scaled Byte (0,24), Res: 0.1
  JausDouble voltage_m2;        // Scaled Byte (0,24), Res: 0.1
  JausDouble alarms;                 // Scaled Short POR DEFINIR!!

}USVInfo3MessageStruct;

typedef USVInfo3MessageStruct* USVInfo3Message;

JAUS_EXPORT USVInfo3Message usvInfo3MessageCreate(void);
JAUS_EXPORT void usvInfo3MessageDestroy(USVInfo3Message);

JAUS_EXPORT JausBoolean usvInfo3MessageFromBuffer(USVInfo3Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean usvInfo3MessageToBuffer(USVInfo3Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT USVInfo3Message usvInfo3MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage usvInfo3MessageToJausMessage(USVInfo3Message message);

JAUS_EXPORT unsigned int usvInfo3MessageSize(USVInfo3Message message);

JAUS_EXPORT char* usvInfo3MessageToString(USVInfo3Message message);
#endif // USV_INFO_3_MESSAGE_H
