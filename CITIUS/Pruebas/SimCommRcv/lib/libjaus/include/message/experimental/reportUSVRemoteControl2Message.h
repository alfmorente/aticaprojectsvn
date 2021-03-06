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
// File Name: reportUSVRemoteControl2Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a ReportUSVRemoteControl2Message

#ifndef REPORT_USV_REMOTE_CONTROL_2_MESSAGE_H
#define REPORT_USV_REMOTE_CONTROL_2_MESSAGE_H

#include "jaus.h"

#ifndef   JAUS_2_PV 
#define  JAUS_2_PV_APPLIED_DIRECTION_BIT                	0
#define  JAUS_2_PV_REQUESTED_RPM_BIT                            1
#define  JAUS_2_PV_REQUESTED_RUDDER_ANGLE_BIT                   2
#define  JAUS_2_PV_APPLIED_RPM_M1_BIT                           3
#define  JAUS_2_PV_APPLIED_RPM_M2_BIT                           4
#define  JAUS_2_PV_APPLIED_RUDDER_ANGLE_BIT    			5
#define  JAUS_2_PV_VELOCITY_LIMITATIONS_BIT                         6
#define  JAUS_2_PV_DIRECTION_LIMITATIONS_BIT                        7
#define  JAUS_2_PV_MODE_SWITCHING_STATUS_BIT                        8
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
  JausShort presenceVector; 

	// MESSAGE DATA MEMBERS GO HERE
  JausDouble applied_direction;                                 // Scaled Short (-PI,PI), Res: 9e-5
  JausDouble requested_rpm;					// Scaled Short (-5000,5000), Res: 0.15
  JausDouble requested_rudder_angle;				// Scaled Short (-90,90), Res: 0.003
  JausDouble applied_rpm_m1;					// Scaled Short (-5000,5000), Res: 0.15
  JausDouble applied_rpm_m2;					// Scaled Short (-5000,5000), Res: 0.15
  JausDouble applied_rudder_angle;				// Scaled Short (-90,90), Res: 0.003
  JausBoolean velocity_limitations;                                // Scaled Byte (1,n)=>(0,255) Enumerado
  JausBoolean direction_limitations;				// Scaled Byte (1,n)=>(0,255) Enumerado
  JausByte mode_switching_status;				// Scaled Byte (1,7)=>(0,255) Enumerado
  

}ReportUSVRemoteControl2MessageStruct;

typedef ReportUSVRemoteControl2MessageStruct* ReportUSVRemoteControl2Message;

JAUS_EXPORT ReportUSVRemoteControl2Message reportUSVRemoteControl2MessageCreate(void);
JAUS_EXPORT void reportUSVRemoteControl2MessageDestroy(ReportUSVRemoteControl2Message);

JAUS_EXPORT JausBoolean reportUSVRemoteControl2MessageFromBuffer(ReportUSVRemoteControl2Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean reportUSVRemoteControl2MessageToBuffer(ReportUSVRemoteControl2Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT ReportUSVRemoteControl2Message reportUSVRemoteControl2MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage reportUSVRemoteControl2MessageToJausMessage(ReportUSVRemoteControl2Message message);

JAUS_EXPORT unsigned int reportUSVRemoteControl2MessageSize(ReportUSVRemoteControl2Message message);

JAUS_EXPORT char* reportUSVRemoteControl2MessageToString(ReportUSVRemoteControl2Message message);
#endif // REPORT_USV_REMOTE_CONTROL_2_MESSAGE_H
