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
// File Name: setDriveByWireMessage.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a SetDriveByWireMessage

#ifndef SET_DRIVE_BY_WIRE_MESSAGE_H
#define SET_DRIVE_BY_WIRE_MESSAGE_H

#include "jaus.h"
#ifndef JAUS_DRIVE_BY_WIRE_PV
#define JAUS_DRIVE_BY_WIRE_PV
#define JAUS_DRIVE_BY_WIRE_PV_FOCO_BIT		0
#define JAUS_DRIVE_BY_WIRE_PV_MONITOR_BIT	1
#define JAUS_DRIVE_BY_WIRE_PV_MASTIL_BIT	2
#define JAUS_DRIVE_BY_WIRE_PV_VALVULA_BIT	3
#define JAUS_DRIVE_BY_WIRE_PV_CHORRO_BIT	4   
#endif


typedef enum 
{	
	FOCO_PAN_RIGHT=0,
	FOCO_PAN_LEFT=1,
	FOCO_TILT_UP=2,
	FOCO_TILT_DOWN=3,
	FOCO_RIGHT_ON=4,
	FOCO_RIGHT_OFF=5,
	FOCO_LEFT_ON=6,
	FOCO_LEFT_OFF=7,
	FOCO_NO_ACTION=8
}focoOptions;

typedef enum 
{	
	MONITOR_PAN_RIGHT=0,
	MONITOR_PAN_LEFT=1,
	MONITOR_TILT_UP=2,
	MONITOR_TILT_DOWN=3,
	MONITOR_ON=4,
	MONITOR_OFF=5,
	MONITOR_NO_ACTION=6

}monitorOptions;

typedef enum 
{	
	ASPIRAR_ON=0,
	ASPIRAR_OFF=1,
	AUTOLLENADO_ON=2,
	AUTOLLENADO_OFF=3,
	BOMBA_FUERZA_ON=4,
	BOMBA_FUERZA_OFF=5,
	AUMENTAR_PRESION=6,
	DISMINUIR_PRESION=7,
	VALVULA_NO_ACTION=8

}valvulaOptions;

typedef enum 
{	
	MASTIL_UP=0,
	MASTIL_DOWN=1,
	MASTIL_STOP=2,
	MASTIL_NO_ACTION=3

}mastilOptions;

typedef enum 
{	
	CHORRO=0,
	NUBE=1
}chorroOptions;

typedef struct
{
	// Include all parameters from a JausMessage structure:
	// Header Properties#define JAUS_WRENCH_PV_PROPULSIVE_LINEAR_Z_BIT		2
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

	JausByte idVehicle;

	JausByte presenceVector;
	focoOptions foco;
	monitorOptions monitor;
	valvulaOptions valvula;
	mastilOptions mastil;
	chorroOptions chorro;


	
}SetDriveByWireMessageStruct;

typedef SetDriveByWireMessageStruct* SetDriveByWireMessage;

JAUS_EXPORT SetDriveByWireMessage setDriveByWireMessageCreate(void);
JAUS_EXPORT void setDriveByWireMessageDestroy(SetDriveByWireMessage);

JAUS_EXPORT JausBoolean setDriveByWireMessageFromBuffer(SetDriveByWireMessage message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean setDriveByWireMessageToBuffer(SetDriveByWireMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT SetDriveByWireMessage setDriveByWireMessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage setDriveByWireMessageToJausMessage(SetDriveByWireMessage message);

JAUS_EXPORT unsigned int setDriveByWireMessageSize(SetDriveByWireMessage message);

JAUS_EXPORT char* setDriveByWireMessageToString(SetDriveByWireMessage message);
#endif // SET_MODE_DRIVE_MESSAGE_H
