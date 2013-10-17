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
// File Name: setCameraOptionsMessage.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a SetCameraOptionsMessage

#ifndef SET_CAMERA_OPTIONS_MESSAGE_H
#define SET_CAMERA_OPTIONS_MESSAGE_H



#include "jaus.h"

#ifndef JAUS_CAMERA_OPTIONS
#define JAUS_CAMERA_OPTIONS
#define JAUS_CAMERA_OPTIONS_SATURACION_BIT		0
#define JAUS_CAMERA_OPTIONS_CONTRASTE_BIT		1
#define JAUS_CAMERA_OPTIONS_COMPRESION_BIT	2
#define JAUS_CAMERA_OPTIONS_HUE_BIT	3
#define JAUS_CAMERA_OPTIONS_BRILLO_BIT	4   
#define JAUS_CAMERA_OPTIONS_ACTION_BIT	5   
#endif




#ifndef JAUS_ACTION_CAMERA_ENUM
#define JAUS_ACTION_CAMERA_ENUM
typedef enum  
{
	NO_ACTION=0,
	IMAGE_START=1,
	IMAGE_STOP=2,
	CAMERA_RESET=3,
	IMAGE_ZOOM_IN=4,
	IMAGE_ZOOM_OUT=5

} JausActionCameraEnum;
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
	
	// Example from setGlobalPoseMessage
	//
	JausUnsignedShort presenceVector;
	//	JausDouble latitudeDegrees;				// Scaled Int (-90, 90)
	//	JausDouble longitudeDegrees;			// Scaled Int (-180, 180)
	//	JausDouble elevationMeters;				// Scaled Int (-10000, 35000)
	//	JausDouble positionRmsMeters;			// Scaled UInt (0, 100)
	//	JausDouble rollRadians;					// Scaled Short (-JAUS_PI, JAUS_PI)
	//	JausDouble pitchRadians;				// Scaled Short (-JAUS_PI, JAUS_PI)
	//	JausDouble yawRadians;					// Scaled Short (-JAUS_PI, JAUS_PI)
	//	JausDouble attitudeRmsRadians;			// Scaled Short (0, JAUS_PI)
	//	JausUnsignedInteger timeStamp;	

	JausByte cameraID;
	JausByte saturacion;
	JausDouble brillo;
	JausByte contraste;
	JausDouble hue;
	JausDouble compresion;

	//Actions
	JausActionCameraEnum action;
	
	
}SetCameraOptionsMessageStruct;

typedef SetCameraOptionsMessageStruct* SetCameraOptionsMessage;

JAUS_EXPORT SetCameraOptionsMessage setCameraOptionsMessageCreate(void);
JAUS_EXPORT void setCameraOptionsMessageDestroy(SetCameraOptionsMessage);

JAUS_EXPORT JausBoolean setCameraOptionsMessageFromBuffer(SetCameraOptionsMessage message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean setCameraOptionsMessageToBuffer(SetCameraOptionsMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT SetCameraOptionsMessage setCameraOptionsMessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage setCameraOptionsMessageToJausMessage(SetCameraOptionsMessage message);

JAUS_EXPORT unsigned int setCameraOptionsMessageSize(SetCameraOptionsMessage message);

JAUS_EXPORT char* setCameraOptionsMessageToString(SetCameraOptionsMessage message);
#endif // SET_CAMERA_OPTIONS_MESSAGE_H
