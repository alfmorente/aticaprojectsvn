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
// File Name: reportUSVObservationsConfig8Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a ReportUSVObservationsConfig8Message

#ifndef REPORT_USV_OBSERVATIONS_CONFIG_8_MESSAGE_H
#define REPORT_USV_OBSERVATIONS_CONFIG_8_MESSAGE_H

#include "jaus.h"

#ifndef   JAUS_8_PV 
#define  JAUS_8_PV_PAN_BIT    			0 //Short
#define  JAUS_8_PV_TILT_BIT    			1 //Short
#define  JAUS_8_PV_ZOOM_BIT                	2 //Bool3
#define  JAUS_8_PV_DAY_NIGHT_FUNCTION_BIT    	3 //Bool1
#define  JAUS_8_PV_INFRARED_FILTER_BIT    	4 //Bool2
#define  JAUS_8_PV_TRACKING_FUNCTION_BIT    	5 //Bool3
#define  JAUS_8_PV_WIPER_BIT                	6 //Bool3


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
  JausByte presenceVector; 
		
  JausDouble active_pan;				// Scaled Short (-180,180)
  JausDouble active_tilt;				// Scaled Short (-90,90)
  JausDouble active_zoom;                               // Scaled Short (0,100)
  JausBoolean active_day_night_function;		
  JausBoolean active_infrared_filter;			
  JausBoolean active_tracking_function;		
  JausBoolean active_wiper;		
  
}ReportUSVObservationsConfig8MessageStruct;

typedef ReportUSVObservationsConfig8MessageStruct* ReportUSVObservationsConfig8Message;

JAUS_EXPORT ReportUSVObservationsConfig8Message reportUSVObservationsConfig8MessageCreate(void);
JAUS_EXPORT void reportUSVObservationsConfig8MessageDestroy(ReportUSVObservationsConfig8Message);

JAUS_EXPORT JausBoolean reportUSVObservationsConfig8MessageFromBuffer(ReportUSVObservationsConfig8Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean reportUSVObservationsConfig8MessageToBuffer(ReportUSVObservationsConfig8Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT ReportUSVObservationsConfig8Message reportUSVObservationsConfig8MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage reportUSVObservationsConfig8MessageToJausMessage(ReportUSVObservationsConfig8Message message);

JAUS_EXPORT unsigned int reportUSVObservationsConfig8MessageSize(ReportUSVObservationsConfig8Message message);

JAUS_EXPORT char* reportUSVObservationsConfig8MessageToString(ReportUSVObservationsConfig8Message message);
#endif // REPORT_USV_OBSERVATIONS_CONFIG_8_MESSAGE_H
