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
// File Name: aditionalGPSINSInfo4Message.h
//
// Written By: Danny Kent (jaus AT dannykent DOT com), Tom Galluzzo 
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description: This file defines the attributes of a AditionalGPSINSInfo4Message

#ifndef ADITIONAL_GPSINS_INFO_4_MESSAGE_H
#define ADITIONAL_GPSINS_INFO_4_MESSAGE_H

#include "jaus.h"

#ifndef  JAUS_4_PV 
#define  JAUS_4_PV_LONGITUDINAL_ACC_BIT                         0
#define  JAUS_4_PV_LATERAL_ACC_BIT                              1
#define  JAUS_4_PV_VERTICAL_ACC_BIT                             2
#define  JAUS_4_PV_GPSINS_STATUS_BIT                            3
#define  JAUS_4_PV_MEASURE_QUALITY_BIT                          4
#define  JAUS_4_PV_ST_LAT_DEVIATION_BIT    			5
#define  JAUS_4_PV_ST_LON_DEVIATION_BIT    			6
#define  JAUS_4_PV_ST_ALT_DEVIATION_BIT    			7
#define  JAUS_4_PV_DGPS_CORRECTIONS_BIT    			8
#define  JAUS_4_PV_GPSINS_AVAILABILITY_BIT    			9
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
        
        // Presence vector
  JausShort presenceVector;
  
  JausDouble longitudinal_acc;                   // Scaled Int (-65534,65534), Res: 3e-5
  JausDouble lateral_acc;                       // Scaled Int (-65534,65534), Res: 3e-5
  JausDouble vertical_acc;                      // Scaled Int (-65534,65534), Res: 3e-5
  JausByte gpsins_status;                       // Scaled Byte (0,10), Enum
  JausDouble measure_quality[3];                  // Scaled Short (0,100)x3, Res:  0.3906
  // COMO HACERLO
  /*JausDouble measure_quality;                  // Scaled Short (0,100), Res:  0.3906
  JausDouble calidadDeLaMedida1;                  // Scaled Short (0,100), Res:  0.3906
  JausDouble calidadDeLaMedida2;                  // Scaled Short (0,100), Res:  0.3906*/
  //
  JausDouble st_lat_deviation;         // Scaled Short (0,1000), Res:  0.0153
  JausDouble st_lon_deviation;        // Scaled Short (0,1000), Res:  0.0153
  JausDouble st_alt_deviation;          // Scaled Short (0,1000), Res:  0.0153
  JausBoolean dgps_corrections;
  JausBoolean gpsins_availability;


}AditionalGPSINSInfo4MessageStruct;

typedef AditionalGPSINSInfo4MessageStruct* AditionalGPSINSInfo4Message;

JAUS_EXPORT AditionalGPSINSInfo4Message aditionalGPSINSInfo4MessageCreate(void);
JAUS_EXPORT void aditionalGPSINSInfo4MessageDestroy(AditionalGPSINSInfo4Message);

JAUS_EXPORT JausBoolean aditionalGPSINSInfo4MessageFromBuffer(AditionalGPSINSInfo4Message message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean aditionalGPSINSInfo4MessageToBuffer(AditionalGPSINSInfo4Message message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT AditionalGPSINSInfo4Message aditionalGPSINSInfo4MessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage aditionalGPSINSInfo4MessageToJausMessage(AditionalGPSINSInfo4Message message);

JAUS_EXPORT unsigned int aditionalGPSINSInfo4MessageSize(AditionalGPSINSInfo4Message message);

JAUS_EXPORT char* aditionalGPSINSInfo4MessageToString(AditionalGPSINSInfo4Message message);
#endif // ADITIONAL_GPSINS_INFO_4_MESSAGE_H
