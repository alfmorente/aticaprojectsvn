// File Name: environmentTelemetry.h
//
// Description: This file defines the attributes of a EnvironmentTelemetryMessage

#ifndef ENVIRONMENT_TELEMETRY_MESSAGE_H
#define ENVIRONMENT_TELEMETRY_MESSAGE_H

#include "jaus.h"
#define JAUS_ENVIRONMENT_TELEMETRY 0XD000

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
	
	JausDouble averageWindSpeed;		// Scaled Short (0,60)
	JausDouble averageWindDirection;	// Scaled Short (0,360)
	JausDouble airTemperature;			// Scaled Short (-52,60)
	JausDouble relativeHumidity;		// Scaled Short (0,100)
	JausDouble barometricPressure;		// Scaled Short (600,1100)
	JausDouble rainfall;				
	JausInteger rainDuration;
	JausDouble hailAccumulated;
	
}EnvironmentTelemetryMessageStruct;

typedef EnvironmentTelemetryMessageStruct* EnvironmentTelemetryMessage;

JAUS_EXPORT EnvironmentTelemetryMessage environmentTelemetryMessageCreate(void);
JAUS_EXPORT void environmentTelemetryMessageDestroy(EnvironmentTelemetryMessage);

JAUS_EXPORT JausBoolean environmentTelemetryMessageFromBuffer(EnvironmentTelemetryMessage message, unsigned char* buffer, unsigned int bufferSizeBytes);
JAUS_EXPORT JausBoolean environmentTelemetryMessageToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);

JAUS_EXPORT EnvironmentTelemetryMessage environmentTelemetryMessageFromJausMessage(JausMessage jausMessage);
JAUS_EXPORT JausMessage environmentTelemetryMessageToJausMessage(EnvironmentTelemetryMessage message);

JAUS_EXPORT unsigned int environmentTelemetryMessageSize(EnvironmentTelemetryMessage message);

JAUS_EXPORT char* environmentTelemetryMessageToString(EnvironmentTelemetryMessage message);
#endif // ENVIRONMENT_TELEMETRY_MESSAGE_H
