// File Name: environmentTelemetryMessage.c
//
// Description: This file defines the functionality of a EnvironmentTelemetryMessage

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "jaus.h"

static const int commandCode = JAUS_ENVIRONMENT_TELEMETRY;
static const int maxDataSizeBytes = 0;

static JausBoolean headerFromBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static JausBoolean headerToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int headerToString(EnvironmentTelemetryMessage message, char **buf);

static JausBoolean dataFromBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static int dataToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes);
static void dataInitialize(EnvironmentTelemetryMessage message);
static void dataDestroy(EnvironmentTelemetryMessage message);
static unsigned int dataSize(EnvironmentTelemetryMessage message);

// ************************************************************************************************************** //
//                                    USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

// Initializes the message-specific fields
static void dataInitialize(EnvironmentTelemetryMessage message)
{
	//Experimental message bit
	message->properties.expFlag = JAUS_EXPERIMENTAL_MESSAGE;
	
	//Set initial values
	message->averageWindSpeed = newJausDouble(0);			//Scaled short (0,60)
	message->averageWindDirection = newJausDouble(0);		//Scaled short (0,360)
	message->airTemperature = newJausDouble(0);				//Scaled short (-52,60)
	message->relativeHumidity = newJausDouble(0);			//Scaled short (0,100)
	message->barometricPressure = newJausDouble(600);		//Scaled short (600,1100)
	message->rainfall = newJausDouble(0);
	message->rainDuration = newJausInteger(0),
	message->hailAccumulated = newJausDouble(0);
}

// Destructs the message-specific fields
static void dataDestroy(EnvironmentTelemetryMessage message)
{
	// Free message fields
}

// Return boolean of success
static JausBoolean dataFromBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	JausShort tempShort = 0;
	
	if(bufferSizeBytes == message->dataSize)
	{
		// Unpack Message Fields from Buffer
		// Average Wind Speed Scaled short (0,60)
		if(!jausShortFromBuffer(&tempShort, buffer+index, bufferSizeBytes-index)) 
			return JAUS_FALSE;
	    index += JAUS_SHORT_SIZE_BYTES;
	    message->averageWindSpeed = jausShortToDouble(tempShort, 0, 60);
	    
	    // Average Wind Direction Scaled Short (0,360)
	    if(!jausShortFromBuffer(&tempShort, buffer+index, bufferSizeBytes-index)) 
			return JAUS_FALSE;
	    index += JAUS_SHORT_SIZE_BYTES;
	    message->averageWindDirection = jausShortToDouble(tempShort, 0, 360);
	    
	    // Air Temperature Scaled Short (-52,60)
	    if(!jausShortFromBuffer(&tempShort, buffer+index, bufferSizeBytes-index)) 
			return JAUS_FALSE;
	    index += JAUS_SHORT_SIZE_BYTES;
	    message->airTemperature = jausShortToDouble(tempShort, -52, 60);
	    
   	    // Relative Humidity Scaled Short (0,10)
	    if(!jausShortFromBuffer(&tempShort, buffer+index, bufferSizeBytes-index)) 
			return JAUS_FALSE;
	    index += JAUS_SHORT_SIZE_BYTES;
	    message->relativeHumidity = jausShortToDouble(tempShort, 0, 100);
	    
   	    // Barometric Pressure Scaled Short (600,1100)
	    if(!jausShortFromBuffer(&tempShort, buffer+index, bufferSizeBytes-index)) 
			return JAUS_FALSE;
	    index += JAUS_SHORT_SIZE_BYTES;
	    message->barometricPressure = jausShortToDouble(tempShort, 600, 1100);

		// Rainfall 
		if(!jausDoubleFromBuffer(&message->rainfall, buffer+index, bufferSizeBytes-index))
			return JAUS_FALSE;
		index += JAUS_DOUBLE_SIZE_BYTES;
		
		// Rain Duration
		if(!jausIntegerFromBuffer(&message->rainDuration, buffer+index, bufferSizeBytes-index))
			return JAUS_FALSE;
		index += JAUS_INTEGER_SIZE_BYTES;
		
		// Hail Accumulated
		if(!jausDoubleFromBuffer(&message->hailAccumulated, buffer+index, bufferSizeBytes-index))
			return JAUS_FALSE;
		index += JAUS_DOUBLE_SIZE_BYTES;
		return JAUS_TRUE;
	}
	else
	{
		return JAUS_FALSE;
	}
}

// Returns number of bytes put into the buffer
static int dataToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	JausShort tempShort =0;
	
	if(bufferSizeBytes >= dataSize(message))
	{
	// Pack Message Fields to Buffer
	// Average Wind Speed Scaled Short (0,60)
	tempShort = jausShortFromDouble(message->averageWindSpeed, 0, 60);
	// pack
	if(!jausShortToBuffer(tempShort, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_SHORT_SIZE_BYTES;
	
	
	// Average Wind Direction Scaled Short (0,360)
	tempShort = jausShortFromDouble(message->averageWindDirection, 0, 360);
	// pack
	if(!jausShortToBuffer(tempShort, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_SHORT_SIZE_BYTES;
	
	
	// Air Temperature Scaled Short (-52,60)
	tempShort = jausShortFromDouble(message->airTemperature, -52, 60);
	// pack
	if(!jausShortToBuffer(tempShort, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_SHORT_SIZE_BYTES;
	
	
	// Relative Humidity Scaled Short (0,100)
	tempShort = jausShortFromDouble(message->relativeHumidity, 0, 100);
	// pack
	if(!jausShortToBuffer(tempShort, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_SHORT_SIZE_BYTES;
	
	
	// Barometric Pressure Scaled Short (0,60)
	tempShort = jausShortFromDouble(message->barometricPressure, 600, 1100);
	// pack
	if(!jausShortToBuffer(tempShort, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_SHORT_SIZE_BYTES;
	
	
	// Rainfall
	// pack
	if(!jausDoubleToBuffer(message->rainfall, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_DOUBLE_SIZE_BYTES;
		
	// Rain Duration
	// pack
	if(!jausIntegerToBuffer(message->rainDuration, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_INTEGER_SIZE_BYTES;
		
	// Hail Accumulated
	// pack
	if(!jausDoubleToBuffer(message->hailAccumulated, buffer+index, bufferSizeBytes-index)) 
		return JAUS_FALSE;
	index += JAUS_DOUBLE_SIZE_BYTES;
	
	}

	return index;
}

static int dataToString(EnvironmentTelemetryMessage message, char **buf)
{
  //message already verified 

  //Setup temporary string buffer
  
  //Fill in maximum size of output string
  unsigned int bufSize = 400;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);
  
  strcat((*buf), "\nAverage Wind Speed(m/s): ");
  jausDoubleToString(message->averageWindSpeed, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nAverage Wind Direction(degrees): ");
  jausDoubleToString(message->averageWindDirection, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nAir Temperature(Celsius): ");
  jausDoubleToString(message->airTemperature, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nRelative Humidity(%): ");
  jausDoubleToString(message->relativeHumidity, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nBarometric Pressure(hPa): ");
  jausDoubleToString(message->barometricPressure, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nRainfall(mm): ");
  jausDoubleToString(message->rainfall, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nRain Duration(s): ");
  jausIntegerToString(message->rainDuration, (*buf)+strlen(*buf));
    
  strcat((*buf), "\nHail Accumulated(hit/cm2): ");
  jausDoubleToString(message->hailAccumulated, (*buf)+strlen(*buf));
  	  
  return strlen((*buf));
}

// Returns number of bytes put into the buffer
static unsigned int dataSize(EnvironmentTelemetryMessage message)
{
	int index = 0;
	
	// Average Wind Speed
	index += JAUS_SHORT_SIZE_BYTES;
	
	// Average Wind Direction
	index += JAUS_SHORT_SIZE_BYTES;
	
	// Air Temperature
	index += JAUS_SHORT_SIZE_BYTES;
	
	// Relative Humidity 
	index += JAUS_SHORT_SIZE_BYTES;
	
	// Barometric Pressure
	index += JAUS_SHORT_SIZE_BYTES;
	
	// Rainfall
	index += JAUS_DOUBLE_SIZE_BYTES;
	
	// Rain Duration
	index += JAUS_INTEGER_SIZE_BYTES;

	// Hail Accumulated
	index += JAUS_DOUBLE_SIZE_BYTES;
		
	return index;
}

// ************************************************************************************************************** //
//                                    NON-USER CONFIGURED FUNCTIONS
// ************************************************************************************************************** //

EnvironmentTelemetryMessage environmentTelemetryMessageCreate(void)
{
	EnvironmentTelemetryMessage message;

	message = (EnvironmentTelemetryMessage)malloc( sizeof(EnvironmentTelemetryMessageStruct) );
	if(message == NULL)
	{
		return NULL;
	}
	
	// Initialize Values
	message->properties.priority = JAUS_DEFAULT_PRIORITY;
	message->properties.ackNak = JAUS_ACK_NAK_NOT_REQUIRED;
	message->properties.scFlag = JAUS_NOT_SERVICE_CONNECTION_MESSAGE;
	message->properties.expFlag = JAUS_NOT_EXPERIMENTAL_MESSAGE;
	message->properties.version = JAUS_VERSION_3_3;
	message->properties.reserved = 0;
	message->commandCode = commandCode;
	message->destination = jausAddressCreate();
	message->source = jausAddressCreate();
	message->dataFlag = JAUS_SINGLE_DATA_PACKET;
	message->dataSize = maxDataSizeBytes;
	message->sequenceNumber = 0;
	
	dataInitialize(message);
	message->dataSize = dataSize(message);
	
	return message;	
}

void environmentTelemetryMessageDestroy(EnvironmentTelemetryMessage message)
{
	dataDestroy(message);
	jausAddressDestroy(message->source);
	jausAddressDestroy(message->destination);
	free(message);
}

JausBoolean environmentTelemetryMessageFromBuffer(EnvironmentTelemetryMessage message, unsigned char* buffer, unsigned int bufferSizeBytes)
{
	int index = 0;
	
	if(headerFromBuffer(message, buffer+index, bufferSizeBytes-index))
	{
		index += JAUS_HEADER_SIZE_BYTES;
		if(dataFromBuffer(message, buffer+index, bufferSizeBytes-index))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE;
		}
	}
	else
	{
		return JAUS_FALSE;
	}
}

JausBoolean environmentTelemetryMessageToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < environmentTelemetryMessageSize(message))
	{
		return JAUS_FALSE; //improper size	
	}
	else
	{	
		message->dataSize = dataToBuffer(message, buffer+JAUS_HEADER_SIZE_BYTES, bufferSizeBytes - JAUS_HEADER_SIZE_BYTES);
		if(headerToBuffer(message, buffer, bufferSizeBytes))
		{
			return JAUS_TRUE;
		}
		else
		{
			return JAUS_FALSE; // headerToEnvironmentTelemetryBuffer failed
		}
	}
}

EnvironmentTelemetryMessage environmentTelemetryMessageFromJausMessage(JausMessage jausMessage)
{
	EnvironmentTelemetryMessage message;
	
	if(jausMessage->commandCode != commandCode)
	{
		return NULL; // Wrong message type
	}
	else
	{
		message = (EnvironmentTelemetryMessage)malloc( sizeof(EnvironmentTelemetryMessageStruct) );
		if(message == NULL)
		{
			return NULL;
		}
		
		message->properties.priority = jausMessage->properties.priority;
		message->properties.ackNak = jausMessage->properties.ackNak;
		message->properties.scFlag = jausMessage->properties.scFlag;
		message->properties.expFlag = jausMessage->properties.expFlag;
		message->properties.version = jausMessage->properties.version;
		message->properties.reserved = jausMessage->properties.reserved;
		message->commandCode = jausMessage->commandCode;
		message->destination = jausAddressCreate();
		*message->destination = *jausMessage->destination;
		message->source = jausAddressCreate();
		*message->source = *jausMessage->source;
		message->dataSize = jausMessage->dataSize;
		message->dataFlag = jausMessage->dataFlag;
		message->sequenceNumber = jausMessage->sequenceNumber;
		
		// Unpack jausMessage->data
		if(dataFromBuffer(message, jausMessage->data, jausMessage->dataSize))
		{
			return message;
		}
		else
		{
			return NULL;
		}
	}
}

JausMessage environmentTelemetryMessageToJausMessage(EnvironmentTelemetryMessage message)
{
	JausMessage jausMessage;
	int size;
	
	jausMessage = (JausMessage)malloc( sizeof(struct JausMessageStruct) );
	if(jausMessage == NULL)
	{
		return NULL;
	}	
	
	jausMessage->properties.priority = message->properties.priority;
	jausMessage->properties.ackNak = message->properties.ackNak;
	jausMessage->properties.scFlag = message->properties.scFlag;
	jausMessage->properties.expFlag = message->properties.expFlag;
	jausMessage->properties.version = message->properties.version;
	jausMessage->properties.reserved = message->properties.reserved;
	jausMessage->commandCode = message->commandCode;
	jausMessage->destination = jausAddressCreate();
	*jausMessage->destination = *message->destination;
	jausMessage->source = jausAddressCreate();
	*jausMessage->source = *message->source;
	jausMessage->dataSize = dataSize(message);
	jausMessage->dataFlag = message->dataFlag;
	jausMessage->sequenceNumber = message->sequenceNumber;
	
	jausMessage->data = (unsigned char *)malloc(jausMessage->dataSize);
	jausMessage->dataSize = dataToBuffer(message, jausMessage->data, jausMessage->dataSize);
		
	return jausMessage;
}

unsigned int environmentTelemetryMessageSize(EnvironmentTelemetryMessage message)
{
	return (unsigned int)(dataSize(message) + JAUS_HEADER_SIZE_BYTES);
}

char* environmentTelemetryMessageToString(EnvironmentTelemetryMessage message)
{
  if(message)
  {
    char* buf1 = NULL;
    char* buf2 = NULL;
    
    int returnVal;
    
    //Print the message header to the string buffer
    returnVal = headerToString(message, &buf1);
    
    //Print the message data fields to the string buffer
    returnVal += dataToString(message, &buf2);
    
    char* buf;
    buf = (char*)malloc(strlen(buf1)+strlen(buf2));
    strcpy(buf, buf1);
    strcat(buf, buf2);
    
    free(buf1);
    free(buf2);
    
    return buf;
  }
  else
  {
    char* buf = "Invalid EnvironmentTelemetry Message";
    char* msg = (char*)malloc(strlen(buf)+1);
    strcpy(msg, buf);
    return msg;
  }
}
//********************* PRIVATE HEADER FUNCTIONS **********************//

static JausBoolean headerFromBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{
		// unpack header
		message->properties.priority = (buffer[0] & 0x0F);
		message->properties.ackNak	 = ((buffer[0] >> 4) & 0x03);
		message->properties.scFlag	 = ((buffer[0] >> 6) & 0x01);
		message->properties.expFlag	 = ((buffer[0] >> 7) & 0x01);
		message->properties.version	 = (buffer[1] & 0x3F);
		message->properties.reserved = ((buffer[1] >> 6) & 0x03);
		
		message->commandCode = buffer[2] + (buffer[3] << 8);
	
		message->destination->instance = buffer[4];
		message->destination->component = buffer[5];
		message->destination->node = buffer[6];
		message->destination->subsystem = buffer[7];
	
		message->source->instance = buffer[8];
		message->source->component = buffer[9];
		message->source->node = buffer[10];
		message->source->subsystem = buffer[11];
		
		message->dataSize = buffer[12] + ((buffer[13] & 0x0F) << 8);

		message->dataFlag = ((buffer[13] >> 4) & 0x0F);

		message->sequenceNumber = buffer[14] + (buffer[15] << 8);
		
		return JAUS_TRUE;
	}
}

static JausBoolean headerToBuffer(EnvironmentTelemetryMessage message, unsigned char *buffer, unsigned int bufferSizeBytes)
{
	JausUnsignedShort *propertiesPtr = (JausUnsignedShort*)&message->properties;
	
	if(bufferSizeBytes < JAUS_HEADER_SIZE_BYTES)
	{
		return JAUS_FALSE;
	}
	else
	{	
		buffer[0] = (unsigned char)(*propertiesPtr & 0xFF);
		buffer[1] = (unsigned char)((*propertiesPtr & 0xFF00) >> 8);

		buffer[2] = (unsigned char)(message->commandCode & 0xFF);
		buffer[3] = (unsigned char)((message->commandCode & 0xFF00) >> 8);

		buffer[4] = (unsigned char)(message->destination->instance & 0xFF);
		buffer[5] = (unsigned char)(message->destination->component & 0xFF);
		buffer[6] = (unsigned char)(message->destination->node & 0xFF);
		buffer[7] = (unsigned char)(message->destination->subsystem & 0xFF);

		buffer[8] = (unsigned char)(message->source->instance & 0xFF);
		buffer[9] = (unsigned char)(message->source->component & 0xFF);
		buffer[10] = (unsigned char)(message->source->node & 0xFF);
		buffer[11] = (unsigned char)(message->source->subsystem & 0xFF);
		
		buffer[12] = (unsigned char)(message->dataSize & 0xFF);
		buffer[13] = (unsigned char)((message->dataFlag & 0xFF) << 4) | (unsigned char)((message->dataSize & 0x0F00) >> 8);

		buffer[14] = (unsigned char)(message->sequenceNumber & 0xFF);
		buffer[15] = (unsigned char)((message->sequenceNumber & 0xFF00) >> 8);
		
		return JAUS_TRUE;
	}
}

static int headerToString(EnvironmentTelemetryMessage message, char **buf)
{
  //message existance already verified 

  //Setup temporary string buffer
  
  unsigned int bufSize = 500;
  (*buf) = (char*)malloc(sizeof(char)*bufSize);
  
  strcpy((*buf), jausCommandCodeString(message->commandCode) );
  strcat((*buf), " (0x");
  sprintf((*buf)+strlen(*buf), "%04X", message->commandCode);

  strcat((*buf), ")\nReserved: ");
  jausUnsignedShortToString(message->properties.reserved, (*buf)+strlen(*buf));

  strcat((*buf), "\nVersion: ");
  switch(message->properties.version)
  {
    case 0:
      strcat((*buf), "2.0 and 2.1 compatible");
      break;
    case 1:
      strcat((*buf), "3.0 through 3.1 compatible");
      break;
    case 2:
      strcat((*buf), "3.2 and 3.3 compatible");
      break;
    default:
      strcat((*buf), "Reserved for Future: ");
      jausUnsignedShortToString(message->properties.version, (*buf)+strlen(*buf));
      break;
  }

  strcat((*buf), "\nExp. Flag: ");
  if(message->properties.expFlag == 0)
    strcat((*buf), "Not Experimental");
  else 
    strcat((*buf), "Experimental");
  
  strcat((*buf), "\nSC Flag: ");
  if(message->properties.scFlag == 1)
    strcat((*buf), "Service Connection");
  else
    strcat((*buf), "Not Service Connection");
  
  strcat((*buf), "\nACK/NAK: ");
  switch(message->properties.ackNak)
  {
  case 0:
    strcat((*buf), "None");
    break;
  case 1:
    strcat((*buf), "Request ack/nak");
    break;
  case 2:
    strcat((*buf), "nak response");
    break;
  case 3:
    strcat((*buf), "ack response");
    break;
  default:
    break;
  }
  
  strcat((*buf), "\nPriority: ");
  if(message->properties.priority < 12)
  {
    strcat((*buf), "Normal Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }
  else
  {
    strcat((*buf), "Safety Critical Priority ");
    jausUnsignedShortToString(message->properties.priority, (*buf)+strlen(*buf));
  }
  
  strcat((*buf), "\nSource: ");
  jausAddressToString(message->source, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nDestination: ");
  jausAddressToString(message->destination, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nData Size: ");
  jausUnsignedIntegerToString(message->dataSize, (*buf)+strlen(*buf));
  
  strcat((*buf), "\nData Flag: ");
  jausUnsignedIntegerToString(message->dataFlag, (*buf)+strlen(*buf));
  switch(message->dataFlag)
  {
    case 0:
      strcat((*buf), " Only data packet in single-packet stream");
      break;
    case 1:
      strcat((*buf), " First data packet in muti-packet stream");
      break;
    case 2:
      strcat((*buf), " Normal data packet");
      break;
    case 4:
      strcat((*buf), " Retransmitted data packet");
      break;
    case 8:
      strcat((*buf), " Last data packet in stream");
      break;
    default:
      strcat((*buf), " Unrecognized data flag code");
      break;
  }
  
  strcat((*buf), "\nSequence Number: ");
  jausUnsignedShortToString(message->sequenceNumber, (*buf)+strlen(*buf));
  
  return strlen((*buf));
  

}
