#ifndef SICKLMS1XX_H
#define	SICKLMS1XX_H

#include <vector>
#include <Modulo_Laser2D_Rear/SocketCommunication.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include <cstdlib>




#define NO_ERROR 0
#define ERROR_FRECUENCY 1
#define ERROR_ANGULAR_RESOLUTION 2
#define ERROR_FRECUENCY_AND_RESOLUTION 3
#define ERROR_SCAN_AREA 4
#define ERROR_OTHER 5

#define COMMAND_NO_ACEPTED 1
#define ERROR_START_DEVICE 0
#define ERROR_USER_LEVEL 0

#define LEVEL_MAINTENANCE_PERSONEL 2
#define LEVEL_AUTHORISED_CLIENT 3
#define LEVEL_SERVICE 4
#define PASSWORD 0xF4724744


//Tipos de LED
#define STOP 0
#define OK	1
#define Q1	2
#define Q2	3
#define CONTAMINATION 4

//Status
#define ON 1
#define OFF 0

using namespace std;
//Estado del laser
enum status
{
	undefined=0,
	initialisation=1,
	configuration=2,
	idle=3,
	rotating=4,
	in_preparation=5,
	ready=6,
	measurement=7
};

// Tipo de rotation
enum rotType
{
	none=0,
	pitching=1,
	rolling=2,
	free_rotation=3
};

//resolucion
enum resolution
{
	eightBits=0,
	sixteenBits=1
};

//tipo de filtro
enum typeFilter
{
	firstEcho=0,
	allEchoes=1,
	lastEcho=2
};

//Tipo de contaminacion
enum ContLevel 
{
	noContamination=0,
	warningContamination=1,
	errorContamination=2,
	seriousErrorContamination=3
};


//Informacion del dispositivo
struct deviceInformation
{
	unsigned short versionNumberMeasure; 
	unsigned short deviceNumber;
	unsigned int serialNumber;
	unsigned short deviceStatus;
};

//Informacion de la Fecha y Hora
struct timeInformation
{
	unsigned short year;
	unsigned char  month;
	unsigned char  day;
	unsigned char  hour;
	unsigned char  minute;
	unsigned char  second;
	unsigned int   microSecond;
};

//Informacion de la posicion del laser
struct positionInformation
{
	unsigned int positionFlag;
	float xPosition;
	float yPosition;
	float zPosition;
	float xRotation;
	float yRotation;
	float zRotation;
	unsigned int rotationType;
	
};


// Informacion de eventos
struct eventInformation
{
	unsigned short eventInfo;
	string eventType;
	unsigned int encoderPosition;
	unsigned int eventTime;
	int angularPosition;

};

//Informacion del estado del laser
struct statusInformation
{
	unsigned short messageCounter;
	unsigned short scanCounter;
	unsigned int powerUpDuration;
	unsigned int transmisionDuration;
	unsigned short inputStatus;
	unsigned short outputStatus;
	
	unsigned short statusLMS;
	bool statusTemperatureRange;
};

//Informacion de los encodersdel laser
struct encoderInformation
{
	unsigned short encoderPosition[3];
	unsigned short numberOfEncoders;
	unsigned int encoderSpeed[3];

};

//Informacion de los canales de informacion
struct channelsInformation
{
	
	string measuredDataContent;
	float scalingFactor;
	float scalingOffset;
	float startingAngle;
	float angularStepWidth;
	unsigned int numberData;
	vector<unsigned short> data;

};

//Otra informacion del laser
struct otherInformation
{
	unsigned short nameDevideFlag;
	string deviceName;
	unsigned short commentFlag;
	string commentContent;

};


//Informacion para una peticion al laser de información sobre este. Solo lectura
struct queryInformation
{
	unsigned short statusLMS;
	bool operatingTempRange;
	string time;
	string date;
	unsigned int led1;
	unsigned int led2;
	unsigned int led3;


};

//Informacion sobre los parametros basicos de medida del laser. Son parametros de configuracion del laser. Lectura y escritura
struct measuringParameters
{
	unsigned int scanningFrecuency;
	float angleResolution;
	unsigned short numberSegments;
	float startAngle;
	float stopAngle;

};

//Informacion sobre los parametros basicos de medida del laser. Son parametros de configuracion del laser. Lectura y escritura
struct conectionParameters
{
	string ipLaser;
	unsigned int portLaser;

};

//Informacion sobre el formato de los datos de salida del laser.Son parametros de configuración del laser. Solo escritura
struct dataOutput
{
	unsigned short outputChannel;
	bool remission;
	int Resolution;
	int Unit;
	unsigned short encoderData;
	bool positionFlag;
	bool deviceNameFlag;
	bool commentFlag;
	bool timeFlag;
	short outputInterval;
};

//Informacion sobre un scan del laser. Solo lectura
struct laserScan
{

	unsigned int scanningFrecuency;
	unsigned int measurementFrequency;

	timeInformation laserTime;
	deviceInformation laserDevice;
	unsigned short numberChannels16Bit;
	unsigned short numberChannels8Bit;

	channelsInformation laserChannel16[4];
	channelsInformation laserChannel8[4];

	positionInformation laserPosition;
	eventInformation laserEvent;
	statusInformation laserStatus;
	encoderInformation laserEncoders;
	otherInformation laserOthers;	


};

class Sicklms1xx
{

	public:

		SocketCommunication* laserSocket;
		//socketComunication* laserSocketAutomatic;		

	private:
		bool Automatic;
	
		int selecUserLevel(char userLevel,int password);
		int startDevice();
		void guardaDatos(laserScan*);
		int startMeasurement(int clientPassword);
		int stopMeasurement(int clientPassword);
	

	public:
		Sicklms1xx();

		int connect(short puerto,string ip);
		void disconnect();
		int queryStatus(queryInformation*);
		int getScanData(laserScan*,int);
		int getMeasuringParameters(measuringParameters*);
		int getContaminationLevel(unsigned short*);

		int setScanDataContinuous(bool state);
		int configureLaser(measuringParameters laserParametersMeasure,int password);
		int configureDataOutput(dataOutput configOutput,int clientPassword);
		int frEchoFilter(int typeFilter,int clientPassword);
		int setLmsOutput(char outputNumber, bool statusOutput);
		bool setLMSLEDs(char typeLED,bool statusLED);
		bool setLMSDisplay(char displayStatus);

		int saveDataPermanently(int clientPassword);
		



};
#endif	/* SICKLMS1XX_H */