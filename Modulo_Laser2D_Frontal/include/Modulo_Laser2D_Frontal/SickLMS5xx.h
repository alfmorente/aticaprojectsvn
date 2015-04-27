/**
 * @file   SickLMS5xx.h
 * @brief  Fichero Cabecera de gestion del Laser SickLMS5xx
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#ifndef SICKLMS5XX_H
#define	SICKLMS5XX_H

#include <vector>
#include <Modulo_Laser2D_Frontal/SocketCommunication.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include <cstdlib>




#define NO_ERROR -1  ///<Constante que indica que no ha habido ningun error
#define ERROR_FRECUENCY 1 ///<Constante que indica error al configurar la frecuencia del laser
#define ERROR_ANGULAR_RESOLUTION 2 ///<Constante que indica error al configurar la resolucion angular del laser
#define ERROR_FRECUENCY_AND_RESOLUTION 3///<Constante que indica error al configurar la frecuencia y la resolucion angular del laser
#define ERROR_SCAN_AREA 4 ///<Constante que indica error al configurar el area de escaneo
#define ERROR_OTHER 5 ///<Constante que indica error de configuracion de otro tipo

#define COMMAND_NO_ACEPTED 1 ///<Constante que indica que el comando ha sido aceptado por el laser
#define ERROR_START_DEVICE 0 ///<Constante que indica error al iniciar dispositivo
#define ERROR_USER_LEVEL 0 ///<Constante que indica error al configurar el modo de gestion del dispositivo

#define LEVEL_MAINTENANCE_PERSONEL 2 ///<Constante que indica gestion del laser a nivel de mantenimiento personal
#define LEVEL_AUTHORISED_CLIENT 3///<Constante que indica gestion del laser a nivel de servicio
#define LEVEL_SERVICE 4 ///<Constante que indica gestion del laser a nivelde servicio
#define PASSWORD 0xF4724744 ///<Constante que indica el password para el cliente autorizado

//Tipos de LED
#define STOP 0 ///<Constante que indica LED STOP
#define OK	1 ///<Constante que indica LED OK
#define Q1	2 ///<Constante que indica LED Q1
#define Q2	3 ///<Constante que indica LED Q2
#define CONTAMINATION 4 ///<Constante que indica LED Contaminacion

using namespace std;

/**
 * \enum status
 * \brief  Enumerado con los distintos estados del laser
 */
//Estado del laser
enum status
{
	undefined=0, ///< Undefinido  
	initialisation=1, ///< Inicializando
	configuration=2, ///< Configurando
	idle=3, ///< Ocioso
	rotating=4,///< Rotando
	in_preparation=5, ///< En proceso de preparación
	ready=6,///< Preparado
	measurement=7 ///< Midiendo
};

/**
 * \enum rotType
 * \brief  Enumerado con los distintos tipos de rotación
 */
// Tipo de rotation
enum rotType
{
	none=0, ///< Ninguno
	pitching=1, ///<Rotación pitch
	rolling=2, ///< Rotación Roll
	free_rotation=3 ///< Libre rotación
};

/**
 * \enum resolution
 * \brief  Enumerado con los distintos tipos de resoluciones
 */
//resolucion
enum resolution
{
	eightBits=0, ///< Resolución de 8 bits
	sixteenBits=1 ///< Resolución de 16 bits
};

/**
 * \enum typeFilter
 * \brief  Enumerado con los distintos tipos de filtros
 */
//tipo de filtro
enum typeFilter
{
	firstEcho=0, ///< Primer rebote del señal del laser
	allEchoes=1, ///< Todos los rebotes de la señal del laser
	lastEcho=2   ///< Último rebote de la señal del laser
};

/**
 * \enum ContLevel
 * \brief  Enumerado con los distintos tipos de contaminación
 */
//Tipo de contaminacion
enum ContLevel 
{
	noContamination=0, ///< No contaminación
	warningContamination=1, ///< Aviso de contaminación 
	errorContamination=2, ///< Error leve por contaminación
	seriousErrorContamination=3 ///<Error serio de contaminación
};


/**
 * \struct deviceInformation
 * \brief  Estructura con informacion del dispositivo
 */
//Informacion del dispositivo
struct deviceInformation
{
	unsigned short versionNumberMeasure; 
	unsigned short deviceNumber;
	unsigned int serialNumber;
	unsigned short deviceStatus;
};

/**
 * \struct timeInformation
 * \brief  Estructura con informacion de la fecha
 */
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

/**
 * \struct positionInformation
 * \brief  Estructura con informacion de la posicion del dispositivo
 */
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

/**
 * \struct eventInformation
 * \brief  Estructura con informacion de los eventos del laser
 */
// Informacion de eventos
struct eventInformation
{
	unsigned short eventInfo;
	string eventType;
	unsigned int encoderPosition;
	unsigned int eventTime;
	int angularPosition;

};

/**
 * \struct statusInformation
 * \brief  Estructura con informacion del estado del laser
 */
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

/**
 * \struct encoderInformation
 * \brief  Estructura con informacion de los encoders del laser
 */
//Informacion de los encoders del laser
struct encoderInformation
{
	unsigned short encoderPosition[3];
	unsigned short numberOfEncoders;
	unsigned int encoderSpeed[3];

};

/**
 * \struct channelsInformation
 * \brief  Estructura con informacion de los canales de salida del laser
 */
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

/**
 * \struct otherInformation
 * \brief  Estructura con otra informacion del laser
 */
//Otra informacion del laser
struct otherInformation
{
	unsigned short nameDevideFlag;
	string deviceName;
	unsigned short commentFlag;
	string commentContent;

};


/**
 * \struct queryInformation
 * \brief  Estructura con informacion de la respuesta a un Query al laser
 */
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

/**
 * \struct measuringParameters
 * \brief  Estructura con informacion de la respuesta a una peticion de los parametros de medida
 */
//Informacion sobre los parametros basicos de medida del laser. Son parametros de configuracion del laser. Lectura y escritura
struct measuringParameters
{
	unsigned int scanningFrecuency;
	float angleResolution;
	unsigned short numberSegments;
	float startAngle;
	float stopAngle;

};

/**
 * \struct conectionParameters
 * \brief  Estructura con informacion de conexion con el laser
 */
//Informacion sobre los parametros basicos de medida del laser. Son parametros de configuracion del laser. Lectura y escritura
struct conectionParameters
{
	string ipLaser;
	unsigned int portLaser;

};

/**
 * \struct dataOutput
 * \brief  Estructura con informacion de la respuesta a una peticion de los datos de salida del laser 
 */
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

/**
 * \struct laserScan
 * \brief  Estructura que incluye toda la informacion de laser en un escaneo de este 
 */
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

/**
 * \class Sicklms5xx
 * \brief Clase que engloba el protocolo para la comunicacion con el laser
 */
class Sicklms5xx
{

	public:
		SocketCommunication* laserSocket;	

	private:
		bool Automatic;
	
		int selecUserLevel(char userLevel,int password);
		int startDevice();
		void guardaDatos(laserScan*);
		int startMeasurement(int clientPassword);
		int stopMeasurement(int clientPassword);
	

	public:
		Sicklms5xx();

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
		int saveDataPermanently(int clientPassword);
		

};
#endif	/* SICKLMS1XX_H */
