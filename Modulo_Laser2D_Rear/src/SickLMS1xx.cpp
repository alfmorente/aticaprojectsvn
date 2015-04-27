/**
 * @file   SickLMS1xx.cpp
 * @brief  Fichero fuente de gestion del Laser SickLMS1xx
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include <Modulo_Laser2D_Rear/SickLMS1xx.h>

/**
 * Constructor de la clase
 */
Sicklms1xx::Sicklms1xx()
{
	Automatic=false;
}


/**
 * Metodo para conectar con el laser
 * @param[in] puerto Puerto del laser
 * @param[in] ip  IP del laser
 */
/** connect: Metodo publico que conecta con el Laser
    
    Parametros de entrada: 
	-puerto: puerto donde se conecta
	-ip: ip del laser
    Parametro de salida: booleano que indica si la conexion se hizo de manera correcta
**/
int Sicklms1xx::connect(short puerto,string ip)
{
	bool errorSocket;
	laserSocket=new SocketCommunication(&errorSocket);
	if(errorSocket<0)
		return LASER_SOCKET_FAIL;

	if(!laserSocket->connectLaser(puerto,ip))
		return CONNECTION_ERROR;
	else
		return NO_ERROR;
}

/**
 * Metodo para desconectar del laser
 */
/** disconnect: Metodo publico que conecta con el Laser
    
    Parametros de entrada: 
	-puerto: puerto donde se conecta
	-ip: ip del laser
    Parametro de salida: No tiene
**/
void Sicklms1xx::disconnect()
{
	laserSocket->disconnectLaser();
	delete laserSocket;
}

/**
 * Metodo para poner a medir el laser
 * @param[in] clientPassword Contraseña para realizar esta accion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** startMeasurement: Metodo privado que le comunica al laser que empieze a medir 
    
    Parametros de entrada: 
	-clientPassword: password del modo cliente para que se pueda llevar a cabop el comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::startMeasurement(int clientPassword)
{

	int error;
	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=NO_ERROR)
		return error;


	laserSocket->createMessage("sMN","LMCstartmeas",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","LMCstartmeas",1))
		return INCORRECT_ANSWER;

	if(convertToInteger(laserSocket->data[0])==COMMAND_NO_ACEPTED)
		return START_MEASURE_NA;
	else
	{	
	
		error=startDevice();
		if(error!=NO_ERROR)
			return error;
		else
		{
			sleep(1);
			return NO_ERROR;
		}
	}
}

/**
 * Metodo para que pare de medir el laser
 * @param[in] clientPassword Contraseña para realizar esta accion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** stopMeasurement: Metodo privado que le comunica al laser que pare de medir 
    
    Parametros de entrada: 
	-clientPassword: password del modo cliente para que se pueda llevar a cabop el comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::stopMeasurement(int clientPassword)
{
	int error;
	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=NO_ERROR)
		return error;

	laserSocket->createMessage("sMN","LMCstopmeas",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","LMCstopmeas",1))
		return INCORRECT_ANSWER;


	if(convertToInteger(laserSocket->data[0])==COMMAND_NO_ACEPTED)
		return STOP_MEASURE_NA;
	else
		return NO_ERROR;
}

/**
 * Metodo para obtener el estado del laser
 * @param[io] laserQuery Estructura donde poner la respuesta de la trama
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** queryStatus: Metodo publico que le pide al laser informacion sobre su estado
    
    Parametros de entrada: 
	-laserQuery: estructura del tipo queryInformation (Ver laser.h )
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::queryStatus(queryInformation* laserQuery)
{
	laserSocket->createMessage("sRN","STlms",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sRA","STlms",9))
		return INCORRECT_ANSWER;


	if(laserQuery!=NULL)
	{	
		laserQuery->statusLMS=convertToUnsignedInteger(laserSocket->data[0]);
		laserQuery->operatingTempRange=convertToInteger(laserSocket->data[1]);
		laserQuery->time=laserSocket->data[3];
		laserQuery->date=laserSocket->data[5];
		laserQuery->led1=convertToInteger(laserSocket->data[6]);
		laserQuery->led2=convertToInteger(laserSocket->data[7]);
		laserQuery->led3=convertToInteger(laserSocket->data[8]);
	}
	
	return NO_ERROR;
}

/**
 * Metodo para habilitar/deshabilitar el escaneo continuo
 * @param[in] state Indica si se quiere habilitar o deshabilitar
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** setScanDataContinuous: Metodo publico que habilita o deshabilita el laser en modo continuo
    
    Parametros de entrada: 
	-state: Indica si se activa o desactiva el modo continuo
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::setScanDataContinuous(bool state)
{
	stringstream aux;
	string data;
	data=convertTostring(state);
	
	if(state!=Automatic)
	{
		laserSocket->createMessage("sEN","LMDscandata",&data,1);
		if(!laserSocket->sendMessage())
			return COMM_ERROR;
		if(!laserSocket->recvMessage(1))
			return COMM_ERROR;
		if(!laserSocket->compruebaRespuesta("sEA","LMDscandata",1))
			return INCORRECT_ANSWER;
	}		

	Automatic=state;
	return NO_ERROR;

}


/**
 * Metodo para obtener un scan del laser
 * @param[io] laseScanData Estructura donde se guardara la informacion obtenida
 * @param[in] timeout Tiempo maximo de espera de dicha informacion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** getScanData: Metodo publico para obtener principalmente las distancia a las que estan los obstaculos para los distintos ángulos
    
    Parametros de entrada: 
	-laserScanData: estructura del tipo laserScan (Ver laser.h )
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::getScanData(laserScan* laserScanData,int timeout)
{
	if(Automatic)
	{	
		
		if(!laserSocket->recvMessage(timeout))
			return COMM_ERROR;
		
		if(laserSocket->compruebaRespuesta("sSN","LMDscandata",-1))
			guardaDatos(laserScanData);
		else
			return INCORRECT_ANSWER;
		
	}
	else
	{	
		laserSocket->createMessage("sRN","LMDscandata",NULL,0);
		if(!laserSocket->sendMessage())
			return COMM_ERROR;
		if(!laserSocket->recvMessage(0.25))
			return COMM_ERROR;
		if(!laserSocket->compruebaRespuesta("sRA","LMDscandata",-1))
			return INCORRECT_ANSWER;
		guardaDatos(laserScanData);
	}
	return NO_ERROR;

}

/**
 * Metodo para obtener los parametros de configuracion de medida del laser
 * @param[io] laseParametersMeasure Estructura donde se guardara la informacion obtenida
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** getMeasuringParameters: Metodo publico para obtener los parametros basicos de configuracion del laser
    
    Parametros de entrada: 
	-laserParametersMeasure: estructura del tipo measuringParameters (Ver laser.h )
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::getMeasuringParameters(measuringParameters* laserParametersMeasure)
{
	laserSocket->createMessage("sRN","LMPscancfg",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sRA","LMPscancfg",5))
		return INCORRECT_ANSWER;

	laserParametersMeasure->scanningFrecuency=convertToInteger(laserSocket->data[0])/100;
	laserParametersMeasure->numberSegments=convertToInteger(laserSocket->data[1]);
	laserParametersMeasure->angleResolution=convertToInteger(laserSocket->data[2])/10000;
	laserParametersMeasure->startAngle=convertToInteger(laserSocket->data[3])/10000;
	laserParametersMeasure->stopAngle=convertToInteger(laserSocket->data[4])/10000;

	return NO_ERROR;

}


/**
 * Metodo para poner el nivel de acceso al laser
 * @param[in] userLevel Nivel de acceso
 * @param[in] password Contraseña para dicho nivel de acceso
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** selecUserLevel: Metodo privado para poner el nivel de uso del laser
    
    Parametros de entrada: 
	-userLevel: nivel de uso: mantenimiento,cliente...
	-password: contraseña para ese nivel
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

     Nota: En principio solo se utilizara el nivel cliente.
**/
int Sicklms1xx::selecUserLevel(char userLevel,int password)
{
	string data[2];
	data[0]=divideInTwoBytes(convertTostring(userLevel,true,false),1);
	data[1]=convertTostring(password,true,true);
	

	laserSocket->createMessage("sMN","SetAccessMode",data,2);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","SetAccessMode",1))
		return INCORRECT_ANSWER;	
	
	if(convertToInteger(laserSocket->data[0])==ERROR_USER_LEVEL)
		return SELECT_USER_LEVEL_NA;
	else
		return NO_ERROR;


}

/**
 * Metodo para configurar los parametros de medida del laser
 * @param[io] laseParametersMeasure Estructura donde se guardara la informacion obtenida
 * @param[in] clientPassword Contraseña para realizar esta operacion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** configureLaser: Metodo publico para configurar los parametros basicos del laser
    
    Parametros de entrada: 
	-laserParametersMeasure: estructura del tipo measuringParameters (Ver laser.h )
	- password: contraseña para mandar este comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
int Sicklms1xx::configureLaser(measuringParameters laserParametersMeasure,int clientPassword)
{
	int error;
	bool automaticAuxiliar=Automatic;


	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(false);
		if(error!=NO_ERROR)
			return error;
	}
	error=stopMeasurement(clientPassword);
	if(error!=NO_ERROR)
		return error;

	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=NO_ERROR)
		return error;

	string data[5];

	data[0]=convertTostring((int)laserParametersMeasure.scanningFrecuency*100,false,true);
	data[1]=convertTostring(laserParametersMeasure.numberSegments,false,true);
	data[2]=convertTostring((int)(laserParametersMeasure.angleResolution*10000),false,true);
	data[3]=convertTostring((int)laserParametersMeasure.startAngle*10000,false,true);
	data[4]=convertTostring((int)laserParametersMeasure.stopAngle*10000,false,true);


	laserSocket->createMessage("sMN","mLMPsetscancfg",data,5);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(20))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","mLMPsetscancfg",6))
		return INCORRECT_ANSWER;	
	
	int errorCode;
	errorCode=convertToInteger(laserSocket->data[0]);
        switch(errorCode)
        {
            case ERROR_FRECUENCY:
                error=INVALID_SAMPLE_FREQ;
                break;
            case ERROR_ANGULAR_RESOLUTION:
                error=INVALID_ANGLE_RESOLUTION;
                break;                        
            case ERROR_FRECUENCY_AND_RESOLUTION:
                error=INVALID_ANGLE_SAMPLE_RESOLUTION;
                break; 
            case ERROR_SCAN_AREA:
                error=INVALID_SCAN_AREA;
                break; 
            case ERROR_OTHER:
                error=UNKNOWN_ERROR;
                break; 
            default:
                error=NO_ERROR;
                break;
        }
        if(error!=NO_ERROR)
                return error;

	error=startMeasurement(clientPassword);
	if(error!=NO_ERROR)
		return error;

	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(true);
		if(error!=NO_ERROR)
			return error;
	}
	return NO_ERROR;
}

/**
 * Metodo para configurar las salidas del laser
 * @param[io] configOutput Estructura donde se guardara la informacion obtenida
 * @param[in] password Contraseña para realizar esta operacion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** configureDataOutput: Metodo publico para configurar el formato de los datos de salida de getScanData
    
    Parametros de entrada: 
	-configOutput: estructura del tipo dataOutput (Ver laser.h )
	- password: contraseña para mandar este comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
int Sicklms1xx::configureDataOutput(dataOutput configOutput,int clientPassword)
{
	int error;
	bool automaticAuxiliar=Automatic;
	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(false);
		if(error!=NO_ERROR)
			return error;
	}

	error=stopMeasurement(clientPassword);
	if(error!=NO_ERROR)
		return error;
	
	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=NO_ERROR)
		return error;

	string data[10];
	data[0]=divideInTwoBytes(convertTostring(configOutput.outputChannel,true,false),2);
	data[1]=convertTostring(configOutput.remission);
	data[2]=convertTostring(configOutput.Resolution,false,false);
	data[3]=convertTostring(configOutput.Unit,false,false);
	data[4]=divideInTwoBytes(convertTostring(configOutput.encoderData,true,false),2);
	data[5]=convertTostring(configOutput.positionFlag);
	data[6]=convertTostring(configOutput.deviceNameFlag);
	data[7]=convertTostring(configOutput.commentFlag);
	data[8]=convertTostring(configOutput.timeFlag);
	data[9]=convertTostring(configOutput.outputInterval,false,true);

	laserSocket->createMessage("sWN","LMDscandatacfg",data,10);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(20))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sWA","LMDscandatacfg",0))
		return INCORRECT_ANSWER;	

	error=startMeasurement(clientPassword);
	if(error!=NO_ERROR)
		return error;
	
	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(true);
		if(error!=NO_ERROR)
			return error;
	}
	return NO_ERROR;

}

/**
 * Metodo para obtener el nivel de contaminacion del laser
 * @param[io] contaminationLevel Variable donde se guardara la informacion obtenida
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** getContaminationLevel: Metodo publico para obtener el nivel de contaminacion del laser
    
    Parametros de entrada: 
	-contaminationLevel: variable donde se guarda la informacion requerida
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
int Sicklms1xx::getContaminationLevel(unsigned short* contaminationLevel)
{

	laserSocket->createMessage("sRN","LCMstate",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sRA","LCMstate",1))
		return INCORRECT_ANSWER;	
	*contaminationLevel=convertToInteger(laserSocket->data[0]);
	
	return NO_ERROR;

}

/**
 * Metodo para configurar las salidas del laser
 * @param[in] outputNumber Numero de salida del laser a configurar
 * @param[in] statusOutput Valor al que se pone dicha salida
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** setLmsOutput: Metodo publico para configurar el estado de las salidas digitales del laser
    
    Parametros de entrada: 
	-outputNumber: salida digital a configurar
	-statusOutput: valor de la salida digital (0 0 1)
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
int  Sicklms1xx::setLmsOutput(char outputNumber, bool statusOutput)
{

	string data[2];
	data[0]=convertTostring(outputNumber,false,false);
	data[1]=convertTostring(statusOutput);

	laserSocket->createMessage("sMN","mDOSetOutput",data,2);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","mDOSetOutput",1))
		return INCORRECT_ANSWER;	
	

	if(convertToInteger(laserSocket->data[0])== COMMAND_NO_ACEPTED)
		return SET_LMS_OUTPUT_NA;
	else
		return NO_ERROR;

}

/**
 * Metodo para configurar los leds del laser
 * @param[in] typeLED LED a configurar
 * @param[in] statusLED Valor a configurar en el LED
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** setLMSLEDs: Metodo publico para configurar el estado de los leds del laser
    
    Parametros de entrada: 
	-typeLED: LED a configurar
	-statusLED: estado del led (ON o OFF)
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
bool  Sicklms1xx::setLMSLEDs(char typeLED,bool statusLED)
{

	string data[2];
	data[0]=convertTostring(typeLED,false,false);
	data[1]=convertTostring(statusLED);

	laserSocket->createMessage("sMN","mLMLSetLed",data,2);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","mLMLSetLed",1))
		return INCORRECT_ANSWER;	
	

	if(convertToInteger(laserSocket->data[0])==COMMAND_NO_ACEPTED)
		return SET_LMS_LEDS_NA;
	else
		return NO_ERROR;

}

/**
 * Metodo para configurar el display del laser
 * @param[in] displayStatus Valor a configurar en el display 
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** setLMSLEDs: Metodo publico para configurar el estado de los leds del laser
    
    Parametros de entrada: 
	-displayStatus: Indica que segmentos del display se encienden.
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
bool  Sicklms1xx::setLMSDisplay(char displayStatus)
{

	string data;
	data=convertTostring(displayStatus,false,false);

	laserSocket->createMessage("sMN","mLMLSetDisp",&data,1);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","mLMLSetDisp",1))
		return INCORRECT_ANSWER;	
	

	if(convertToInteger(laserSocket->data[0])==COMMAND_NO_ACEPTED)
		return SET_LMS_DYSPLAY_NA;
	else
		return NO_ERROR;

}

/**
 * Metodo para salvar permanentemente la configuracion actual
 * @param[in] clientPassword contraseña para poder realizar la operacion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** saveDataPermanently: Metodo publico para salvar la configuracion en la EEPROM del laser
    
    Parametros de entrada: 
	- password: contraseña para mandar este comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 

**/
int Sicklms1xx::saveDataPermanently(int clientPassword)
{
	
	int error;
	bool automaticAuxiliar=Automatic;
	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(false);
		if(error!=NO_ERROR)
			return error;
	}

	error=stopMeasurement(PASSWORD);
	if(error!=NO_ERROR)
		return error;

	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=0)
		return error;

	laserSocket->createMessage("sMN","mEEwriteall",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(20))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","mEEwriteall",1))
		return INCORRECT_ANSWER;	


	if(convertToInteger(laserSocket->data[0])== COMMAND_NO_ACEPTED)
		return SAVE_DATA_NA;
	else
		return NO_ERROR;

	error=startMeasurement(PASSWORD);
	if(error!=NO_ERROR)
		return error;

	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(true);
		if(error!=NO_ERROR)
			return error;
	}
	return NO_ERROR;

}

/**
 * Metodo para iniciar el dispositivo
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** startDevice: Metodo privado que le comunica al laser que empieze a medir (Es llamado dentro de startMeasurement)
    
    Parametros de entrada: Ninguno
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/
int Sicklms1xx::startDevice()
{
	
	laserSocket->createMessage("sMN","Run",NULL,0);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(1))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sAN","Run",1))
		return INCORRECT_ANSWER;	


	if(convertToInteger(laserSocket->data[0])== ERROR_START_DEVICE)
		return START_DEVICE_NA;
	else
		return NO_ERROR;

}

/**
 * Metodo para configurar el tipo de filtro del laser
 * @param[in] typeFilter Tipo de filtro a configurar
 * @param[in] clientPassword Contraseña para que se pueda realizar la operacion
 * @return Entero que indica si la operacion fue realizada correctamente
 */
/** frEchoFilter: Metodo publico para comfigurar el tipo de filtro
    
    Parametros de entrada: 
	-typeFilter: tipo de filtro
	-clientPassword: contraseña para enviar este comando
    Parametro de salida: int que indica si ha habido error. Si el valor es distinto de cero indica que tipo de error ha ocurrido
			 si es 0 es que no ha habido error 
**/

int  Sicklms1xx::frEchoFilter(int typeFilter,int clientPassword)
{
	int error;
	bool automaticAuxiliar=Automatic;
	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(false);
		if(error!=NO_ERROR)
			return error;
	}

	error=stopMeasurement(PASSWORD);
	if(error!=NO_ERROR)
		return error;

	error=selecUserLevel(LEVEL_AUTHORISED_CLIENT,clientPassword);
	if(error!=0)
		return error;

	string data;
	data=convertTostring(typeFilter,false,false);
	laserSocket->createMessage("sWN","FREchoFilter",&data,1);
	if(!laserSocket->sendMessage())
		return COMM_ERROR;
	if(!laserSocket->recvMessage(20))
		return COMM_ERROR;
	if(!laserSocket->compruebaRespuesta("sMN","FREchoFilter",1))
		return INCORRECT_ANSWER;	

	error=startMeasurement(PASSWORD);
	if(error!=NO_ERROR)
		return error;

	
	if(automaticAuxiliar)
	{
		error=setScanDataContinuous(true);
		if(error!=NO_ERROR)
			return error;
	}

	return NO_ERROR;


}

/**
 * Metodo para guardar la informacion del laser despues de un scan
 * @param[io] laserScanData Estructura donde se guarda la informacion
 */

/** guardaDatos: Metodo privado que guarda todos los parametros de un scan del laser
	
	Parametros de entrada:
		-laserScanData: estructura de tipo laserScan (vease laser.h)
	Parametros de salida: ninguno
**/
void Sicklms1xx::guardaDatos(laserScan* laserScanData)
{
	

	int indice=0;
	laserScanData->laserDevice.versionNumberMeasure=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserDevice.deviceNumber=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserDevice.serialNumber=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserDevice.deviceStatus=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserDevice.deviceStatus=convertToUnsignedInteger(laserSocket->data[indice++]);

	laserScanData->laserStatus.messageCounter=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserStatus.scanCounter=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserStatus.powerUpDuration=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserStatus.transmisionDuration=convertToUnsignedInteger(laserSocket->data[indice++]);
	laserScanData->laserStatus.inputStatus=convertToUnsignedInteger(laserSocket->data[indice++]);
	indice++; //valor 00h
	laserScanData->laserStatus.outputStatus=convertToUnsignedInteger(laserSocket->data[indice++]);
	indice++; //valor 00h
	indice++; //Un byte reservado

	laserScanData->scanningFrecuency=convertToUnsignedInteger(laserSocket->data[indice++])/100;
	laserScanData->measurementFrequency=convertToUnsignedInteger(laserSocket->data[indice++]);
	
	laserScanData->laserEncoders.numberOfEncoders=convertToUnsignedInteger(laserSocket->data[indice++]);
	for(int i=0;i< laserScanData->laserEncoders.numberOfEncoders;i++)
	{
	
		laserScanData->laserEncoders.encoderPosition[i]=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserEncoders.encoderSpeed[i]=convertToUnsignedInteger(laserSocket->data[indice++]);
	}
		
	laserScanData->numberChannels16Bit=convertToUnsignedInteger(laserSocket->data[indice++]);
	for(int i=0;i< laserScanData->numberChannels16Bit;i++)
	{
		laserScanData->laserChannel16[i].measuredDataContent=laserSocket->data[indice++];
		laserScanData->laserChannel16[i].scalingFactor=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserChannel16[i].scalingOffset=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserChannel16[i].startingAngle=convertToInteger(laserSocket->data[indice++])/10000.0;
		laserScanData->laserChannel16[i].angularStepWidth=convertToUnsignedInteger(laserSocket->data[indice++])/10000.0;
		laserScanData->laserChannel16[i].numberData=convertToUnsignedInteger(laserSocket->data[indice++]);

		laserScanData->laserChannel16[i].data.clear();
		for(unsigned int j=0;j<laserScanData->laserChannel16[i].numberData;j++)
			laserScanData->laserChannel16[i].data.push_back(convertToUnsignedInteger(laserSocket->data[indice++]));
	}


	laserScanData->numberChannels8Bit=convertToUnsignedInteger(laserSocket->data[indice++]);
	for(int i=0;i< laserScanData->numberChannels8Bit;i++)
	{
		laserScanData->laserChannel8[i].measuredDataContent=laserSocket->data[indice++];
		laserScanData->laserChannel8[i].scalingFactor=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserChannel8[i].scalingOffset=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserChannel8[i].startingAngle=convertToInteger(laserSocket->data[indice++])/10000.0;
		laserScanData->laserChannel8[i].angularStepWidth=convertToUnsignedInteger(laserSocket->data[indice++])/10000.0;
		laserScanData->laserChannel8[i].numberData=convertToUnsignedInteger(laserSocket->data[indice++]);

		laserScanData->laserChannel8[i].data.clear();
		for(unsigned int j=0;j<laserScanData->laserChannel8[i].numberData;j++)
			laserScanData->laserChannel8[i].data.push_back(convertToUnsignedInteger(laserSocket->data[indice++]));
	}

	if(convertToUnsignedInteger(laserSocket->data[indice++])==1)
	{
		laserScanData->laserPosition.xPosition=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.yPosition=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.zPosition=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.xRotation=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.yRotation=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.zRotation=convertToReal(laserSocket->data[indice++]);
		laserScanData->laserPosition.rotationType=convertToUnsignedInteger(laserSocket->data[indice++]);
	}
	
	if(convertToUnsignedInteger(laserSocket->data[indice++])==1)
		laserScanData->laserOthers.deviceName=laserSocket->data[indice++];

	if(convertToUnsignedInteger(laserSocket->data[indice++])==1)
		laserScanData->laserOthers.commentContent=laserSocket->data[indice++];
	
	if(convertToUnsignedInteger(laserSocket->data[indice++])==1)
	{
		
		laserScanData->laserTime.year=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.month=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.day=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.hour=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.minute=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.second=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserTime.microSecond=convertToUnsignedInteger(laserSocket->data[indice++]);
	}

	if(convertToUnsignedInteger(laserSocket->data[indice++])==1)
	{
		laserScanData->laserEvent.eventType=laserSocket->data[indice++];
		laserScanData->laserEvent.encoderPosition=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserEvent.eventTime=convertToUnsignedInteger(laserSocket->data[indice++]);
		laserScanData->laserEvent.angularPosition=convertToInteger(laserSocket->data[indice++]);
		
	}	
}
