/**
 * @file   Files.cpp
 * @brief  Fichero fuente de gestion de ficheros
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#include <Modulo_Laser2D_Rear/Files.h>

/**
 * Constructor de la clase
 */
Files::Files()
{
    
}

/**
 * Metodo que abre los ficheros de datos,configuracion y posibles errores
 * @return Entero que indica si la apertura de ficheros fue correcta  
 */
int Files::openFiles()
{
    fileError.open("fileError.txt");
    if(fileError.bad())
      return LASER_LOG_FILE_ERROR;

    fileData.open("fileData.txt");
    if(fileData.bad())
    {
      writeErrorInLOG(LASER_LOG_FILE_ERROR,"Fichero de datos");
      return LASER_LOG_FILE_ERROR;
    }
    fileConfig.open("configLaser.txt");
    if(!fileConfig.is_open())	
    {
       writeErrorInLOG(LASER_LOG_FILE_ERROR,"Fichero de configuracion");
       return LASER_CONFIG_FILE_ERROR;
    }
    return NO_ERROR;
    
}

/**
 * Actualiza la estructura de configuracion con los datos del fichero
 * @param[in] parametro Nombre del parametro 
 * @param[in] valor  Valor del parametro
 * @param[io] config  Estructura para guardar la configuracion de medidas del laser
 * @param[io] configOutput  Estructura para guardar la configuracion de salida del laser
 * @param[io] configConection  Estructura para guardar la configuracion de conexion con el laser
 * @return Booleano que indica si el parametro a configurar existe
 */
bool Files::setValueConfiguration(string parametro,string valor,measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection)
{

	stringstream auxiliar;
	auxiliar << valor;
	if(parametro=="ipLaser")
		auxiliar >> configConection->ipLaser;
	else if(parametro=="portLaser")
		auxiliar >> configConection->portLaser;
	else if(parametro=="scanningFrecuency")
		auxiliar >> config->scanningFrecuency;
	else if(parametro=="angleResolution")
		auxiliar >> config->angleResolution;
	else if(parametro=="startAngle")
		auxiliar >> config->startAngle;
	else if(parametro=="stopAngle")
		auxiliar >> config->stopAngle;
	else if(parametro=="numberSegments")
		auxiliar >> config->numberSegments;
	else if(parametro=="encoderData")
		auxiliar >> configOutput->encoderData;
	else if(parametro=="positionFlag")
		auxiliar >> configOutput->positionFlag;
	else if(parametro=="deviceNameFlag")
		auxiliar >> configOutput->deviceNameFlag;
	else if(parametro=="commentFlag")
		auxiliar >> configOutput->commentFlag;
	else if(parametro=="timeFlag")
		auxiliar >> configOutput->timeFlag;
	else if(parametro=="outputInterval")
		auxiliar >> configOutput->outputInterval;
	else 
		return false;
	
	return true;
}

/**
 * Metodo que lee el fichero de configuracion
 * @param[io] config  Estructura para guardar la configuracion de medidas del laser
 * @param[io] configOutput  Estructura para guardar la configuracion de salida del laser
 * @param[io] configConection  Estructura para guardar la configuracion de conexion con el laser
 * @return Entero que indica si la lectura del fichero de configuracion fue correcta
 */
int Files::readConfig(measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection)
{
	string cadena;
	int pos;
	string var,valor;
	
        //Voy leyendo cada parametro del fichero
        while (!fileConfig.eof())
        { 
                getline(fileConfig,cadena);
                if(cadena[0] != '#' && !isspace(cadena[0]) && cadena[0] != NULL)
                {

                        pos = cadena.find(":");
                        if(pos!=-1)
                        {
                                valor = cadena.substr (pos+1);
                                var = cadena.substr(0,pos);
                                if(!setValueConfiguration(var,valor,config,configOutput,configConection))
                                        return LASER_CONFIG_FILE_STRUCTURE_ERROR;
                        }
                        else
                                return LASER_CONFIG_FILE_STRUCTURE_ERROR;
                }
        }
        fileConfig.close();
        config->numberSegments=1;
        configOutput->outputChannel=1; //reserved
        configOutput->remission=true; //reserved 
        configOutput->encoderData=0;
        configOutput->Resolution=sixteenBits;
        configOutput->Unit=0;
        return NO_ERROR;
}

/**
 * Escribe en el fichero LOG el error producido en el modulo
 * @param[in] error Error a escribir en el fichero LOG
 * @param[in] typeResponse Cadena que define el error
 */
void Files::writeErrorInLOG(int error, string typeResponse)
{
	struct timeval timeLog;

	gettimeofday(&timeLog,NULL);
     	fileError << timeLog.tv_sec%18000 << "." << timeLog.tv_usec << ":\t";
	switch(error)
	{
           
		case LASER_SOCKET_FAIL:
			fileError << typeResponse << ": Socket failed";
			break;
		case CONNECTION_ERROR:
			fileError << typeResponse << ": Connection failed with Laser";
			break;
		case COMM_ERROR:
			fileError << "Failed Comunicaction with Laser sicklms5xx.Terminated comunication with Laser";
			break;
		case INVALID_SAMPLE_FREQ:
			fileError << typeResponse << ": Invalid frecuency.Terminated comunication with Laser";
			break;
		case INVALID_ANGLE_RESOLUTION:
			fileError << typeResponse << ": Invalid angular resolution.Terminated comunication with Laser";
			break;
		case INVALID_ANGLE_SAMPLE_RESOLUTION:
			fileError << typeResponse << ": Invalid frecuency and angular resolution.Terminated comunication with Laser";
			break;
		case INVALID_SCAN_AREA:
			fileError << typeResponse << ": Invalid scan Area.Terminated comunication with Laser";
			break;
		case UNKNOWN_ERROR:
			fileError << typeResponse << ": Other error.Terminated comunication with Laser";
			break;
		case INCORRECT_ANSWER:
			fileError << typeResponse << ": Response Error";
			break;
		case START_MEASURE_NA:
			fileError << typeResponse << ": Command Start Measure not accepted.Terminated comunication with Laser";
			break;
		case STOP_MEASURE_NA:
			fileError << typeResponse << ": Command Stop Measure not accepted.Terminated comunication with Laser";
			break;
		case SELECT_USER_LEVEL_NA:
			fileError << typeResponse << ": Command Select User Level not accepted.Terminated comunication with Laser";
			break;
		case SET_LMS_OUTPUT_NA:
			fileError << typeResponse << ": Command Set LMS Output not accepted.Terminated comunication with Laser";
			break;
		case SAVE_DATA_NA:
			fileError << typeResponse << ": Command Save Data Permanently not accepted.Terminated comunication with Laser";
			break;
		case START_DEVICE_NA:
			fileError << typeResponse << ": Command Start Device not accepted.Terminated comunication with Laser";
			break;
		case SET_LMS_LEDS_NA:
			fileError << typeResponse << ": Command Set LMS LEDs not accepted.Terminated comunication with Laser";
			break;
		case SET_LMS_DYSPLAY_NA:
			fileError << typeResponse << ": Command Set LMS Display not accepted.Terminated comunication with Laser";
			break;
		case FRAME_OVERFLOW:
			fileError << typeResponse << ": Max error response permited.Terminated comunication with Laser";
			break;
		case LASER_CONFIG_FILE_ERROR:
			fileError << typeResponse << ": El fichero no existe";
			break;
		case LASER_CONFIG_FILE_STRUCTURE_ERROR:
			fileError << typeResponse << ": El fichero esta mal estructurado";
			break;                        
		case LASER_LOG_FILE_ERROR:
			fileError << typeResponse << ": El fichero no se pudo abrir";
		default:
			fileError << "Unknown Error.Terminated comunication with Laser";
	}
        fileError << endl;
}

/**
 * Escribe en el LOG informacion obtenida por el modulo
 * @param[in] info Cadena que define la informacion a escribir
 */
void Files::writeDataInLOG(string info)
{
    	struct timeval timeLog;
	gettimeofday(&timeLog,NULL);
        fileData<< timeLog.tv_sec%18000 << "." << timeLog.tv_usec << ":\t";
        fileData <<info<< endl;
}

