/**
 * @file   Files.cpp
 * @brief  Fichero fuente de gestion de ficheros
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#include <Modulo_Camaras/Files.h>
#include "../../Common_files/include/Common_files/constant.h"
#include <iostream>

//ofstream Files::fileError;
//ofstream Files::fileData;

/**
 * Constructor de la clase
 */
Files::Files()
{
    
}

/**
 * Metodo que abre los ficheros de datos,configuracion y posibles errores
 * @return Entero que indica si la apertura de ficheros fue correcta
 *  
 */
int Files::openFiles()
{
    fileError.open("/home/atica/catkin_ws/src/Modulo_Camaras/bin/fileError.txt");
    if(fileError.bad())
      return ERROR_CAMERA_CONFIGURATION;

    fileData.open("/home/atica/catkin_ws/src/Modulo_Camaras/bin/fileData.txt");
    if(fileData.bad())
    {
      //writeErrorInLOG(COMM_LOG_FILE_ERROR,"Fichero de datos");
      return ERROR_CAMERA_CONFIGURATION;
    }
    fileConfig.open("/home/atica/catkin_ws/src/Modulo_Camaras/bin/configCamera.txt");
    if(!fileConfig.is_open())	
    {
       //writeErrorInLOG(COMM_CONFIG_FILE_ERROR,"Fichero de configuracion");
       return ERROR_CAMERA_CONFIGURATION;
    }
    return NO_ERROR;
    
}

/**
 * Metodo que lee el fichero de configuracion
 * @param[io] config  Estructura para guardar la configuracion de la camara
 * @return entero que indica si la lectura del fichero de configuracion fue correcta
 */
int Files::readConfig(CameraConfig* config)
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
                                if(!setValueConfiguration(var,valor,config))
                                        return COMM_CONFIG_FILE_STRUCTURE_ERROR;
                        }
                        else
                                return COMM_CONFIG_FILE_STRUCTURE_ERROR;
                }
        }
        fileConfig.close();
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
           
		/**case JAUS_CONFIG_ERROR:
			fileError << typeResponse << ": Error de configuracion";
			break;
		case CREATE_COMPONENT_ERROR:
			fileError << typeResponse << ": Error al crear el componente";
			break;
		case RUN_COMPONENT_ERROR:
			fileError << typeResponse << "Error al ejecutar el componente";
			break;
		case COMMUNICATION_UCR_FAIL:
			fileError << typeResponse << "Error de comunicacion con la UCR";
			break;
                case COMM_CONFIG_FILE_ERROR:
			fileError << typeResponse << ": El fichero no existe";
			break;
		case COMM_CONFIG_FILE_STRUCTURE_ERROR:
			fileError << typeResponse << ": El fichero esta mal estructurado";
			break;                        
		case COMM_LOG_FILE_ERROR:
			fileError << typeResponse << ": El fichero no se pudo abrir";**/
		default:
			fileError << "Unknown Error.Terminated comunication with camera";
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

/**
 * Actualiza la estructura de configuracion con los datos del fichero
 * @param[in] parametro Nombre del parametro 
 * @param[in] valor  Valor del parametro
 * @param[io] config Estructura para guardar la configuracion de la camara
 * @return Booleano que indica si el parametro a configurar existe
 */
bool Files::setValueConfiguration(string parametro,string valor,CameraConfig* config)
{

	stringstream auxiliar;
	auxiliar << valor;
        
        //Parámetros a configurar en este módulo
        
	if(parametro=="port_name")
		auxiliar >> config->portName;
	else if(parametro=="port_velocity")
		auxiliar >> config->portVelocity; 
	else if(parametro=="id_camera")
		auxiliar >> config->idCamera;  
	else if(parametro=="vel_pan")
		auxiliar >> config->velPAN;  
	else if(parametro=="vel_tilt")
		auxiliar >> config->velTILT;          
	else 
		return false;	
	return true;
}
