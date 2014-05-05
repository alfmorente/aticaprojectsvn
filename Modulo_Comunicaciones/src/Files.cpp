#include <Modulo_Comunicaciones/Files.h>
#include "../../Common_files/include/Common_files/constant.h"

Files::Files()
{
    
}

int Files::openFiles()
{
    fileError.open("fileError.txt");
    if(fileError.bad())
      return COMM_LOG_FILE_ERROR;

    fileData.open("fileData.txt");
    if(fileData.bad())
    {
      writeErrorInLOG(COMM_LOG_FILE_ERROR,"Fichero de datos");
      return COMM_LOG_FILE_ERROR;
    }
    fileConfig.open("configLaser.txt");
    if(!fileConfig.is_open())	
    {
       writeErrorInLOG(COMM_CONFIG_FILE_ERROR,"Fichero de configuracion");
       return COMM_CONFIG_FILE_ERROR;
    }
    return NO_ERROR;
    
}

int Files::readConfig(ComConfig* config)
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


void Files::writeErrorInLOG(int error, string typeResponse)
{
	struct timeval timeLog;

	gettimeofday(&timeLog,NULL);
     	fileError << timeLog.tv_sec%18000 << "." << timeLog.tv_usec << ":\t";
	switch(error)
	{
           
		case JAUS_CONFIG_ERROR:
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
			fileError << typeResponse << ": El fichero no se pudo abrir";
		default:
			fileError << "Unknown Error.Terminated comunication with Laser";
	}
        fileError << endl;
}

void Files::writeDataInLOG(string info)
{
    	struct timeval timeLog;
	gettimeofday(&timeLog,NULL);
        fileData<< timeLog.tv_sec%18000 << "." << timeLog.tv_usec << ":\t";
        fileData <<info<< endl;
}

bool Files::setValueConfiguration(string parametro,string valor,ComConfig* config)
{

	stringstream auxiliar;
	auxiliar << valor;
        
        //Parámetros a configurar en este módulo
        /**
	if(parametro=="ipLaser")
		auxiliar >> configConection->ipLaser;
	else 
		return false;**/	
	return true;
}