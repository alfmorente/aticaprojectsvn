/**
 * @file   Files.h
 * @brief  Fichero de cabecera para gestion de ficheros
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
 */


#ifndef FILES_H
#define	FILES_H

#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>

#define NO_ERROR -1

using namespace std;
//Estructura de configuracion



/**
 * \class Files
 * \brief Clase que gestiona los ficheros
 */
class Files
{
        public:
            //Fichero LOG
            //static ofstream fileError;
            //static ofstream fileData;
            ofstream fileError;
            ofstream fileData;
            ifstream fileConfig;

        public:
            Files();
            int openFiles();
            //static void  writeErrorInLOG(int error, string typeResponse);
            //static void  writeDataInLOG(string info);
            void  writeErrorInLOG(int error, string typeResponse);
            void  writeDataInLOG(string info);
            bool setValueConfiguration(string parametro,string valor,ComConfig* config);
            int  readConfig(ComConfig* config);
            int getTimeout(int outputInterval,int scanningFrecuency);            
    
};

#endif	/* FILES_H */

/**
 *@}
 */