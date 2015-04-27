/**
 * @file   Files.h
 * @brief  Fichero de cabecera para gestion de ficheros
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#ifndef FILES_H
#define	FILES_H

#include <Modulo_Laser2D_Rear/SickLMS1xx.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>

/**
 * \class Files
 * \brief Fichero de cabecera de gestion de ficheros
 */

class Files
{
        public:
            //Fichero LOG
            ofstream fileError;
            ofstream fileData;
            ifstream fileConfig;

        public:
            Files();
            int openFiles();
            void writeErrorInLOG(int error, string typeResponse);
            void writeDataInLOG(string info);
            bool setValueConfiguration(string parametro,string valor,measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection);
            int  readConfig(measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection);
            int getTimeout(int outputInterval,int scanningFrecuency);            
    
};




/**
bool setValueConfiguration(string parametro,string valor,measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection);
int leeConfiguracion(measuringParameters* config,dataOutput* configOutput,conectionParameters* configConection);
void stringError(int error, string typeResponse);
int filesOperation(measuringParameters*,dataOutput*,conectionParameters*);
int getTimeout(int outputInterval,int scanningFrecuency);**/
#endif	/* CONFIG_ERROR_H */

