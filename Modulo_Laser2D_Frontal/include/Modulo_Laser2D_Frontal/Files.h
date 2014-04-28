/* 
 * File:   config_error.h
 * Author: atica
 *
 * Created on 16 de abril de 2014, 13:06
 */

#ifndef FILES_H
#define	FILES_H

#include <Modulo_Laser2D_Frontal/SickLMS5xx.h>
#include <termios.h>
#include <sys/time.h>
#include <math.h>


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

