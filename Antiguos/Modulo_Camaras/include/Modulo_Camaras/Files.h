/* 
 * File:   Files.h
 * Author: atica
 *
 * Created on 16 de abril de 2014, 13:06
 */

#ifndef FILES_H
#define	FILES_H

#include <termios.h>
#include <sys/time.h>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>

#define NO_ERROR -1

using namespace std;
typedef struct dataConfig
{
    char portName[20];
    unsigned int portVelocity;
    unsigned int idCamera;
    int velPAN;
    int velTILT;
   
}CameraConfig;

class Files
{
        public:
            //Fichero LOG
            static ofstream fileError;
            static ofstream fileData;
            ifstream fileConfig;

        public:
            Files();
            int openFiles();
            static void  writeErrorInLOG(int error, string typeResponse);
            static void  writeDataInLOG(string info);
            bool setValueConfiguration(string parametro,string valor,CameraConfig* config);
            int  readConfig(CameraConfig* config);
            int getTimeout(int outputInterval,int scanningFrecuency);            
    
};

#endif	/* FILES_H */

