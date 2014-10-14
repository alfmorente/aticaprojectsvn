/* 
 * File:   GPS_Management.h
 * Author: carlosamores
 *
 * Created on 21 de marzo de 2013, 10:03
 */


#include <string>
#include "constant_gps.h"
#include "../../Common_files/include/Common_files/constant.h"
#include <iostream>


#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <cstdlib>	

#include <sys/time.h>

//libreria para isdigit()
#include <ctype.h>

//Librerias para el puerto serie
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

class GPS_Management
{
	private:
                
                char* nombre_puerto_serie;
                int descriptorSerie;
                struct timeval timeLog;

		ofstream flog;
		struct termios oldtio,newtio;
                // Gestion del puerto serie
                
                
                bool port_opened;
                // Estructuras de los mensajes a recibir
                
	public:
            
                bool error;
            
                float roll;
                float pitch;
                float yaw;
                float latitude;
                float longitude;
                float altitude;
                short imuH;
                int gpsFix;
                int sat;         
           
                // Constructor
		GPS_Management();
                
                bool configurePuertoSerie();
                
                void closePuertoSerie();

                // Recepcion continua de datos
                string rcvData();
                
                void getParamsCarly(string);
                

};

