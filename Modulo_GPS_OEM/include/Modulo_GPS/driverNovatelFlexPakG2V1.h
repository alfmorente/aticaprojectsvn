#ifndef DRIVERNOVATELFLEXPAKG2V1
#define DRIVERNOVATELFLEXPAKG2V1

/***********************************************************************************/
/* driverNovatelFlexPakG2V1.cpp  Félix M. Ballester Goytia    Junio, 2012          */
/*---------------------------------------------------------------------------------*/
/* Comunicacion serie con GPS Novatel FlexPak-G2-V1 - Linux Ubuntu                 */
/*---------------------------------------------------------------------------------*/
/* Envio y tratamiento del comando GPGGARTK. Formato de datos NMEA.                */
/* Envio y tratamiento del comando GPVTG. Formato de datos NMEA.    	           */
/***********************************************************************************/
#define BAUDRATE B9600
#define _POSIX_SOURCE 1 

#define GPGGA "GPGGA"
#define GPVTG "GPVTG"

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <stdexcept>

// Mensajes
#include "Common_files/msg_gps.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_stream.h"
#include "Common_files/msg_backup.h"

// Librerias 
#include "Modulo_GPS/TeachThread.hpp"
// Señales externas
//#include "GPS_OEM6_Driver/external_signals.h"
#include "../../Common_files/include/Common_files/constant.h"
// Interaccion con usuario
#include "Modulo_GPS/interaction.h"

// ROS
#include "ros/ros.h"

// Suscriptores
void fcn_sub_enableModule(const Common_files::msg_module_enable);
void fcn_sub_backup(const Common_files::msg_backup);
void initModuleVariables();



using namespace std;

class driverNovatelFlexPakG2V1
{
	public:
		driverNovatelFlexPakG2V1();
		driverNovatelFlexPakG2V1(int *, char *);
		~driverNovatelFlexPakG2V1();
		int setFrequency(int );
		//char* datos_GPGGARTK (int *,int *,int *,int *,float *,char *,float *,char *,int *,int *,float *,float *,char *,char checksum[2]);
		char *getFrame ();
                char *getHeader(char*);
		int canal;
                bool GPGGAdata(int *,int *,int *,int *,float *,char *,float *,char *,int *,int *,float *,float *,char *,char[2]);
                bool GPVTGdata(float *,char *,float *,char *,float *,char *,float *,char *,char[2]);
                void apagarGPS();
        private:
		
		struct termios newtio,oldtio;
		bool checksumXOR(char*);
};

#endif

