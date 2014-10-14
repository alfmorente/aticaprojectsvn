#include <string.h>
#include <vector>

#include "Modulo_GPS_IMU/GPS_Management.h"

using namespace std;

// Constructor de la clase
GPS_Management::GPS_Management(){
    

    roll = 0;
    pitch = 0;
    yaw = 0;
    latitude = 0;
    longitude = 0;
    altitude = 0;
    imuH = 0;
    gpsFix = 0;
    sat = 0;       

    nombre_puerto_serie = "/dev/ttyS0";
  
    
}


// configuraPuertoSerie: Configura el puerto serie a 19200 baudios, 8 bits, bits de paridad 1, paridad inpar
bool GPS_Management::configurePuertoSerie()
{	
 char * serial_name = (char *) "/dev/ttyS0";

  descriptorSerie = open(serial_name, O_RDWR | O_NOCTTY | O_NDELAY);

  if (descriptorSerie < 0) {

    return false;

  } else {

    tcgetattr(descriptorSerie, &oldtio);
    bzero(&newtio, sizeof (newtio));
    newtio.c_cflag = B9600 | CRTSCTS | CS8 | CLOCAL | CREAD;
    newtio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    newtio.c_oflag = 0;
    newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    newtio.c_cc[VINTR] = 0; /* Ctrl-c */
    newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
    newtio.c_cc[VERASE] = 0; /* del */
    newtio.c_cc[VKILL] = 0; /* @ */
    newtio.c_cc[VEOF] = 4; /* Ctrl-d */
    newtio.c_cc[VTIME] = 0; /* temporizador entre caracter, no usado */
    newtio.c_cc[VMIN] = 1; /* bloqu.lectura hasta llegada de caracter. 1 */
    newtio.c_cc[VSWTC] = 0; /* '\0' */
    newtio.c_cc[VSTART] = 0; /* Ctrl-q */
    newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
    newtio.c_cc[VSUSP] = 0; /* Ctrl-z */
    newtio.c_cc[VEOL] = 0; /* '\0' */
    newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
    newtio.c_cc[VWERASE] = 0; /* Ctrl-w */
    newtio.c_cc[VLNEXT] = 0; /* Ctrl-v */
    newtio.c_cc[VEOL2] = 0; /* '\0' */

    tcflush(descriptorSerie, TCIFLUSH);
    tcsetattr(descriptorSerie, TCSANOW, &newtio);

    usleep(100);

  }

  return true;

}

// closePuertoSerie: Cierra el puerto serie.
void GPS_Management::closePuertoSerie()
{
	close(descriptorSerie);
}

/** recvCommand: Recibe un comando por puerto serie

    Parametros de entrada: void
    Parametro de salida: string con la respuesta recibida **/

string GPS_Management::rcvData() {
  stringstream mensaje;
  unsigned char byte;
  bool dataFound = false;
  
  int countAst = 0;

    while (!dataFound) {

        // HEADER (PRE + BID + MID)
        if (read(descriptorSerie, &byte, 1) > 0) {
            
            
            if (byte == '*') {
                countAst++;
            }else{
                countAst = 0;
            }
            
            if(countAst == 3)
                dataFound = true;
            
            mensaje << byte;
        }
        
    }
  
  
  return mensaje.str();
  
}

void GPS_Management::getParamsCarly(string frame) {
    
    //LATITUDE
    int index = frame.find("LAT:");
    string value;
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    latitude = (float) atof(value.c_str());
    
    //LONGUITUDE
    index = frame.find("LON:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    longitude = (float) atof(value.c_str());
    
    //ALTITUDE
    index = frame.find("ALT:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    
    altitude = (float) atof(value.c_str());

    //ROLL
    index = frame.find("RLL:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    roll = (float) atof(value.c_str());
    
    //PCH
    index = frame.find("PCH:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    pitch = (float) atof(value.c_str());
    
    //YAW
    index = frame.find("YAW:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    yaw = (float) atof(value.c_str());
    
   
    //FIX
    index = frame.find("FIX:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    gpsFix = (int) atoi(value.c_str());
    
    //NUMERO SAT
    index = frame.find("SAT:");
    value.clear();
    for (unsigned int i = index + 4; i < frame.size(); i++) {
        if (frame.at(i) == ',') {
            i = frame.size();
        } else {
            value.push_back(frame.at(i));
        }
    }
    sat = (int) atoi(value.c_str());
    
}
