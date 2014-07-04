#include "Modulo_GPS/PortSerial.h"

using namespace std;

// Constructor de la clase

PortSerial::PortSerial()
{

}

// Apertura del puerto serie

bool PortSerial::openSerial(char* nombre){
  this->descriptorSerie = open(nombre, O_RDWR | O_NOCTTY | O_NDELAY);
  if (descriptorSerie == -1){
    return false;
  }else{
    return true;
  }
}

// Cierre del puerto serie

void PortSerial::closeSerial(){
	close(descriptorSerie);
}

// Limpieza del puerto serie

void PortSerial::clean(){
	tcflush(descriptorSerie, TCIFLUSH);
}

// Configuracion del puerto serie

void PortSerial::configura(int speed){
       fcntl(descriptorSerie, F_SETFL, 0);

       tcgetattr(descriptorSerie,&oldtio);
       newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;

       newtio.c_iflag = IGNPAR ;
       newtio.c_oflag = 0;
       newtio.c_lflag = 0;


       newtio.c_cc[VTIME]= 0;
       newtio.c_cc[VMIN]= 1;

       tcflush(descriptorSerie, TCIFLUSH);
       tcsetattr(descriptorSerie,TCSANOW,&newtio);
}

// Envio por puerto serie

bool PortSerial::send(char* data, int tamano)
{
	return write(descriptorSerie,data,tamano);
}

// Recepcion por puerto serie

int PortSerial::recv(char* data,int tamano, int timeout)
{

	int res;

	struct timeval Timeout;
	fd_set readfs;
	FD_ZERO(&readfs);
	FD_SET(descriptorSerie, &readfs);

	Timeout.tv_usec = 0;  /* milisegundos */
	Timeout.tv_sec  = timeout;  /* segundos */


	if(select(descriptorSerie+1, &readfs, NULL, NULL, &Timeout)>0)
	{
		res = read(descriptorSerie,data,tamano);
		if(res<=0)
			return SERIAL_BAD;
	}
	else
		return SERIAL_TIMEOUT;

	return SERIAL_OK;

}



