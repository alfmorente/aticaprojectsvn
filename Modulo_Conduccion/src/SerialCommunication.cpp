#include "../include/Modulo_Conduccion/SerialCommunication.hpp"

#include <iostream>
//------------------
//-- CONSTANTS
//------------------

//--TIMEOUT in micro-sec (It is set to 1 sec in this example)
#define TIMEOUT 400000

using namespace std;

SerialCommunication::SerialCommunication(char* serial_name, int baud){

    this->serial_name = serial_name;
    this->baud = baud;
    
    this->serialFlagActive = true;

}

SerialCommunication::~SerialCommunication() {
    serial_close();
}

/******************************************************************************/
/* Open the serial port                                                       */
/*----------------------------------------------------------------------------*/
/* INPUT:                                                                     */
/*   -serial_name: Serial device name                                         */
/*   -baud: Serial speed. The constants Bxxxx are used, were xxxx  is the     */
/*          speed. They are defined in termios.h. For example: B9600, B19200..*/
/*          For more information type "man termios"                           */
/*                                                                            */
/* RETURNS:                                                                   */
/*   -The Serial device descriptor  (-1 if there is an error)                 */
/******************************************************************************/
int SerialCommunication::serial_open(char *serial_name, speed_t baud)
{
  struct termios newtermios;
  int fd;

  // Open the serial port
  fd = open(serial_name,O_RDWR | O_NOCTTY);

  // Configure the serial port attributes:
  //   -- No parity
  //   -- 8 data bits
  //   -- other things...
  newtermios.c_cflag= CBAUD | CS8 | CLOCAL | CREAD;
  newtermios.c_iflag=IGNPAR;
  newtermios.c_oflag=0;
  newtermios.c_lflag=0;
  newtermios.c_cc[VMIN]=1;
  newtermios.c_cc[VTIME]=0;

  // Set the speed
  cfsetospeed(&newtermios,baud);
  cfsetispeed(&newtermios,baud);

  // flush the input buffer
  if (tcflush(fd,TCIFLUSH)==-1) {
    return -1;
  }

  // flush the output buffer
  if (tcflush(fd,TCOFLUSH)==-1) {
    return -1;
  }

  //-- Configure the serial port now!!
  if (tcsetattr(fd,TCSANOW,&newtermios)==-1) {
    return -1;
  }

  //-- Return the file descriptor
  return fd;
}

/*****************************************************/
/* Sending a string to the serial port.              */
/*                                                   */
/* INPUT:                                            */
/*   -serial_fd: Serial device descriptor            */
/*   -data:      data string to send                 */
/*   -size:      data string size                    */
/*****************************************************/
void SerialCommunication::serial_send(char *data, int size)
{
  write(serial_fd, data, size);
  //cout << "String sent------> " << data << "\n";
}

/*************************************************************************/
/* Receiving a string from the serial port                               */
/*-----------------------------------------------------------------------*/
/* INPUT                                                                 */
/*   -serial_fd: Serial device descriptor                                */
/*   -size: Maximum data size to receive                                 */
/*   -timeout_usec: Timeout time (in micro-secs) for receiving the data  */
/*                                                                       */
/* OUTPUT:                                                               */
/*   -data: The serial data received within the timeout_usec time        */
/*                                                                       */
/* RETURNS:                                                              */
/*   -The number of bytes received.                                      */
/*   -0 if none received. It means a TIMEOUT!                            */
/*************************************************************************/
int SerialCommunication::serial_read(char *data, int size, int timeout_usec)
{
  fd_set fds;
  struct timeval timeout;
  int count=0;
  int ret;
  int n;

  //-- Wait for the data. A block of size bytes is expected to arrive
  //-- within the timeout_usec time. This block can be received as
  //-- smaller blocks.
  do {
      //-- Set the fds variable to wait for the serial descriptor
      FD_ZERO(&fds);
      FD_SET (serial_fd, &fds);

      //-- Set the timeout in usec.
      timeout.tv_sec = 0;
      timeout.tv_usec = timeout_usec;

      //-- Wait for the data
      ret=select (FD_SETSIZE,&fds, NULL, NULL,&timeout);

    //-- If there are data waiting: read it
      if (ret==1) {

        //-- Read the data (n bytes)
        n=read (serial_fd, &data[count], size-count);

        //-- The number of bytes receives is increased in n
        count+=n;

        //-- The last byte is always a 0 (for printing the string data)
        data[count]=0;
      }

    //-- Repeat the loop until a data block of size bytes is received or
    //-- a timeout occurs
  } while (count<size && ret==1);

  //-- Return the number of bytes reads. 0 If a timeout has occurred.
  return count;
}


/********************************************************************/
/* Close the serial port                                            */
/*------------------------------------------------------------------*/
/* INPUT: :                                                         */
/*   fd: Serial device descriptor                                   */
/********************************************************************/
void SerialCommunication::serial_close()
{
  close(serial_fd);
}


void SerialCommunication:: abrir_cerrar (int adam, int relay, bool tipo)
{
	stringstream cadena;
	
	cadena<<"#0";
	cadena<<adam;
	cadena<<"1";

	switch(relay)
	{
		case 0:
			cadena<<"0";
			break;
		case 1:
			cadena<<"1";
			break;
		case 2:
			cadena<<"2";
			break;
		case 3:
			cadena<<"3";
			break;
		case 4:
			cadena<<"4";
			break;
		case 5:
			cadena<<"5";
			break;
		case 6:
			cadena<<"6";
			break;
		case 7:
			cadena<<"7";
			break;
	}

	if (tipo == true)	// ABRIR RELAY
		cadena<<"01\r";
	else 			// CERRAR RELAY
		cadena<<"00\r";

	//limpiamos el puerto serie
	tcflush(serial_fd, TCIFLUSH);

	//write(canal,(char*)cadena.str().c_str(), cadena.str().length());
        serial_send((char*)cadena.str().c_str(), cadena.str().length());

        usleep(20);
        
        // Recepción del caracter de retorno
        escuchar_puerto(data, 1); //Escuchará bloques de tamaño 1 byte
	usleep(50000);
}

        




void SerialCommunication::escuchar_puerto(char *data, int size){

       //-- Wait for the received data
      int n;
      n=serial_read(data,size,TIMEOUT);

      //-- Show the received data
      //cout << "String received--> ";
      fflush(stdout);

      if (n>0) {
        cout << data << "(" << n << " bytes)\n";
      }
      else {
        cout << "Timeout!\n";
      }

}

// Ejecutado al inicio. Debe dejar todos los relés abiertos en su estado inicial
void SerialCommunication::iniciar(){
        abrir_cerrar(1, 0, false);
        abrir_cerrar(1, 1, false);
	abrir_cerrar(1, 2, false);
	abrir_cerrar(1, 3, false);
	abrir_cerrar(2, 0, false);
	abrir_cerrar(2, 1, false);
	abrir_cerrar(2, 2, false);
	abrir_cerrar(2, 3, false);
	abrir_cerrar(2, 5, false);
	abrir_cerrar(2, 6, false);
	abrir_cerrar(2, 7, false);
        usleep(20000);

}


bool SerialCommunication::EstablishCommunication(){
  //-- Open the serial port
  //-- The speed is configure at 38400 baud in this case
  bool res;

  serial_fd = serial_open(serial_name,baud);

  //-- Error checking
  if (serial_fd == -1) {
    //cout << "Error opening the serial device: "<< serial_name << " \n";
    res = false;
  }
  else{
    //cout << "Conexión Serie "<< serial_name << " establecida correctamente \n";
    res = true;
  }

  return res;

}


int SerialCommunication::nivelBomba () {

//	ROS_INFO("LLEGA");
	int resultado = 0;
	stringstream cadena;
	char buf[255];
	char cad[255];	
	int byte_leidos = 0;
	float valor;
	float valor_max = 12.22;
	float valor_min = 3.967;
        
	cadena<<"#0";
	cadena<<"3"; //Número de la Adam
	cadena<<"0\r";

	//limpiamos el puerto serie
	tcflush(serial_fd, TCIFLUSH);


	write(serial_fd,(char*)cadena.str().c_str(), cadena.str().length());
        usleep(100000);
             
	byte_leidos = read(serial_fd, buf, 10);
	if (byte_leidos > 0) {
		buf[byte_leidos]='\0';
 
		int j = 0;
		for (int i = 2; i<strlen(buf); i++) {
			cad[j] = buf[i];
			j++;
		}			
		cad[j]='\0';
//		printf ("resultadovvv = %s", buf);
//		printf ("resultado = %s", cad);
		valor = atof(cad);
		resultado = (int) ((valor-valor_min)*103/(valor_max-valor_min));
		if (resultado >100 || resultado < 0)
			resultado = 0;				
		printf ("valor = %f", valor);
//		printf ("resultado = %d", resultado);

	}

	return resultado;
}


void SerialCommunication::DoWork() {
    
   
    //SerialManager(5,0); // OpenAllRelays() - Inicialización de ADAM (necesaria en caso de salida no ordenada)
    
    //SerialManager(1,0); // power_ignition(ON) - Alimentación de la ECU del motor

    while (serialFlagActive){
        
        usleep(20000);
       
    }
    
    //SerialManager(5,0); // OpenAllRelays()

}