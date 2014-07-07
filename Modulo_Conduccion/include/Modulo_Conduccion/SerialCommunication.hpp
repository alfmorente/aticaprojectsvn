/* 
 * File:   SerialCommunication.hpp
 * Author: Sergio
 *
 * Created on Julio 04, 2014
 */

#ifndef _SERIALCOMMUNICATION_HPP
#define	_SERIALCOMMUNICATION_HPP

#include "Thread.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <cstdlib>


//-- ASCII string to send through the serial port
#define wordSize    1
#define ON          1
#define OFF         0

using namespace std;

class SerialCommunication: public Thread{

public:
    SerialCommunication(char* serial_name, int baud);
    virtual ~SerialCommunication();

    int  serial_open(char *serial_name, speed_t baud);
    void serial_send(char *data, int size);
    int  serial_read(char *data, int size, int timeout_usec);
    void serial_close();

    // Funciones de Alto Nivel
    bool EstablishCommunication();
    
    void abrir_cerrar (int adam, int relay, bool tipo);

    void escuchar_puerto(char *data, int size);
    void iniciar();
    
    int nivelBomba ();
    virtual void DoWork();
    
    bool serialFlagActive;        /* Usada para indicar si debe seguirse ejecutando */

private:
       
    string state;    
    int serial_fd;           //-- Serial port descriptor
    int baud;                //-- Serial port Baud rate
    char * serial_name;      //-- Serial port name
    char data[wordSize];     //-- The received command

};

#endif	/* _SERIALCOMMUNICATION_HPP */