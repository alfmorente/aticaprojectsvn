/* 
 * File:   ConnectionManager.h
 * Author: atica
 *
 * Created on 27 de mayo de 2014, 12:15
 */

#ifndef CONNECTIONMANAGER_H
#define	CONNECTIONMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* CONNECTIONMANAGER_H */

/*******************************************************************************
 *                              LIBRERIAS
*******************************************************************************/
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <string>

/*******************************************************************************
 *              CONSTANTES IDENTIFICADORES DE DISPOSITIVOS
*******************************************************************************/

#define ID_BLINKER_RIGHT 0
#define ID_BLINKER_LEFT 1
#define ID_BLINKER_EMERGENCY 2
#define ID_DIPSP 3
#define ID_DIPSS 4
#define ID_DIPSR 5
#define ID_KLAXON 6
#define ID_GEAR 7
#define ID_THROTTLE 8
#define ID_MOTOR_RPM 9
#define ID_CRUISING_SPEED 10 
#define ID_MOTOR_TEMPERATURE 11
#define ID_HANDBRAKE 12
#define ID_BRAKE 13
#define ID_STEERING 14
#define ID_ALARMS 15

/*******************************************************************************
 *              CONSTANTES CONMUTACION MANUAL/TELEMANDADO
*******************************************************************************/

#define MT_BLINKERS 16
#define MT_LIGHTS 17
#define MT_THROTTLE 18
#define MT_BRAKE 19
#define MT_HANDBRAKE 20
#define MT_STEERING 21
#define MT_GEAR 22

/*******************************************************************************
 *              MAXIMOS/MINIMOS PARA EL AJUSTE DE VALORES
*******************************************************************************/

#define MAX_BOOLEAN 1
#define MIN_BOOLEAN 0
#define MAX_GEAR 2
#define MIN_GEAR 0
#define MAX_PERCENT 100
#define MIN_PERCENT 0
#define MAX_STEERING 100
#define MIN_STEERING -100
#define MAX_RPM_SPD 1000
#define MIN_RPM_SPD 0
#define MAX_TEMPERATURE 400
#define MIN_TEMPERATURE 0



/*******************************************************************************
 *              CLASE MANEJADOR DE CONEXION CON DRIVING
*******************************************************************************/

class DrivingConnectionManager{
    public:
        DrivingConnectionManager(char *serial_name);
        // Manejadores de la transmisión
        bool setSpeed(int speed);
        bool disconnect();
        bool send(char * command);
        char *recieve();
        // Manejadores de comandos
        char *createCommand(char *typeOfCommand, short device, int value);
        void extractCommand(char *command);
        // Funciones auxiliares
        char* obtainDeviceName(short deviceID);
        int adjustValue(short device, int value);
        
    private:
        struct termios newtio;
        struct termios oldtio;
        int channel;
};