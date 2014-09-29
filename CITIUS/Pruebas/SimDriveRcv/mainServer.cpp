/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 19 de junio de 2014, 10:05
 */

#include <stdio.h>          
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "constant.h"

#define PORT 10000 /* El puerto que será abierto */
#define BACKLOG 1 /* El número de conexiones permitidas */

#define MAXDATASIZE 100   

typedef struct {
    short steering;
    short thottle;
    short brake;
    bool parkingBrake;
    unsigned short gear;
    unsigned short speed;
    short motorRPM;
    short motorTemperature;
    bool lights;
    bool blinkerLeft;
    bool blinkerRight;
    bool dipss;
    bool dipsr;
    bool dipsp;
    bool klaxon;
}DrivingInfo;

/*******************************************************************************
    *               PROTOTIPOS DE FUNCIONES SECUNDARIAS
*******************************************************************************/

void requestDispatcher(FrameDriving frame, int socketDescriptor, int *currentMsgCount, DrivingInfo *vehicleInfo);

bool isCriticalInstruction(short element);

short getDeviceValue(short element, DrivingInfo vehicleInfo);

void setDeviceValue(DrivingInfo *vehicleInfo, short element, short value);


int main(int argc, char *argv[]) {
    
    DrivingInfo vehicleInfo;
    vehicleInfo.lights = false;
    vehicleInfo.blinkerLeft = false;
    vehicleInfo.blinkerRight = false;
    vehicleInfo.dipsp = false;
    vehicleInfo.dipsr = false;
    vehicleInfo.dipss = false;
    vehicleInfo.klaxon = false;
    vehicleInfo.brake = 0;
    vehicleInfo.thottle = 0;
    vehicleInfo.steering = 0;
    vehicleInfo.parkingBrake = false;
    vehicleInfo.gear = 0;
    vehicleInfo.speed = 0;
    vehicleInfo.motorTemperature = 0;
    vehicleInfo.motorRPM = 0;

    int fd, fd2, currentMsgCount = 1; /* los ficheros descriptores */

    struct sockaddr_in server;
    /* para la información de la dirección del servidor */

    struct sockaddr_in client;
    /* para la información de la dirección del cliente */

    socklen_t sin_size;

    /* A continuación la llamada a socket() */
    if ((fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        printf("error en socket()\n");
        exit(-1);
    }

    server.sin_family = AF_INET;

    server.sin_port = htons(PORT);
    /* ¿Recuerdas a htons() de la sección "Conversiones"? =) */

    server.sin_addr.s_addr = INADDR_ANY;
    /* INADDR_ANY coloca nuestra dirección IP automáticamente */

    bzero(&(server.sin_zero), 8);
    /* escribimos ceros en el reto de la estructura */

    char buf[MAXDATASIZE];
    /* en donde es almacenará el texto recibido */

    /* A continuación la llamada a bind() */
    if (bind(fd, (struct sockaddr*) &server, sizeof (struct sockaddr)) == -1) {
        printf("Error en bind() \n");
        exit(-1);
    }

    if (listen(fd, BACKLOG) == -1) { /* llamada a listen() */
        printf("Error en listen()\n");
        exit(-1);
    }
    sin_size = sizeof (struct sockaddr_in);
    printf("Esperando conexion de RosNode_Driving\n");
    /* A continuación la llamada a accept() */
    if ((fd2 = accept(fd, (struct sockaddr *) &client, &sin_size)) == -1) {
        printf("Error en accept()\n");
        exit(-1);
    }

    printf("Se obtuvo una conexión desde %s\n", inet_ntoa(client.sin_addr));
    /* que mostrará la IP del cliente */


    while (1) {
        
        // Buffer de envio
        char bufData[8];
        if ((recv(fd2, bufData, MAXDATASIZE, 0)) == -1) {
            /* llamada a recv() */
            printf("Error en recv() \n");
            exit(-1);
        } else {
            // Estructura de recepcion
            FrameDriving fdr;
            // Rellenado del buffer
            memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
            memcpy(&fdr.id_instruction, &bufData[2], sizeof (fdr.id_instruction));
            memcpy(&fdr.element, &bufData[4], sizeof (fdr.element));
            memcpy(&fdr.value, &bufData[6], sizeof (fdr.value));
            if(fdr.instruction == SET){
                printf("SET: %d = %d\n", fdr.element, fdr.value);
                if(isCriticalInstruction(fdr.element)){
                    printf("Cuanta sim: %d - Cuenta mensaje: %d\n",currentMsgCount,fdr.id_instruction);
                }
            }else if(fdr.instruction == GET){
                printf("GET: %d\n", fdr.element);
            }else{
                printf("Comando desconocido\n");
            }
            
            // Gestiona las peticiones
            requestDispatcher(fdr, fd2, &currentMsgCount, &vehicleInfo);
            
        }
    }
}

void requestDispatcher(FrameDriving frame, int socketDescriptor, int *currentMsgCount, DrivingInfo *vehicleInfo){
    if(frame.instruction == SET){
        if(isCriticalInstruction(frame.element)){
            // Despacha peticion y devuelve ACK (con ID_INSTRUCCION)
            if(frame.id_instruction == *currentMsgCount){ // Orden OK --> ACK
                FrameDriving ret;
                ret.instruction = ACK;
                ret.id_instruction = *currentMsgCount;
                ret.element = frame.element;
                ret.value = frame.value;
                //Buffer de envio 
                char buff[8];
                memcpy(&buff[0], &ret.instruction, sizeof (ret.instruction));
                memcpy(&buff[2], &ret.id_instruction, sizeof (ret.id_instruction));
                memcpy(&buff[4], &ret.element, sizeof (ret.element));
                memcpy(&buff[6], &ret.value, sizeof (ret.value));
                send(socketDescriptor, buff, sizeof (buff), 0);
                usleep(100); 
                (*currentMsgCount)= (*currentMsgCount)+1;
            }else{                                      // Orden ERROR --> NACK
                FrameDriving ret;
                ret.instruction = NACK;
                ret.id_instruction = *currentMsgCount;
                ret.element = frame.element;
                ret.value = frame.value;
                //Buffer de envio 
                char buff[8];
                memcpy(&buff[0], &ret.instruction, sizeof (ret.instruction));
                memcpy(&buff[2], &ret.id_instruction, sizeof (ret.id_instruction));
                memcpy(&buff[4], &ret.element, sizeof (ret.element));
                memcpy(&buff[6], &ret.value, sizeof (ret.value));
                send(socketDescriptor, buff, sizeof (buff), 0);
                usleep(100); 
            }
        } else {
            // Despacha peticion y devuelve ACK (sin ID_INSTRUCCION)
            FrameDriving ret;
            ret.instruction = ACK;
            ret.id_instruction = 0;
            ret.element = frame.element;
            ret.value = frame.value;
            //Buffer de envio 
            char buff[8];
            memcpy(&buff[0], &ret.instruction, sizeof (ret.instruction));
            memcpy(&buff[2], &ret.id_instruction, sizeof (ret.id_instruction));
            memcpy(&buff[4], &ret.element, sizeof (ret.element));
            memcpy(&buff[6], &ret.value, sizeof (ret.value));
            send(socketDescriptor, buff, sizeof (buff), 0);
            usleep(100); 
        }
        setDeviceValue(vehicleInfo, frame.element, frame.value);
    }else if(frame.instruction == GET) {
        // Despacha peticion y devuelve INFO  (sin ID_INSTRUCCION)
        FrameDriving ret;
        ret.instruction = INFO;
        ret.id_instruction = 0;
        ret.element = frame.element;
        ret.value = getDeviceValue(frame.element,*vehicleInfo);
        //Buffer de envio 
        char buff[8];
        memcpy(&buff[0], &ret.instruction, sizeof (ret.instruction));
        memcpy(&buff[2], &ret.id_instruction, sizeof (ret.id_instruction));
        memcpy(&buff[4], &ret.element, sizeof (ret.element));
        memcpy(&buff[6], &ret.value, sizeof (ret.value));
        send(socketDescriptor, buff, sizeof (buff), 0);
        usleep(100); 
    }
    *currentMsgCount++;
}

bool isCriticalInstruction(short element){
    if(element == RESET
            || element == GEAR
            || element == MT_GEAR
            || element == THROTTLE
            || element == MT_THROTTLE
            || element == CRUISING_SPEED
            || element == HANDBRAKE
            || element == MT_HANDBRAKE
            || element == BRAKE
            || element == MT_BRAKE
            || element == STEERING
            || element == MT_STEERING){
        return true;
    }else{
        return false;
    }

}

short getDeviceValue(short element, DrivingInfo vehicleInfo) {
    switch (element) {
        case THROTTLE:
            return vehicleInfo.thottle;
            break;
        case BRAKE:
            return vehicleInfo.brake;
            break;
        case HANDBRAKE:
            return vehicleInfo.parkingBrake;
            break;
        case STEERING:
            return vehicleInfo.steering;
            break;
        case GEAR:
            return vehicleInfo.gear;
            break;
        case MOTOR_RPM:
            return vehicleInfo.motorRPM;
            break;
        case CRUISING_SPEED:
            return vehicleInfo.speed;
            break;
        case MOTOR_TEMPERATURE:
            return vehicleInfo.motorTemperature;
            break;
        case BLINKER_LEFT:
            return vehicleInfo.blinkerLeft;
            break;
        case BLINKER_RIGHT:
            return vehicleInfo.blinkerRight;
            break;
        case KLAXON:
            return vehicleInfo.klaxon;
            break;
        case DIPSP:
            return vehicleInfo.dipsp;
            break;
        case DIPSS:
            return vehicleInfo.dipss;
            break;
        case DIPSR:
            return vehicleInfo.dipsr;
            break;
        default:
            break;
    }
}

void setDeviceValue(DrivingInfo *vehicleInfo, short element, short value){
    switch (element) {
        case THROTTLE:
            (*vehicleInfo).thottle = value;
            break;
        case BRAKE:
            (*vehicleInfo).brake = value;
            break;
        case HANDBRAKE:
            (*vehicleInfo).parkingBrake = (bool) value;
            break;
        case STEERING:
            (*vehicleInfo).steering = value;
            break;
        case GEAR:
            (*vehicleInfo).gear = value;
            break;
        case MOTOR_RPM:
            (*vehicleInfo).motorRPM = value;
            break;
        case CRUISING_SPEED:
            (*vehicleInfo).speed = value;
            break;
        case MOTOR_TEMPERATURE:
            (*vehicleInfo).motorTemperature = value;
            break;
        case BLINKER_LEFT:
            (*vehicleInfo).blinkerLeft = (bool) value;
            break;
        case BLINKER_RIGHT:
            (*vehicleInfo).blinkerRight = (bool) value;
            break;
        case KLAXON:
            (*vehicleInfo).klaxon = (bool) value;
            break;
        case DIPSP:
            (*vehicleInfo).dipsp = (bool) value;
            break;
        case DIPSS:
            (*vehicleInfo).dipss = (bool) value;
            break;
        case DIPSR:
            (*vehicleInfo).dipsr = (bool) value;
            break;
        default:
            break;
    }
}