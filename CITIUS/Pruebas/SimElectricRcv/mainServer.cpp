/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 26 de junio de 2014, 10:05
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

#define PORT 20000 /* El puerto que será abierto */
#define BACKLOG 1 /* El número de conexiones permitidas */

#define MAXDATASIZE 100   

typedef struct {
    short battery_level;
    short battery_voltage;
    short battery_current;
    short battery_temperature;
    short supply_alarms;
}ElectricInfo;

/*******************************************************************************
    *               PROTOTIPOS DE FUNCIONES SECUNDARIAS
*******************************************************************************/

void requestDispatcher(FrameDriving frame, int socketDescriptor, int *currentMsgCount, ElectricInfo *electricInfo);

bool isCriticalInstruction(short element);

short getDeviceValue(short element, ElectricInfo electricInfo);

void setDeviceValue(ElectricInfo *electricInfo, short element, short value);

int main(int argc, char *argv[]) {
    
    ElectricInfo electricInfo;
    electricInfo.battery_level = 0;
    electricInfo.battery_voltage = 0;
    electricInfo.battery_current = 0;
    electricInfo.battery_temperature = 0;
    electricInfo.supply_alarms = 0x00;
    
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
    printf("Esperando conexion de RosNode_Electric\n");
    
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
        if ((recv(fd2, bufData, MAXDATASIZE, 0)) <= 0) {
            /* llamada a recv() */
            printf("No se recibe na \n");
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
                printf("SET: %d = %d\n",fdr.element, fdr.value);
                if(isCriticalInstruction(fdr.element)){
                    printf("Cuanta sim: %d - Cuenta mensaje: %d\n",currentMsgCount,fdr.id_instruction);
                }
            }else if(fdr.instruction == GET){
                printf("GET: %d\n",fdr.element);
            }else{
                printf("comando desconocido\n");
            }
            
            // Gestiona las peticiones
            requestDispatcher(fdr, fd2, &currentMsgCount, &electricInfo);
            
        }
    }
}

void requestDispatcher(FrameDriving frame, int socketDescriptor, int *currentMsgCount, ElectricInfo *electricInfo){
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
        setDeviceValue(electricInfo, frame.element, frame.value);
    }else if(frame.instruction == GET) {
        // Despacha peticion y devuelve INFO  (sin ID_INSTRUCCION)
        FrameDriving ret;
        ret.instruction = INFO;
        ret.id_instruction = 0;
        ret.element = frame.element;
        ret.value = getDeviceValue(frame.element,*electricInfo);
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
    if(element == SUPPLY_TURN_ON
            || element == TURN_OFF){
        return true;
    }else{
        return false;
    }

}

short getDeviceValue(short element, ElectricInfo electricInfo) {
    switch (element) {
        case BATTERY_LEVEL:
            return electricInfo.battery_level;
            break;
        case BATTERY_VOLTAGE:
            return electricInfo.battery_voltage;
            break;
        case BATTERY_CURRENT:
            return electricInfo.battery_current;
            break;
        case BATTERY_TEMPERATURE:
            return electricInfo.battery_temperature;
            break;
        case SUPPLY_ALARMS:
            return electricInfo.supply_alarms;
            break;
        default:
            break;
    }
}

void setDeviceValue(ElectricInfo *electricInfo, short element, short value){
    switch (element) {
        case BATTERY_LEVEL:
            (*electricInfo).battery_level = value;
            break;
        case BATTERY_VOLTAGE:
            (*electricInfo).battery_voltage = value;
            break;
        case BATTERY_CURRENT:
            (*electricInfo).battery_current = (bool) value;
            break;
        case BATTERY_TEMPERATURE:
            (*electricInfo).battery_temperature = value;
            break;
        case SUPPLY_ALARMS:
            (*electricInfo).supply_alarms = value;
            break;
        default:
            break;
    }
}