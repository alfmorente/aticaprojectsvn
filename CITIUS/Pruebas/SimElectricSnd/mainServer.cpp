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

#include "constant.h"
#include "MessageDispatcher.h"

/*******************************************************************************
    *               PROTOTIPOS DE FUNCIONES SECUNDARIAS
*******************************************************************************/

int printfMenu();
void requestDispatcher(int socketDispatcher, int request);

int main(int argc, char *argv[]) {
    
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
    
    bool exit = false;
    
    int newSt = 0;

  while (!exit) {

    newSt = printfMenu();
    if (newSt == 33) { // Salir
      exit = true;
    } else { // Opcion valida
      requestDispatcher(fd2, newSt);
    }
  }
}

int printfMenu() {
  
  cout << "****************************************" << endl;
  cout << "1 ) Enviar mensaje BATTERY LEVEL" << endl;
  cout << "2 ) Enviar mensaje BATTERY VOLTAGE" << endl;
  cout << "3 ) Enviar mensaje BATTERY CURRENT" << endl;
  cout << "4 ) Enviar mensaje BATTERY TEMPERATURE" << endl;
  cout << "5 ) Enviar mensaje SUPPLY CHECK" << endl;
  cout << "6 ) Enviar mensaje TURN OFF" << endl;
  cout << "7 ) Enviar mensaje SUPPLY 5" << endl;
  cout << "8 ) Enviar mensaje SUPPLY 12" << endl;
  cout << "9 ) Enviar mensaje SUPPLY 24 DRIVE" << endl;
  cout << "10) Enviar mensaje SUPPLY 24 OCC" << endl;
  cout << "11) Enviar mensaje CONTROL SYSTEM SUPPLY" << endl;
  cout << "12) Enviar mensaje CONTROL SYSTEM SUPPLY 5" << endl;
  cout << "13) Enviar mensaje CONTROL SYSTEM SUPPLY 12" << endl;
  cout << "14) Enviar mensaje CONTROL SYSTEM SUPPLY 24" << endl;
  cout << "15) Enviar mensaje CONTROL SYSTEM SUPPLY 48" << endl;
  cout << "16) Enviar mensaje DRIVE SYSTEM SUPPLY" << endl;
  cout << "17) Enviar mensaje DRIVE SYSTEM SUPPLY 5" << endl;
  cout << "18) Enviar mensaje DRIVE SYSTEM SUPPLY 12" << endl;
  cout << "19) Enviar mensaje DRIVE SYSTEM SUPPLY 24" << endl;
  cout << "20) Enviar mensaje DRIVE SYSTEM SUPPLY 48" << endl;
  cout << "21) Enviar mensaje COMM SYSTEM SUPPLY" << endl;
  cout << "22) Enviar mensaje COMM SYSTEM SUPPLY 5" << endl;
  cout << "23) Enviar mensaje COMM SYSTEM SUPPLY 12" << endl;
  cout << "24) Enviar mensaje COMM SYSTEM SUPPLY 24" << endl;
  cout << "25) Enviar mensaje COMM SYSTEM SUPPLY 48" << endl;
  cout << "26) Enviar mensaje OBSERVATION SYSTEM SUPPLY" << endl;
  cout << "27) Enviar mensaje OBSERVATION SYSTEM SUPPLY 5" << endl;
  cout << "28) Enviar mensaje OBSERVATION SYSTEM SUPPLY 12" << endl;
  cout << "29) Enviar mensaje OBSERVATION SYSTEM SUPPLY 24" << endl;
  cout << "30) Enviar mensaje OBSERVATION SYSTEM SUPPLY 48" << endl;
  cout << "31) Enviar mensaje SUPPLY ALARMS" << endl;
  cout << "32) Enviar mensaje OPERATION MODE SWITCH" << endl;
  cout << "33) Salir" << endl;
  cout << "****************************************" << endl;
  cout << " Selecciona una opcion:" << endl;

  int intAux;
  cin >> intAux;

  if (intAux < 1 || intAux > 33) {
    cout << "Opcion no valida" << endl;
    return 0;
  } else {
    return intAux;
  }

}

void requestDispatcher(int socketDescriptor, int request) {
  MessageDispatcher *disp = new MessageDispatcher();
  switch (request) {
    case 1: // bettery level
      disp->sendBatteryLevelMsg(socketDescriptor);
      break;
    case 2: // battery voltage 
      disp->sendBatteryVoltageMsg(socketDescriptor);
      break;
    case 3: // battery current 
      disp->sendBatteryCurrentMsg(socketDescriptor);
      break;
    case 4: // battery temperature
      disp->sendBatteryTemperatureMsg(socketDescriptor);
      break;
    case 5: // supply check
      disp->sendSupplyCheckMsg(socketDescriptor);
      break;
    case 6: // turn off
      disp->sendTurnOffMsg(socketDescriptor);
      break;
    case 7: // supply 5
      disp->sendSupply5Msg(socketDescriptor);
      break;
    case 8: // supply 12
      disp->sendSupply12Msg(socketDescriptor);
      break;
    case 9: // supply 24 drive
      disp->sendSupply24DriveMsg(socketDescriptor);
      break;
    case 10: // supply 24 occ
      disp->sendSupply24OCCMsg(socketDescriptor);
      break;
    case 11: // control system supply
      disp->sendControlSystemSupplyMsg(socketDescriptor);
      break;
    case 12: // control system supply 5
      disp->sendControlSystemSupply5Msg(socketDescriptor);
      break;
    case 13: // control system supply 12
      disp->sendControlSystemSupply12Msg(socketDescriptor);
      break;
    case 14: // control system supply 24
      disp->sendControlSystemSupply24Msg(socketDescriptor);
      break;
    case 15: // control system supply 48
      disp->sendControlSystemSupply48Msg(socketDescriptor);
      break;
    case 16: // drive system supply 
      disp->sendDriveSystemSupplyMsg(socketDescriptor);
      break;
    case 17: // drive system supply 5
      disp->sendDriveSystemSupply5Msg(socketDescriptor);
      break;
    case 18: // drive system supply 12
      disp->sendDriveSystemSupply12Msg(socketDescriptor);
      break;
    case 19: // drive system supply 24
      disp->sendDriveSystemSupply24Msg(socketDescriptor);
      break;
    case 20: // drive system supply 48
      disp->sendDriveSystemSupply48Msg(socketDescriptor);
      break;
    case 21: // comm system supply 
      disp->sendCommSystemSupplyMsg(socketDescriptor);
      break;
    case 22: // comm system supply 5
      disp->sendCommSystemSupply5Msg(socketDescriptor);
      break;
    case 23: // comm system supply 12
      disp->sendCommSystemSupply12Msg(socketDescriptor);
      break;
    case 24: // comm system supply 24
      disp->sendCommSystemSupply24Msg(socketDescriptor);
      break;
    case 25: // comm system supply 48
      disp->sendCommSystemSupply48Msg(socketDescriptor);
      break;
    case 26: // observation system supply 
      disp->sendObservationSystemSupplyMsg(socketDescriptor);
      break;
    case 27: // observation system supply 5
      disp->sendObservationSystemSupply5Msg(socketDescriptor);
      break;
    case 28: // observation system supply 12
      disp->sendObservationSystemSupply12Msg(socketDescriptor);
      break;
    case 29: // observation system supply 24
      disp->sendObservationSystemSupply24Msg(socketDescriptor);
      break;
    case 30: // observation system supply 48
      disp->sendObservationSystemSupply48Msg(socketDescriptor);
      break;
    case 31: // supply alarms
      disp->sendSupplyAlarmsMsg(socketDescriptor);
      break;
    case 32: // operation mode switch
      disp->sendOperationModeSwitchMsg(socketDescriptor);
      break;
    default: // nothing to do
      break;


  }
}