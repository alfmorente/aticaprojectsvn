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
    if (newSt == 25) { // Salir
      exit = true;
    } else { // Opcion valida
      requestDispatcher(fd2, newSt);
    }
  }
}

int printfMenu() {

  cout << "****************************************" << endl;
  cout << "1 ) Enviar mensaje THROTTLE" << endl;
  cout << "2 ) Enviar mensaje BRAKE" << endl;
  cout << "3 ) Enviar mensaje HANDBRAKE" << endl;
  cout << "4 ) Enviar mensaje STEERING" << endl;
  cout << "5 ) Enviar mensaje GEAR" << endl;
  cout << "6 ) Enviar mensaje STEERING ALARMS" << endl;
  cout << "7 ) Enviar mensaje DRIVE ALARMS" << endl;
  cout << "8 ) Enviar mensaje BLINKER LEFT" << endl;
  cout << "9 ) Enviar mensaje BLINKER RIGHT" << endl;
  cout << "10) Enviar mensaje EMERGENCY BLINKER" << endl;
  cout << "11) Enviar mensaje DIPSS" << endl;
  cout << "12) Enviar mensaje DIPSR" << endl;
  cout << "13) Enviar mensaje DIPSP" << endl;
  cout << "14) Enviar mensaje KLAXON" << endl;
  cout << "15) Enviar mensaje MT THROTTLE" << endl;
  cout << "16) Enviar mensaje MT BRAKE" << endl;
  cout << "17) Enviar mensaje MT HANDBRAKE" << endl;
  cout << "18) Enviar mensaje MT STEERING" << endl;
  cout << "19) Enviar mensaje MT GEAR" << endl;
  cout << "20) Enviar mensaje MT BLINKER" << endl;
  cout << "21) Enviar mensaje MT LIGHTS" << endl;
  cout << "22) Enviar mensaje MOTOR RPM" << endl;
  cout << "23) Enviar mensaje MOTOR TEMPERATURE" << endl;
  cout << "24) Enviar mensaje CRUISSING SPEED" << endl;
  cout << "25) Salir" << endl;
  cout << "****************************************" << endl;
  cout << " Selecciona una opcion:" << endl;

  int intAux;
  cin >> intAux;

  if (intAux < 1 || intAux > 25) {
    cout << "Opcion no valida" << endl;
    return 0;
  } else {
    return intAux;
  }

}

void requestDispatcher(int socketDescriptor, int request) {
  MessageDispatcher *disp = new MessageDispatcher();
  switch (request) {
    case 1: // throttle
      disp->sendThrottleInfo(socketDescriptor);
      break;
    case 2: // brake
      disp->sendBrakeInfo(socketDescriptor);
      break;
    case 3: // handbrake
      disp->sendHandbrakeInfo(socketDescriptor);
      break;
    case 4: // steering
      disp->sendSteeringInfo(socketDescriptor);
      break;
    case 5: // gear
      disp->sendGearInfo(socketDescriptor);
      break;
    case 6: // steering alarms
      disp->sendSteeringAlarmsInfo(socketDescriptor);
      break;
    case 7: // drive alarms
      disp->sendDriveAlarmsInfo(socketDescriptor);
      break;
    case 8: // blinker left
      disp->sendBlinkerLeftInfo(socketDescriptor);
      break;
    case 9: // blinker right
      disp->sendBlinkerRightInfo(socketDescriptor);
      break;
    case 10: // emergency blinkers
      disp->sendEmergencyBlinkerInfo(socketDescriptor);
      break;
    case 11: // dipss
      disp->sendDipssInfo(socketDescriptor);
      break;
    case 12: // dipsr
      disp->sendDipsrInfo(socketDescriptor);
      break;
    case 13: // dipsp
      disp->sendDipspInfo(socketDescriptor);
      break;
    case 14: // klaxon
      disp->sendKlaxonInfo(socketDescriptor);
      break;
    case 15: // mt_throttle
      disp->sendMTThrottleInfo(socketDescriptor);
      break;
    case 16: // mt_brake
      disp->sendMTBrakeInfo(socketDescriptor);
      break;
    case 17: // mt_handbrake
      disp->sendMTHandbrakeInfo(socketDescriptor);
      break;
    case 18: // mr_steering
      disp->sendMTSteeringInfo(socketDescriptor);
      break;
    case 19: // mt_gear
      disp->sendMTGearInfo(socketDescriptor);
      break;
    case 20: // mt_blinker
      disp->sendMTBlinkersInfo(socketDescriptor);
      break;
    case 21: // mt_lights
      disp->sendMTLightsInfo(socketDescriptor);
      break;
    case 22: // motor rpm
      disp->sendMotorRPMInfo(socketDescriptor);
      break;
    case 23: // motor temperature
      disp->sendMotorTemperatureInfo(socketDescriptor);
      break;
    case 24: // cruissing speed
      disp->sendCruissingSpeedInfo(socketDescriptor);
      break;
    default: // nothing to do
      break;


  }
}