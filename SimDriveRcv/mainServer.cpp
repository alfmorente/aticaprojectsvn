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

#define PORT 10000 /* El puerto que será abierto */
#define BACKLOG 2 /* El número de conexiones permitidas */

#define MAXDATASIZE 100   

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE INSTRUCCION
*******************************************************************************/

#define SET 0
#define GET 1
#define INFO 2
#define ACK 3

/*******************************************************************************
 *           ESTRUCTURA DE INTERCAMBIO CON PAYLOAD DE CONDUCCION
*******************************************************************************/

typedef struct{
    short instruction;
    short element;
    short value;
}FrameDriving;

/* El número máximo de datos en bytes */

int main(int argc, char *argv[]) {

    int fd, fd2; /* los ficheros descriptores */

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
        printf("error en bind() \n");
        exit(-1);
    }

    if (listen(fd, BACKLOG) == -1) { /* llamada a listen() */
        printf("error en listen()\n");
        exit(-1);
    }
    sin_size = sizeof (struct sockaddr_in);
    /* A continuación la llamada a accept() */
    if ((fd2 = accept(fd, (struct sockaddr *) &client, &sin_size)) == -1) {
        printf("error en accept()\n");
        exit(-1);
    }

    printf("Se obtuvo una conexión desde %s\n", inet_ntoa(client.sin_addr));
    /* que mostrará la IP del cliente */


    while (1) {
        
        // Buffer de envio
        char bufData[6];
        if ((recv(fd2, bufData, MAXDATASIZE, 0)) == -1) {
            /* llamada a recv() */
            printf("Error en recv() \n");
            exit(-1);
        } else {
            // Estructura de recepcion
            FrameDriving fdr;
            // Rellenado del buffer
            memcpy(&fdr.instruction, &bufData[0], sizeof (fdr.instruction));
            memcpy(&fdr.element, &bufData[2], sizeof (fdr.element));
            memcpy(&fdr.value, &bufData[4], sizeof (fdr.value));
            printf("Recibido comando INS: %d ELM: %d VAL: %d\n", fdr.instruction, fdr.element, fdr.value);
            
            // Estructura de envio de mensaje de devolucion
            FrameDriving fdr2;
            if (fdr.instruction == SET) {
                fdr2.instruction = ACK;
            } else if (fdr.instruction == GET) {
                fdr2.instruction = INFO;
            }
            
            fdr2.element = fdr.element;
            fdr2.value = fdr.value;
            
            // Rellenado del buffer
            memcpy(&bufData[0], &fdr2.instruction, sizeof (fdr2.instruction));
            memcpy(&bufData[2], &fdr2.element, sizeof (fdr2.element));
            memcpy(&bufData[4], &fdr2.value, sizeof (fdr2.value));
            send(fd2, bufData, sizeof(bufData), 0);
            usleep(100); 
        }


        /*printf("Mensaje del cliente: %s\n",buf); 
        send(fd2, buf, MAXDATASIZE, 0);*/
        /* que enviará el mensaje de bienvenida al cliente */

    }
}
