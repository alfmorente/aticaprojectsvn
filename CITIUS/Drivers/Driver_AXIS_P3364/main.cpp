/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 23 de julio de 2014, 11:34
 */

#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>

#define IP_CAMERA "192.168.24.120"
#define PORT_CAMERA 80

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    int socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    
    if (socketDescriptor < 0) {

        return -1;
        
    } else {
        
        struct hostent *he;
        /* estructura que recibirá información sobre el nodo remoto */

        struct sockaddr_in server;
        /* información sobre la dirección del servidor */

        if ((he = gethostbyname(IP_CAMERA)) == NULL) {
            /* llamada a gethostbyname() */
            return -1;
        }

        if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            /* llamada a socket() */
            return -1;
        }

        server.sin_family = AF_INET;
        server.sin_port = htons(PORT_CAMERA);
        /* htons() es necesaria nuevamente ;-o */
        server.sin_addr = *((struct in_addr *) he->h_addr);
        
        /*he->h_addr pasa la información de ``*he'' a "h_addr" */
        bzero(&(server.sin_zero), 8);

        if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
            /* llamada a connect() */
            return -1;

        }else{
            printf("Conectado a la camara AXIS:\n");
            
            char *comando = (char *)"GET http://root@ugv:192.168.24.120/axis-cgi/com/ptz.cgi?query=position\r\n";
            char respuesta[256];
            int nBytesSent = send(socketDescriptor,comando,strlen(comando),0);
            if(nBytesSent < 0){
                printf("Ha fallado la escritura. NOOOO!!!!!\n");
            }else{
                printf("Ha enviado %d bytes\n",nBytesSent);
                
                int nBytes = recv(socketDescriptor,respuesta,256,0);
                
                if(nBytes<0){
                    
                    printf("Ha fallado la lectura. CAGUEN...!!!!\n");
                    
                }else{
                    respuesta[nBytes]='\0';
                    printf("%s\n",respuesta);
                    
                }
            }
        }
        

        return 0;
    }

}