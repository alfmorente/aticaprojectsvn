/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 1 de julio de 2014, 9:55
 */

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>

#define COMMAND_PRE 0xFA
#define COMMAND_BID 0xFF
#define COMMAND_MID_GOTOCONFIG 0x30
#define COMMAND_MID_REQOUTPUTMODE 0xD0
#define COMMAND_MID_REQDID 0x00
#define COMMAND_LEN_0 0x00

typedef struct{
    unsigned char pre;
    unsigned char bid;
    unsigned char mid;
    unsigned char len;
    unsigned char *data;
    unsigned char cs;
}xsensMsg;

unsigned char calcChecksum(xsensMsg);
bool isCheckSumOK(xsensMsg);
xsensMsg reqDeviceID();
void sendToDevice(xsensMsg);
xsensMsg goToConfig();

struct termios newtio,oldtio;
int canal;

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {

    char * serial_name = (char *)"/dev/ttyUSB0";
    
    canal = open(serial_name, O_RDWR | O_NOCTTY);
    
    if (canal < 0) {
        
        perror(serial_name);
        return -1;
        //exit(-1);
        
    } else {
        
        tcgetattr(canal, &oldtio);
        bzero(&newtio, sizeof (newtio));
        newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        /*	
        BAUDRATE: Fija la tasa bps. Podria tambien usar cfsetispeed y cfsetospeed.    
        CS8     : 8n1 (8bit,no paridad,1 bit de parada)
        CLOCAL  : conexion local, sin control de modem
        CREAD   : activa recepcion de caracteres
         */
        newtio.c_iflag = IGNPAR | ICRNL;
        /*
        IGNPAR  : ignora los bytes con error de paridad
        ICRNL   : mapea CR a NL (en otro caso una entrada CR del otro ordenador no terminaria la entrada) en otro caso hace un dispositivo en bruto (sin otro proceso de entrada)
         */
        newtio.c_oflag = 0;
        /*
        Salida en bruto.
         */
        newtio.c_lflag = ICANON;
        /*
        ICANON  : activa entrada canonica desactiva todas las funcionalidades del eco, y no envia segnales al programa llamador
         */

        /* 
        inicializa todos los caracteres de control los valores por defecto se pueden encontrar en /usr/include/termios.h, y vienen dadas en los comentarios, pero no los necesitamos aqui
         */
        newtio.c_cc[VINTR] = 0; /* Ctrl-c */
        newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
        newtio.c_cc[VERASE] = 0; /* del */
        newtio.c_cc[VKILL] = 0; /* @ */
        newtio.c_cc[VEOF] = 4; /* Ctrl-d */
        newtio.c_cc[VTIME] = 0; /* temporizador entre caracter, no usado */
        newtio.c_cc[VMIN] = 1; /* bloqu.lectura hasta llegada de caracter. 1 */
        newtio.c_cc[VSWTC] = 0; /* '\0' */
        newtio.c_cc[VSTART] = 0; /* Ctrl-q */
        newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
        newtio.c_cc[VSUSP] = 0; /* Ctrl-z */
        newtio.c_cc[VEOL] = 0; /* '\0' */
        newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
        newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
        newtio.c_cc[VWERASE] = 0; /* Ctrl-w */
        newtio.c_cc[VLNEXT] = 0; /* Ctrl-v */
        newtio.c_cc[VEOL2] = 0; /* '\0' */

        /* 
        ahora limpiamos la linea del modem y activamos la configuracion del puerto
         */
        tcflush(canal, TCIFLUSH);
        tcsetattr(canal, TCSANOW, &newtio);

        //escrito = write(canal, comando, strlen(comando) - 1); // Enviamos el comando para que comience a enviarnos datos
        //escrito = write(canal, comando2, strlen(comando2) - 1); // Enviamos el comando para que comience a enviarnos datos

        /*xsensMsg msg1 = goToConfig();
        sendToDevice(msg1);*/
        
        xsensMsg msg2 = reqDeviceID();
        sendToDevice(msg2);

    }
    return 0;
}

// Calculo de checkSum
unsigned char calcChecksum(xsensMsg msg){
    unsigned char cs = 0;
    
    cs += msg.bid;
    cs += msg.mid;
    cs += msg.len;
    
    if(msg.len > 0){
        for(int i = 0; i < msg.len; i++){
            cs += msg.data[i];
        }
    }
    return 0x00 - cs;
}

// Comprobacion del checksum
bool isCheckSumOK(xsensMsg msg){
    unsigned char cs = 0;
    cs += msg.bid;
    cs += msg.mid;
    cs += msg.len;
    
    if(msg.len > 0){
        for(int i = 0; i < msg.len; i++){
            cs += msg.data[i];
        }
    }
    cs += msg.cs;
    
    return cs == 0x00;
}

// GetDeviceID
xsensMsg reqDeviceID(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_REQDID;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// GoToConfig
xsensMsg goToConfig(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_GOTOCONFIG;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

void sendToDevice(xsensMsg msg){
    unsigned char *msg2send = (unsigned char *) malloc(msg.len + 5);
    
    msg2send[0] = msg.pre;
    msg2send[1] = msg.bid;
    msg2send[2] = msg.mid;
    msg2send[3] = msg.len;
    
    if(msg.len > 0){
        for(int i = 0; i < msg.len; i++){
            msg2send[i + 4] = msg.data[i];
        }
    }
    
    msg2send[msg.len + 4] = msg.cs;
    
    for(int i=0;i<msg.len+5;i++){
        printf("%02X\n",0xFF & msg2send[i]);
    }
    
    write(canal, msg2send, msg.len + 5); // Enviamos el comando para que comience a enviarnos datos
    unsigned char prueba;
    while(true){
        
        read(canal,&prueba,1);
        
        if(prueba == COMMAND_PRE){
            
            read(canal,&prueba,1);
            
            if(prueba == COMMAND_BID){
                
                
                read(canal,&prueba,1);
                
                printf("%02X\n",prueba & 0xFF);
                
                
                if(prueba == (COMMAND_MID_REQDID + 0x01)){
                    
                    printf("OLEEE OLEEE OLEEEEE!!!!\n");
                    
                }
                
            }
            
        }
        
    }
    
}



