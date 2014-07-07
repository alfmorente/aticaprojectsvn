/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 1 de julio de 2014, 9:55
 */

#include "constant.h"

using namespace std;



/*
 * 
 */
int main(int argc, char** argv) {

    char * serial_name = (char *) "/dev/ttyUSB0";

    canal = open(serial_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (canal < 0) {

        perror(serial_name);
        return -1;

    } else {

        tcgetattr(canal, &oldtio);
        bzero(&newtio, sizeof (newtio));
        newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
        newtio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
        newtio.c_oflag = 0;
        newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

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

        tcflush(canal, TCIFLUSH);
        tcsetattr(canal, TCSANOW, &newtio);
        
        sendToDevice();
        
        close(canal);
    }
    return 0;
}



void sendToDevice() {
    char *msg2send = (char *) malloc(5);

    msg2send[0] = 0x00;
    msg2send[1] = 0x05;
    msg2send[2] = 0x04;
    unsigned short crc = crc16ccitt_xmodem((uint8_t *)msg2send,3);
    char *crc16 = shortToHexa(crc);
    msg2send[3] = crc16[0];
    msg2send[4] = crc16[1];
        
   
    printf("Sent: ");
    for(int i=0;i< 5;i++){
        printf("%02X ",msg2send[i]&0xFF);
    }
    printf("\n");
    
    printf("%d\n",write(canal, msg2send, 5)); 
    read();

}

void read() {

    unsigned char byte;
    bool ackFound = false;    
    int cuentavieja = 0;

    while (!ackFound) {

        if (read(canal, &byte, 1) > 0) {
            printf("%d: %02X ",cuentavieja+1, byte);
            cuentavieja++;
            if(cuentavieja == 11)
                ackFound = true;
        }
        
    }
}

float hexa2float(unsigned char * buffer){
    union
    {
        float value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}

int hexa2int(unsigned char * buffer){
    union
    {
        int value;
        unsigned char buffer[4];

    }floatUnion;

    floatUnion.buffer[0] = buffer[3];
    floatUnion.buffer[1] = buffer[2];
    floatUnion.buffer[2] = buffer[1];
    floatUnion.buffer[3] = buffer[0];

    return floatUnion.value;
}

double hexa2double(unsigned char * buffer){
    union{
        double value;
        unsigned char buffer[8];
    }floatUnion;

    floatUnion.buffer[0] = buffer[7];
    floatUnion.buffer[1] = buffer[6];
    floatUnion.buffer[2] = buffer[5];
    floatUnion.buffer[3] = buffer[4];
    floatUnion.buffer[4] = buffer[3];
    floatUnion.buffer[5] = buffer[2];
    floatUnion.buffer[6] = buffer[1];
    floatUnion.buffer[7] = buffer[0];

    return floatUnion.value;
}

char *shortToHexa(short s){
    char *buf = (char *) malloc(2);
    memcpy(buf,&s,2);
    return buf;

}


uint16_t straight_16(uint16_t value) {
    return value;
}

uint8_t straight_8(uint8_t value) {
    return value;
}

uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 data_order, bit_order_16 remainder_order,uint16_t remainder, uint16_t polynomial) {
    for (int byte = 0; byte < nBytes; ++byte) {
        remainder ^= (data_order(message[byte]) << 8);
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ polynomial;
            } else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder_order(remainder);
}

uint16_t crc16ccitt_xmodem(uint8_t  *message, int nBytes) {
    return crc16(message, nBytes, straight_8, straight_16, 0x0000, 0x1021);
}
