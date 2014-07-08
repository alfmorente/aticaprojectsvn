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

    char * serial_name = (char *) "/dev/ttyUSB1";

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
        
        sendToDevice(kGetModInfo());
        //sendToDevice(kSetDataComponents());
        //sendToDevice(kGetData());
        
        close(canal);
    }
    return 0;
}

void sendToDevice(TraxMsg traxMsg) {
    short len = hexa2short(traxMsg.byteCount);
    
    char *msg2send = (char *) malloc(len);
    
    int index = 0;

    msg2send[index++] = traxMsg.byteCount[0];
    msg2send[index++] = traxMsg.byteCount[1];
    
    msg2send[index++] = traxMsg.packFrame.idFrame;
    
    switch(traxMsg.packFrame.idFrame){
        case IDFRAME_KGETDATA:
            break;
        case IDFRAME_KSETDATACOMPONENTS:
            msg2send[index++] = traxMsg.packFrame.payload.idCount;
            for(int i = 0; i < traxMsg.packFrame.payload.idCount; i++){
                msg2send[index++] = traxMsg.packFrame.payload.packetData[i].idValue;
            }
            break;
        default:
            break;
    }
    
    msg2send[index++] = traxMsg.crc[0];
    msg2send[index++] = traxMsg.crc[1];
        
   
    printf("Sent: ");
    for(int i=0;i< len;i++){
        printf("%02X ",msg2send[i]&0xFF);
    }
    printf("\n");
    
    printf("%d\n",write(canal, msg2send, len)); 
            
    rcvResponse(traxMsg.packFrame.idFrame);

}

TraxMsg kGetModInfo(){
    TraxMsg retTraxMsg;
    
    retTraxMsg.byteCount = shortToHexa(5);
    
    retTraxMsg.packFrame.idFrame = IDFRAME_KGETMODINFO;
    
    char *frame = (char *)malloc(3);
    frame[0] = retTraxMsg.byteCount[0]; frame[1] = retTraxMsg.byteCount[1]; frame[2] = retTraxMsg.packFrame.idFrame;
    
    retTraxMsg.crc = shortToHexa(crc16ccitt_xmodem((uint8_t *)frame,3));
    
    return retTraxMsg;
}

TraxMsg kGetData(){
    TraxMsg retTraxMsg;
    retTraxMsg.byteCount = shortToHexa(5);
    retTraxMsg.packFrame.idFrame = IDFRAME_KGETDATA;
    char *frame = (char *)malloc(3);
    frame[0] = retTraxMsg.byteCount[0]; frame[1] = retTraxMsg.byteCount[1]; frame[2] = retTraxMsg.packFrame.idFrame;
    retTraxMsg.crc = shortToHexa(crc16ccitt_xmodem((uint8_t *)frame,3));
    return retTraxMsg;
}

TraxMsg kSetDataComponents(){
    TraxMsg retTraxMsg;
    retTraxMsg.byteCount = shortToHexa(10);
    retTraxMsg.packFrame.idFrame = IDFRAME_KSETDATACOMPONENTS;

    retTraxMsg.packFrame.payload.idCount = 0x04;
    retTraxMsg.packFrame.payload.packetData = (PacketData *)malloc(retTraxMsg.packFrame.payload.idCount);
    
    retTraxMsg.packFrame.payload.packetData[0].idValue = 0x05;
    retTraxMsg.packFrame.payload.packetData[1].idValue = 0x18;
    retTraxMsg.packFrame.payload.packetData[2].idValue = 0x19;
    retTraxMsg.packFrame.payload.packetData[2].idValue = 0x4F;
    
    char *frame = (char *)malloc(3 + retTraxMsg.packFrame.payload.idCount);
    
    frame[0] = retTraxMsg.byteCount[0];
    frame[1] = retTraxMsg.byteCount[1];
    frame[2] = retTraxMsg.packFrame.idFrame = IDFRAME_KSETDATACOMPONENTS;
    frame[3] = retTraxMsg.packFrame.payload.idCount = 0x04;
    frame[4] = retTraxMsg.packFrame.payload.packetData[0].idValue;
    frame[5] = retTraxMsg.packFrame.payload.packetData[1].idValue;
    frame[6] = retTraxMsg.packFrame.payload.packetData[2].idValue;
    frame[7] = retTraxMsg.packFrame.payload.packetData[3].idValue;
    
    retTraxMsg.crc = shortToHexa(crc16ccitt_xmodem((uint8_t *)frame,8));
    
    return retTraxMsg;
}


void rcvResponse(char idFrame) {
    bool endRcv = false;
    unsigned char byte;
    int index = 0;
/*
    while (!endRcv) {
        
        if(read(canal, &msgRcv.byteCount, 2) > 0){
            
            short a = hexa2short(msgRcv.byteCount);
            
            if(read(canal, &msgRcv.packFrame.idFrame,1)>0){
                
                printf("leida trama: %02X\n",msgRcv.packFrame.idFrame);
                
                endRcv = true;
            }
        }
    }
    printf("\n-----\n");
 * */
    printf("Lectura: %d\n", idFrame);
    
    unsigned char* recievedFrame;
    unsigned char* tamano;
    //unsigned char byte;
    TraxMsg msgRcv;
    
    switch(idFrame) {
        
        case IDFRAME_KGETMODINFO:
            recievedFrame = (unsigned char *) malloc(13);
            tamano = (unsigned char *) malloc(2);
            while (index < 13) {
              
                if (read(canal, &tamano[index], 1) > 0) {
                    index++;
                    if (read(canal, &tamano[index++], 1) > 0) {
                        short tam = hexa2short((char *) tamano);
                        printf("Tamano: %d\n", tam);
                        
                        while(index<tam){
                            if(read(canal, &byte, 1)>0){
                                printf("%02X ",byte);
                                index++;
                            }
                        }
                        
                    }
                }
                
            }
            
            break;
            
        default:
            printf("UNKNOWN FRAME\n");
            break;
    }
    
    printf("\n");
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
short hexa2short(char buffer[2]){
    union
    {
        short value;
        unsigned char buffer[2];

    }shortUnion;

    shortUnion.buffer[0] = buffer[1];
    shortUnion.buffer[1] = buffer[0];

    return shortUnion.value;
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
    char aux = buf[1];
    buf[1] = buf[0];
    buf[0] = aux;
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
