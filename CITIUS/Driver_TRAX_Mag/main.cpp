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
        
        //sendToDevice(kGetModInfo());
        //rcvResponse();
        
        sendToDevice(kSetDataComponents());
        
        sendToDevice(kGetData());
        sendToDevice(kGetData());
        sendToDevice(kGetData());

        
        
        close(canal);
    }
    return 0;
}

void sendToDevice(TraxMsg traxMsg) {
    short len = hexa2short(traxMsg.byteCount);
    
    char *msg2send = (char *) malloc(len);
    
    int index = 0;
    
    // Bytecount
    msg2send[index++] = traxMsg.byteCount[0];
    msg2send[index++] = traxMsg.byteCount[1];
    
    // ID Frame
    msg2send[index++] = traxMsg.packFrame.idFrame;

    // Payload
    if (len>5) {
        for (int i = 0; i < len - 5; i++) {
            msg2send[index++] = traxMsg.packFrame.payload[i];
        }
    }
     
    // CRC16
    short crc16short = crc16ccitt_xmodem((uint8_t *)msg2send,len-2);
    msg2send[index++] = shortToHexa(crc16short)[0];
    msg2send[index++] = shortToHexa(crc16short)[1];
      
   
    printf("Enviado: ");
    for(int i=0;i< len;i++){
        printf("%02X ",msg2send[i]&0xFF);
    }
    printf("\n");
    
    if(write(canal, msg2send, len)==len){
        if(traxMsg.packFrame.idFrame == IDFRAME_KGETDATA) rcvResponse();
    }else {
        printf("Problemas en la escritura\n");
    }
    
    free(msg2send);
            
}

void rcvResponse() {
    
    // Contador de bytes leidos
    int index = 0;
    // Buffer de recepcion
    //unsigned char* recievedFrame;
    vector< char> recievedFrame;
    //unsigned char byte;
    TraxMsg msgRcv;
    
    unsigned char byte;

    // Tamaño de la trama
    while (index < 2) {
        if (read(canal, &byte, 1) > 0) {
            recievedFrame.push_back(byte);
            index++;
        }
    }

    short tam = hexa2short(recievedFrame);

    recievedFrame.clear();

    recievedFrame.push_back(shortToHexa(tam)[0]);
    recievedFrame.push_back(shortToHexa(tam)[1]);

    // Resto de la trama
    while (index < tam) {
        if (read(canal, &byte, 1) > 0) {
            recievedFrame.push_back(byte);
            index++;
        }
    }
    
    TraxMsg pkg = mngPacket(recievedFrame);
    
    if(pkg.checked){
        TraxMeasurement measur = unpackPayload(pkg.packFrame.payload);
        printf("HEADING STATUS: %d\n",measur.heading_status);
        printf("ORIENTACION ROLL: %f PITCH %f HEADING %f\n",measur.roll,measur.pitch, measur.heading);
        printf("ACC ACCX: %f ACCY %f ACCZ %f\n",measur.accX,measur.accY, measur.accZ);
        printf("GYR GYRX: %f GYRY %f GYRZ %f\n",measur.gyrX,measur.gyrY, measur.gyrZ);
    }
    
}

TraxMsg mngPacket(vector<char> bufferPacket){
    TraxMsg packet;
    
    packet.checked = false;

    // Desempaqueta del paquete
    
    // Tamaño
    packet.byteCount.push_back(bufferPacket[0]);
    packet.byteCount.push_back(bufferPacket[1]);
    short tam = hexa2short(packet.byteCount);

    // Frame ID
    packet.packFrame.idFrame = bufferPacket[2];

    // Payload
    for (int i = 0; i < (tam - 5); i++) {
        packet.packFrame.payload.push_back(bufferPacket[i + 3]);
    }

    // CRC16
    packet.crc.push_back(bufferPacket[tam-2]);
    packet.crc.push_back(bufferPacket[tam-1]);

    char *auxBuff = (char *) malloc(tam-2);
    for(int i=0;i < tam -2; i++){
        auxBuff[i] = bufferPacket[i];
    }
    if((short)(crc16ccitt_xmodem((uint8_t *)auxBuff,tam-2)) == hexa2short(packet.crc)){
        // Recepcion correcta por CRC16
        packet.checked = true;
    }else{
        printf("\n\n%d <> %d \n\n",crc16ccitt_xmodem(((uint8_t *)auxBuff),tam-2),hexa2short(packet.crc));
    }
    free(auxBuff);
    
    return packet;
   
}

TraxMsg kGetModInfo(){
    
    TraxMsg retTraxMsg;
    
    retTraxMsg.byteCount = shortToHexa(5);
    retTraxMsg.packFrame.idFrame = IDFRAME_KGETMODINFO;
    
    return retTraxMsg;
}

TraxMsg kGetData(){
    
    TraxMsg retTraxMsg;
    
    retTraxMsg.byteCount = shortToHexa(5);
    retTraxMsg.packFrame.idFrame = IDFRAME_KGETDATA;
    
    return retTraxMsg;
}

TraxMsg kSetDataComponents(){
    TraxMsg retTraxMsg;
    
    retTraxMsg.byteCount = shortToHexa(10);
    retTraxMsg.packFrame.idFrame = IDFRAME_KSETDATACOMPONENTS;
    
    // Se solicita heading, roll, pitch, heading status
    retTraxMsg.packFrame.payload.push_back(0x0A);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_HEADING);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_PITCH);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_ROLL);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_HEADING_STATUS);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCX);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCY);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCZ);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRX);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRY);
    retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRZ);
        
    return retTraxMsg;
}

TraxMeasurement unpackPayload( std::vector<char> payload){
    int tam = payload[0];
    int numData = 0;
    int index = 1;
   // char *bufFloat = (char *)malloc(4);
    vector<char> bufFloat;
    
    TraxMeasurement measureDev;
    
    while(numData < tam){
        switch(payload[index]){
            case IDMEASURE_HEADING:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.heading = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_PITCH:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.pitch = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ROLL:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.roll = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_HEADING_STATUS:
                measureDev.heading_status = payload[index+1];
                index+=2;
                break;
            case IDMEASURE_ACCX:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.accX = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ACCY:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.accY = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ACCZ:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.accZ = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRX:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.gyrX = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRY:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.gyrY = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRZ:
                bufFloat.push_back(payload[index+1]);
                bufFloat.push_back(payload[index+2]);
                bufFloat.push_back(payload[index+3]);
                bufFloat.push_back(payload[index+4]);
                measureDev.gyrZ = hexa2float(bufFloat);
                index+=5;
                break;
            default:
                break;
        }
        numData++;
    }
    return measureDev;
}