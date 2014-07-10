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
        rcvResponse();
        
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
    
    write(canal, msg2send, len); 
            
}

void rcvResponse() {
    
    // Contador de bytes leidos
    int index = 0;
    // Buffer de recepcion
    unsigned char* recievedFrame;
    //unsigned char byte;
    TraxMsg msgRcv;

    recievedFrame = (unsigned char *) malloc(2);
    // Tamaño de la trama
    while (index < 2) {
        if (read(canal, &recievedFrame[index], 1) > 0) {
            index++;
        }
    }

    short tam = hexa2short((char *) recievedFrame);

    recievedFrame = (unsigned char *) malloc(tam);
    
    recievedFrame[0] = shortToHexa(tam)[0];
    recievedFrame[1] = shortToHexa(tam)[1];

    // Resto de la trama
    while (index < tam) {
        if (read(canal, &recievedFrame[index], 1) > 0) {
            index++;
        }
    }
    
    printf("Recibida: ");
    for(int i = 0; i < tam ; i++){
        printf("%02X ",recievedFrame[i]);
    }
    printf("\n");

    TraxMsg pkg = mngPacket(recievedFrame);
    
    if(pkg.checked){
        TraxMeasurement measur = unpackPayload(pkg.packFrame.payload);
        printf("HEADING STATUS: %d\n",measur.heading_status);
        printf("ORIENTACION ROLL: %f PITCH %f HEADING %f\n",measur.roll,measur.pitch, measur.heading);
        printf("ACC ACCX: %f ACCY %f ACCZ %f\n",measur.accX,measur.accY, measur.accZ);
        printf("GYR GYRX: %f GYRY %f GYRZ %f\n",measur.gyrX,measur.gyrY, measur.gyrZ);
    }
    
}

TraxMsg mngPacket(unsigned char *bufferPacket){
    TraxMsg packet;
    
    packet.checked = false;

    // Desempaqueta del paquete
    
    // Tamaño
    packet.byteCount = (char *) malloc(2);
    packet.byteCount[0] = bufferPacket[0];
    packet.byteCount[1] = bufferPacket[1];
    short tam = hexa2short(packet.byteCount);

    // Frame ID
    packet.packFrame.idFrame = bufferPacket[2];

    // Payload
    packet.packFrame.payload = (char *) malloc(tam - 5);

    for (int i = 0; i < (tam - 5); i++) {
        packet.packFrame.payload[i] = bufferPacket[i + 3];
    }


    // CRC16
    packet.crc = (char *) malloc(2);
    packet.crc[0] = bufferPacket[tam-2];
    packet.crc[1] = bufferPacket[tam-1];
    
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

    retTraxMsg.packFrame.payload = (char *) malloc(5);
    
    // Se solicita heading, roll, pitch, heading status
    retTraxMsg.packFrame.payload[0] = 0x0A; // 10 medidas
    retTraxMsg.packFrame.payload[1] = IDMEASURE_HEADING; // heading
    retTraxMsg.packFrame.payload[2] = IDMEASURE_PITCH; // pitch
    retTraxMsg.packFrame.payload[3] = IDMEASURE_ROLL; // roll
    retTraxMsg.packFrame.payload[4] = IDMEASURE_HEADING_STATUS; // heading status
    retTraxMsg.packFrame.payload[5] = IDMEASURE_ACCX; // accX
    retTraxMsg.packFrame.payload[6] = IDMEASURE_ACCY; // accY
    retTraxMsg.packFrame.payload[7] = IDMEASURE_ACCZ; // accZ
    retTraxMsg.packFrame.payload[8] = IDMEASURE_GYRX; // gyrX
    retTraxMsg.packFrame.payload[9] = IDMEASURE_GYRY; // gyrY
    retTraxMsg.packFrame.payload[10] = IDMEASURE_GYRZ; // gyrZ
    
    return retTraxMsg;
}

TraxMeasurement unpackPayload( char* payload){
    int tam = payload[0];
    int numData = 0;
    int index = 1;
    char *bufFloat = (char *)malloc(4);
    
    TraxMeasurement measureDev;
    
    while(numData < tam){
        switch(payload[index]){
            case IDMEASURE_HEADING:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.heading = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_PITCH:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.pitch = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ROLL:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.roll = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_HEADING_STATUS:
                measureDev.heading_status = payload[index+1];
                index+=2;
                break;
            case IDMEASURE_ACCX:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.accX = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ACCY:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.accY = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_ACCZ:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.accZ = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRX:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.gyrX = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRY:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
                measureDev.gyrY = hexa2float(bufFloat);
                index+=5;
                break;
            case IDMEASURE_GYRZ:
                bufFloat[0] = payload[index+1];
                bufFloat[1] = payload[index+2];
                bufFloat[2] = payload[index+3];
                bufFloat[3] = payload[index+4];
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