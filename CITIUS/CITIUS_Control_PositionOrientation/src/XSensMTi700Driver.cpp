
#include "XSensMTi700Driver.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

XSensMTi700Driver::XSensMTi700Driver() {
    canal = -1;
}

/*******************************************************************************
 * CONEXIÓN DEL DISPOSITIVO
 ******************************************************************************/

bool XSensMTi700Driver::connectToDevice() {
    
    char * serial_name = (char *) "/dev/ttyUSB3";

    canal = open(serial_name, O_RDWR | O_NOCTTY | O_NDELAY);

    if (canal < 0) {

        return false;

    } else {

        tcgetattr(canal, &oldtio);
        bzero(&newtio, sizeof (newtio));
        newtio.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
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
    }
    return true;
}

/*******************************************************************************
 * DESCONEXION DE DISPOSITIVO
 ******************************************************************************/

void XSensMTi700Driver::disconnectDevice(){
        close(canal);
        tcsetattr(STDOUT_FILENO,TCSANOW,&oldtio);
}

/*******************************************************************************
 * CONFIGURACION DE DISPOSITIVO DE LA CLASE
 ******************************************************************************/

void XSensMTi700Driver::configureDevice(){
    
    // Modo configuracion
    sendToDevice(goToConfig());
    
    // Configuracion de dispositivo
    sendToDevice(setOutPutConfiguration());
    
    // Modo stream de medidas
    sendToDevice(goToMeasurement());

}

/*******************************************************************************
 * RECOPILACION DE DATOS
 ******************************************************************************/

GPSINSInfo XSensMTi700Driver::getData(){
    GPSINSInfo info;
    info.orientationStatus = 2;
    return info;
}

/*******************************************************************************
 * FORMACION DE MENSAJES A ENVIAR
 ******************************************************************************/

XsensMsg XSensMTi700Driver::goToConfig() {
    XsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_GOTOCONFIG;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

XsensMsg XSensMTi700Driver::goToMeasurement() {
    XsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_GOTOMEASUREMENT;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

XsensMsg XSensMTi700Driver::setOutPutConfiguration() {
    XsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETOUTPUTCONFIGURATION;
    xsMsg.len = 0x18;
    // Temperatura
    xsMsg.data[0] = 0x08;
    xsMsg.data[1] = 0x13;
    xsMsg.data[2] = 0x00;
    xsMsg.data[3] = 0x01;
    // Orientacion
    xsMsg.data[4] = 0x20;
    xsMsg.data[5] = 0x33;
    xsMsg.data[6] = 0x00;
    xsMsg.data[7] = 0x01;
    // Posicion (Lat + Lon)
    xsMsg.data[8] = 0x50;
    xsMsg.data[9] = 0x43;
    xsMsg.data[10] = 0x00;
    xsMsg.data[11] = 0x01;
    // Posicion (Alt)
    xsMsg.data[12] = 0x50;
    xsMsg.data[13] = 0x20;
    xsMsg.data[14] = 0x00;
    xsMsg.data[15] = 0x01;
    // Aceleracion
    xsMsg.data[16] = 0x40;
    xsMsg.data[17] = 0x20;
    xsMsg.data[18] = 0x00;
    xsMsg.data[19] = 0x01;
    // Velocity
    xsMsg.data[20] = 0xD0;
    xsMsg.data[21] = 0x10;
    xsMsg.data[22] = 0x00;
    xsMsg.data[23] = 0x01;


    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

/*******************************************************************************
 * OPERACIONES CON CHECKSUM
 ******************************************************************************/

unsigned char XSensMTi700Driver::calcChecksum(XsensMsg msg) {
    unsigned char cs = 0;

    cs += msg.bid;
    cs += msg.mid;
    cs += msg.len;

    if (msg.len > 0) {
        for (int i = 0; i < msg.len; i++) {
            cs += msg.data[i];
        }
    }
    return 0x00 - cs;
}

bool XSensMTi700Driver::isCheckSumOK(XsensMsg msg) {
    unsigned char cs = 0;
    cs += msg.bid;
    cs += msg.mid;
    cs += msg.len;

    if (msg.len > 0) {
        for (int i = 0; i < msg.len; i++) {
            cs += msg.data[i];
        }
    }
    cs += msg.cs;

    return cs == 0x00;
}

/*******************************************************************************
 * OPERACIONES DE ENVIO DE MENSAJES Y RECEPCION DE ACK
 ******************************************************************************/

void XSensMTi700Driver::sendToDevice(XsensMsg msg) {
    unsigned char *msg2send = (unsigned char *) malloc(msg.len + 5);

    msg2send[0] = msg.pre;
    msg2send[1] = msg.bid;
    msg2send[2] = msg.mid;
    msg2send[3] = msg.len;

    if (msg.len > 0) {
        for (int i = 0; i < msg.len; i++) {
            msg2send[i + 4] = msg.data[i];
        }
    }

    msg2send[msg.len + 4] = msg.cs;
    
    printf("Sent: ");
    for(int i=0;i< (msg.len + 5);i++){
        printf("%02X ",msg2send[i]&0xFF);
    }
    printf("\n");
    
    write(canal, msg2send, msg.len + 5); // Enviamos el comando para que comience a enviarnos datos
    waitForAck(msg.mid);

}

void XSensMTi700Driver::waitForAck(unsigned char _mid) {

    bool ackFound = false;
    XsensMsg xsMsg;

    while (!ackFound) {

        if (read(canal, &xsMsg.pre, 1) > 0) {

            // PRE
            if (xsMsg.pre == COMMAND_PRE) {
                printf("PRE found!\n");
                
                read(canal, &xsMsg.bid, 1);

                // BID
                if (xsMsg.bid == COMMAND_BID) {
                    printf("BID found!\n");
                    read(canal, &xsMsg.mid, 1);
                    printf("MID found! :: %02X\n",xsMsg.mid);
                    // MID
                    switch (xsMsg.mid) {
                        case (COMMAND_MID_GOTOCONFIG + 1):

                            //LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("GoToConfig ACK received!\n");
                            }else{
                                //printf("ERROR in GotoConfig ACK reception\n");
                            }
                            printf("---\n"); 
                            break;
                        case (COMMAND_MID_REQDID + 1):

                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");    
                            break;
                        case (COMMAND_MID_SETOUTPUTMODE + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");    
                            break;
                        case (COMMAND_MID_SETOUTPUTSETTINGS + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");    
                            break;
                        case (COMMAND_MID_GOTOMEASUREMENT + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");
                            break;
                        case (COMMAND_MID_SETPERIOD + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");
                            break;
                        case (COMMAND_MID_SETOUTPUTSKIPFACTOR + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");
                            break;
                        case (COMMAND_MID_SETOUTPUTCONFIGURATION + 1):
                            // LEN
                            read(canal, &xsMsg.len, 1);
                            printf("LEN found! :: %02X\n",xsMsg.len);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            printf("DATA found! :: ");
                            for (int i = 0; i < xsMsg.len; i++) {
                                read(canal, &xsMsg.data[i], 1);
                                printf("%02X ",xsMsg.data[i]);
                            }
                            printf("\n");
                            
                            // CS
                            read(canal, &xsMsg.cs, 1);
                            printf("CS found! :: %02X\n",xsMsg.cs);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                                //printf("Device_ID ACK received!\n Device ID:");
                                for(int i=0;i<xsMsg.len;i++) printf("%02X",xsMsg.data[i]);
                                printf("\n");
                            }else{
                                //printf("ERROR in Device_ID ACK reception\n");
                            }
                            printf("---\n");
                            break;
                    }
                }
            }
        }
    }
}

/*******************************************************************************
 * FUNCIONES DE CONVERSION DE TIPOS
 ******************************************************************************/

float XSensMTi700Driver::hexa2float(unsigned char * buffer){
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

int XSensMTi700Driver::hexa2int(unsigned char * buffer){
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

double XSensMTi700Driver::hexa2double(unsigned char * buffer){
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

