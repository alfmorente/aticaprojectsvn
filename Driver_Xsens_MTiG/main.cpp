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

        xsensMsg msg = goToConfig();
        sendToDevice(msg);
                
        /*msg = setOutPutSkipFactor();
        sendToDevice(msg);
        
        msg = setPeriod();
        sendToDevice(msg);
        
        msg = reqDeviceID();
        sendToDevice(msg);

        msg = setOutPutMode();
        sendToDevice(msg);
        
        msg = setOutPutSettings();
        sendToDevice(msg);*/
        msg = setOutPutConfiguration();
        sendToDevice(msg);
        
        msg = goToMeasurement();
        sendToDevice(msg);
        
        streamDataMng();

    }
    return 0;
}



void sendToDevice(xsensMsg msg) {
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

void waitForAck(unsigned char _mid) {

    unsigned char byte;
    unsigned char frameLen;
    bool ackFound = false;
    xsensMsg xsMsg;

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

unsigned char calcChecksum(xsensMsg msg) {
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

// Comprobacion del checksum

bool isCheckSumOK(xsensMsg msg) {
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

// GetDeviceID

xsensMsg reqDeviceID() {
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_REQDID;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// GoToConfig

xsensMsg goToConfig() {
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_GOTOCONFIG;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// GotToMeasurement

xsensMsg goToMeasurement() {
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_GOTOMEASUREMENT;
    xsMsg.len = COMMAND_LEN_0;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// SetOutPutMode

xsensMsg setOutPutMode(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETOUTPUTMODE;
    xsMsg.len = 0x02;
    
    
    outPutMode mode;
    mode.temperature = true;
    mode.calibrated_data = true;
    mode.orientation = true;
    mode.auxiliary_data = true;
    mode.position = true;
    mode.velocity = true;
    mode.status = true;
    mode.raw_gps = false;
    mode.raw_ins = false;

    unsigned short modeShort = 0;
    
    modeShort = mode.temperature |
            mode.calibrated_data << 1 |
            mode.orientation << 2 |
            mode.auxiliary_data << 3 |
            mode.position << 4 |
            mode.velocity << 5 |
            mode.status << 11 |
            mode.raw_gps << 12 |
            mode.raw_ins << 14;
    printf("Mode: %d\n", modeShort);
        
    //memcpy(xsMsg.data,&modeShort,2);
    xsMsg.data[1] = 0x06;
    xsMsg.data[0] = 0x00;


    
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// SetOutPutSettings

xsensMsg setOutPutSettings(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETOUTPUTSETTINGS;
    xsMsg.len = 0x04;
    xsMsg.data[0] = 0x00;
    xsMsg.data[1] = 0x00;
    xsMsg.data[2] = 0x00;
    xsMsg.data[3] = 0x09;
    
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// SetOupPutSkipFactor

xsensMsg setOutPutSkipFactor(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETOUTPUTSKIPFACTOR;
    xsMsg.len = 0x02;
    xsMsg.data[0] = 0x00;
    xsMsg.data[1] = 0x63;
    
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// SetPeriod

xsensMsg setPeriod(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETPERIOD;
    xsMsg.len = 0x02;
    xsMsg.data[0] = 0x04;
    xsMsg.data[1] = 0x80;
    
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

// SetOutPutConfiguration

xsensMsg setOutPutConfiguration(){
    xsensMsg xsMsg;
    xsMsg.pre = COMMAND_PRE;
    xsMsg.bid = COMMAND_BID;
    xsMsg.mid = COMMAND_MID_SETOUTPUTCONFIGURATION;
    xsMsg.len = 0x0C;
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
    // Posicion
    xsMsg.data[8] = 0x50;
    xsMsg.data[9] = 0x43;
    xsMsg.data[10] = 0x00;
    xsMsg.data[11] = 0x01;
    xsMsg.cs = calcChecksum(xsMsg);
    return xsMsg;
}

void streamDataMng() {
    xsensMsg xsMsg;

    
    while (true){
        if (read(canal, &xsMsg.pre, 1) > 0) {

            // PRE
            if (xsMsg.pre == COMMAND_PRE) {
                printf("PRE found!\n");

                read(canal, &xsMsg.bid, 1);

                // BID
                if (xsMsg.bid == COMMAND_BID) {
                    printf("BID found!\n");
                    read(canal, &xsMsg.mid, 1);
                    printf("MID found! :: %02X\n", xsMsg.mid);
                    // MID
                    if (xsMsg.mid == (COMMAND_MID_MTDATA2)) {

                        //LEN
                        read(canal, &xsMsg.len, 1);
                        printf("LEN found! :: %d\n", xsMsg.len);

                        // DATA
                        xsMsg.data = (unsigned char*) malloc(xsMsg.len + 1);
                        printf("DATA found! :: ");
                        for (int i = 0; i < xsMsg.len; i++) {
                            read(canal, &xsMsg.data[i], 1);
                            printf("%02X ", xsMsg.data[i]);
                        }
                        printf("\n");                        

                        // CS
                        read(canal, &xsMsg.cs, 1);
                        printf("CS found! :: %02X\n", xsMsg.cs);

                        if (isCheckSumOK(xsMsg)) {
                            printf("MTData2 ACK received!\n");
                        } else {
                            printf("ERROR in MTData2 ACK reception\n");
                        }
                        
                       printf("Iniciando el tratamiento de datos...\n");

                        printf("Data management...\n");

                        dataPacketMT2 dataPacket;
                        int i = 0;
                        
                        while(i<xsMsg.len){
                            
                            //unsigned char * aux = (unsigned char *)malloc(2);

                            dataPacket.idGroup = xsMsg.data[i++];
                            dataPacket.idSignal = xsMsg.data[i++];
                            dataPacket.len = xsMsg.data[i++];
                            
                            dataPacket.data = (unsigned char *) malloc(dataPacket.len);
                            
                            for(int j = 0; j < dataPacket.len;j++){
                                
                                dataPacket.data[j]=xsMsg.data[i++];
                            }
                            packetMng(dataPacket);
                            
                        }
                        printf("------------------------------------------------------------------------\n");
                    }
                }
            }
        }
    }

}

void packetMng(dataPacketMT2 dataPacket) {
    //printf("ID Packet found:Group %02X Signal %02X with length %d\n",dataPacket.idGroup,dataPacket.idSignal,dataPacket.len);
    unsigned char *auxBuf;
    unsigned short auxShort;
    int auxInt;
    float auxFloat;
    
    switch (dataPacket.idGroup) {
        
        case 0x10: // Timestamp
            printf("Got timestamp packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // UTC Time
                    printf("   UTC time %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Packet counter
                    auxBuf = (unsigned char *) malloc(2);
                    auxBuf[0] = dataPacket.data[1];
                    auxBuf[1] = dataPacket.data[0];
                    memcpy(&auxShort,auxBuf,2);
                    printf("   Packetcounter: %d\n",auxShort);
                    
                    break;
                case 0x30: // Integer Time of Week
                    printf("   Integer Time of Week %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // GPS Age
                    printf("   GPS Age %d bytes\n", dataPacket.len);
                    break;
                case 0x50: // Pressure Age
                    printf("   Pressure Age %d bytes\n", dataPacket.len);
                    break;
                case 0x60: // Sample Time Fine
                    printf("   Sample Time Fine %d bytes\n", dataPacket.len);
                    break;
                case 0x70: // Sample Time Coarse
                    printf("   Sample Time Coarse %d bytes\n", dataPacket.len);
                    break;
                case 0x80: // Frame Range
                    printf("   Frame Range %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: Timestamp %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;
            
            
        case 0x08: // Temperature
            printf("Got temperature packet\n");
            if ((dataPacket.idSignal & 0xF0) == 0x10) // Temperature
                printf("   Temperature: %d bytes\n", dataPacket.len);
            // printf("   Temperature: %f ºC\n", hexa2float(dataPacket.data));
            else
                printf("   UNKNOWN :: Temperature %d bytes\n", dataPacket.len);
            printf("\n");
            break;
            
            
        case 0x88: // GPS
            printf("Got GPS packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x30: // DOP
                    printf("   DOP %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // SOL
                    printf("   SOL %d bytes\n", dataPacket.len);
                    break;
                case 0x80: // Time UTC
                    printf("   Time UTC %d bytes\n", dataPacket.len);
                    break;
                case 0xA0: // SV info
                    printf("   SV info %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: GPS %d bytes\n", dataPacket.len);
                    break;
            }*/
            
            printf("\n");
            break;
            
            
        case 0x20: // Orientation
            printf("Got orientation packet\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Quaternion
                    printf("   Quaternion %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Rotation Matrix
                    printf("   Rotation Matrix %d bytes\n", dataPacket.len);
                    break;
                case 0x30: // Euler Angles
                    printf("   Euler Angles %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 0; i < 8; i++) auxBuf[i]=dataPacket.data[i];
                    printf("   Roll: %3.8lf\n", hexa2double(auxBuf));
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 8; i < 16; i++) auxBuf[i-8]=dataPacket.data[i];
                    printf("   Pitch: %3.8lf\n", hexa2double(auxBuf));
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 16; i < 24; i++) auxBuf[i-16]=dataPacket.data[i];
                    printf("   Yaw: %3.8lf\n", hexa2double(auxBuf));
                    break;
                default:
                    printf("   UNKNOWN :: Orientation %d bytes\n", dataPacket.len);
                    break;
            }
            
            printf("\n");
            break;
            
            
        case 0x30: // Pressure
            printf("Got pressure packet\n");
            /*if((dataPacket.idSignal & 0xF0) == 0x10) // Pressure
                printf("   Pressure: %f mbar\n",  (float) hexa2int(dataPacket.data) / 100);
            else
                printf("   UNKNOWN :: Pressure %d bytes \n", dataPacket.len);*/
            printf("\n");
            break;
            
            
        case 0x40: // Acceleration
            printf("Got acceleration packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Delta V
                    //printf("   Delta V %d bytes\n", dataPacket.len);
                    unsigned char * buf;
                    buf = (unsigned char *) malloc(4);
                    for(int i = 0 ; i < 4; i ++) buf[i] = dataPacket.data[i];
                    printf("   Delta V (X): %f\n",hexa2float(buf));
                    for(int i = 4 ; i < 8; i ++) buf[i-4] = dataPacket.data[i];
                    printf("   Delta V (Y): %f\n",hexa2float(buf));
                    for(int i = 8 ; i < 12; i ++) buf[i-8] = dataPacket.data[i];
                    printf("   Delta V (Z): %f\n",hexa2float(buf));
                    break;
                case 0x20: // Acceleration
                    printf("   Acceleration %d bytes\n", dataPacket.len);
                    break;
                case 0x30: // Free acceleration
                    printf("   Free acceleration %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: Acceleration %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;
            
            
        case 0x50: // Position
            printf("Got position packet\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Altitude MSL
                    printf("   Altitude MSL V %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Altitude Ellipsoid
                    printf("   Altitude Ellipsoid %d bytes\n", dataPacket.len);
                    break;
                case 0x30: // Position ECEF
                    printf("   Position ECEF %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // LatLon
                    printf("   LatLon %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 0; i < 8; i++) auxBuf[i]=dataPacket.data[i];
                    printf("   Latitud: %2.8lf\n", hexa2double(auxBuf));
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 8; i < 16; i++) auxBuf[i-8]=dataPacket.data[i];
                    printf("   Longitud: %2.8lf\n", hexa2double(auxBuf));
                    break;
                default:
                    printf("   UNKNOWN :: position %d bytes\n", dataPacket.len);
                    break;
            }
            printf("\n");
            break;
            
            
        case 0x80: // Angular velocity
            printf("Got angular velocity packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x20: // Rate of Turn
                    printf("   Rate of Turn %d bytes\n", dataPacket.len);
                    break;
                case 0x30: // Delta Q
                    unsigned char * buf;
                    buf = (unsigned char *) malloc(4);
                    for(int i = 0 ; i < 4; i ++) buf[i] = dataPacket.data[i];
                    printf("   Delta q0: %f\n",hexa2float(buf));
                    for(int i = 4 ; i < 8; i ++) buf[i-4] = dataPacket.data[i];
                    printf("   Delta q1: %f\n",hexa2float(buf));
                    for(int i = 8 ; i < 12; i ++) buf[i-8] = dataPacket.data[i];
                    printf("   Delta q2: %f\n",hexa2float(buf));
                    for(int i = 12 ; i < 16; i ++) buf[i-12] = dataPacket.data[i];
                    printf("   Delta q3: %f\n",hexa2float(buf));
                    break;
                default:
                    printf("   UNKNOWN :: angular velocity %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;
            
            
        case 0xA0: // Sensor component readout
            printf("Got sensor component readout packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // ACC+GYR+MAG+Temperature
                    printf("   ACC+GYR+MAG+Temperature V %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Gyro Temperature
                    printf("   Gyro Temperature %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: SCR %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;
            
            
        case 0xB0: // Analog in
            printf("Got analog in packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Analog in 1
                    printf("   Analog in 1 %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Analog in 2
                    printf("   Analog in 2 %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: Analog in %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;


        case 0xC0: // Magnetic
            printf("Got magnetic packet\n");
            /*if ((dataPacket.idSignal & 0xF0) == 0x20) { // Magnetic field{
                printf("   Magnetic field %d bytes\n", dataPacket.len);
                unsigned char * buf;
                buf = (unsigned char *) malloc(4);
                for (int i = 0; i < 4; i++) buf[i] = dataPacket.data[i];
                printf("   Mag (X): %f\n", hexa2float(buf));
                for (int i = 4; i < 8; i++) buf[i - 4] = dataPacket.data[i];
                printf("   Mag (Y): %f\n", hexa2float(buf));
                for (int i = 8; i < 12; i++) buf[i - 8] = dataPacket.data[i];
                printf("   Mag (Z): %f\n", hexa2float(buf));
            } else
                printf("   UNKNOWN :: Magnetic %d bytes\n", dataPacket.len);*/
            printf("\n");
            break;
            
            
        case 0xD0: // Velocity
            printf("Got velocity packet\n");
            /*if((dataPacket.idSignal & 0xF0) == 0x10) // Velocity XYZ
                printf("   Velocity XYZ %d bytes\n", dataPacket.len);
            else
                printf("   UNKNOWN :: Velocity %d bytes\n", dataPacket.len); */           
            printf("\n");
            break;
            
            
        case 0xE0: // Status
            printf("Got status packet\n");
            /*switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Status Byte
                    printf("   Status Byte %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Status Word
                    unsigned char * buf;
                    buf = (unsigned char *) malloc(4);
                    for(int i = 0 ; i < 4; i ++) buf[i] = dataPacket.data[i];
                    printf("   Status: %d\n",hexa2int(buf));
                    printf("   Status Word %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // RSSI
                    printf("   RSSI %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: Status in %d bytes\n", dataPacket.len);
                    break;
            }*/
            printf("\n");
            break;
            
            
        default: // Unknown
            printf("Got KNOWN packet: %02X\n", dataPacket.idGroup);
            printf("\n");
            break;

    }
}

float hexa2float(unsigned char * buffer)
{
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

int hexa2int(unsigned char * buffer)
{
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

float hexa2double(unsigned char * buffer){
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

float parseFixPointFormat(unsigned char * data) {
    
    unsigned char *buf = (unsigned char *) malloc(2);
    short auxShort;
    int ent;
    buf[0] = data[1];
    buf[1] = data[0];
    memcpy(&auxShort, buf, 2);
    
    
    
    buf = (unsigned char *) malloc(4);
    buf[0] = data[3];
    buf[1] = data[2];
    buf[2] = data[1];
    buf[3] = data[0];
    memcpy(&ent, buf, 2);

    for (int i = 0; i < 6; i++) {
        printf("%02X ",data[i]);
    }
    printf("\n");
    

    return ent/auxShort;
}