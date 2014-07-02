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
        
        msg = reqDeviceID();
        sendToDevice(msg);

        msg = setOutPutMode();
        sendToDevice(msg);
        
        msg = setOutPutSettings();
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
    xsMsg.data[1] = 0x3F;
    xsMsg.data[0] = 0x08;


    
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
    xsMsg.data[3] = 0x75;
    xsMsg.data[2] = 0x00;
    xsMsg.data[1] = 0x00;
    xsMsg.data[0] = 0x00;
    
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
                        dataPacketMT2 dataPacket;
                        int i = 0;
                        
                        while(i<xsMsg.len){
                            
                            //unsigned char * aux = (unsigned char *)malloc(2);

                            dataPacket.idPacket = (xsMsg.data[i]*256)+(xsMsg.data[i+1]);
                            
                            i++;
                            i++;
                            dataPacket.len = xsMsg.data[i];
                            i++;
                            
                            dataPacket.data = (unsigned char *) malloc(dataPacket.len);
                            
                            for(int j = 0; j < dataPacket.len;j++){
                                
                                dataPacket.data[j]=xsMsg.data[i];
                                i++;                                
                            }
                            
                            printf("ID Packet found: %d with length %d\n",dataPacket.idPacket,dataPacket.len);
                        }
                        printf("---\n");
                    }
                }
            }
        }
    }

}