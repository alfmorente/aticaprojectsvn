

#include "XSensMTi700Driver.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

XSensMTi700Driver::XSensMTi700Driver() {
    canal = -1;
    // Inicio variable recepcion de datos
    posOriInfo.orientationStatus = 0;
    posOriInfo.positionStatus = 0;
    posOriInfo.latitude = 0;
    posOriInfo.longitude = 0;
    posOriInfo.altitude = 0;
    posOriInfo.roll = 0;
    posOriInfo.pitch = 0;
    posOriInfo.yaw = 0;
    posOriInfo.velX = 0;
    posOriInfo.velY = 0;
    posOriInfo.velZ = 0;
    posOriInfo.accX = 0;
    posOriInfo.accY = 0;
    posOriInfo.accZ = 0;
    posOriInfo.rateX = 0;
    posOriInfo.rateY = 0;
    posOriInfo.rateZ = 0;
}

/*******************************************************************************
 * CONEXIÓN DEL DISPOSITIVO
 ******************************************************************************/

bool XSensMTi700Driver::connectToDevice() {
    
    char * serial_name = (char *) "/dev/ttyUSB0";

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

void XSensMTi700Driver::disconnectDevice() {
    close(canal);
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

bool XSensMTi700Driver::getData() {
    XsensMsg xsMsg;
    short rcvdBytes;
    
    if (read(canal, &xsMsg.pre, 1) > 0) {

        // PRE
        if (xsMsg.pre == COMMAND_PRE) {

            rcvdBytes = read(canal, &xsMsg.bid, 1);

            // BID
            if (xsMsg.bid == COMMAND_BID) {
                rcvdBytes = read(canal, &xsMsg.mid, 1);
                // MID
                if (xsMsg.mid == (COMMAND_MID_MTDATA2)) {

                    //LEN
                    rcvdBytes = read(canal, &xsMsg.len, 1);

                    // DATA
                    xsMsg.data = (unsigned char*) malloc(xsMsg.len + 1);
                    for (int i = 0; i < xsMsg.len; i++) {
                        rcvdBytes = read(canal, &xsMsg.data[i], 1);
                    }

                    // CS
                    rcvdBytes = read(canal, &xsMsg.cs, 1);

                    if (!isCheckSumOK(xsMsg)) printf("ERROR in MTData2 ACK reception\n");

                    dataPacketMT2 dataPacket;
                    int i = 0;

                    while (i < xsMsg.len) {

                        //unsigned char * aux = (unsigned char *)malloc(2);

                        dataPacket.idGroup = xsMsg.data[i++];
                        dataPacket.idSignal = xsMsg.data[i++];
                        dataPacket.len = xsMsg.data[i++];

                        dataPacket.data = (unsigned char *) malloc(dataPacket.len);

                        for (int j = 0; j < dataPacket.len; j++) {

                            dataPacket.data[j] = xsMsg.data[i++];
                        }
                        packetMng(dataPacket);
                        return true;

                    }
                }
            }
        }
    }
    return false;
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
    xsMsg.data = (unsigned char *) malloc(xsMsg.len);
    // Velocidad angular Rate turn
    xsMsg.data[0] = 0x80;
    xsMsg.data[1] = 0x20;
    xsMsg.data[2] = 0x00;
    xsMsg.data[3] = FREC_REQ_DATA;
    // Orientacion
    xsMsg.data[4] = 0x20;
    xsMsg.data[5] = 0x30;
    xsMsg.data[6] = 0x00;
    xsMsg.data[7] = FREC_REQ_DATA;
    // Posicion (Lat + Lon)
    xsMsg.data[8] = 0x50;
    xsMsg.data[9] = 0x43;
    xsMsg.data[10] = 0x00;
    xsMsg.data[11] = FREC_REQ_DATA;
    // Posicion (Alt) MLS
    xsMsg.data[12] = 0x50;
    xsMsg.data[13] = 0x10;
    xsMsg.data[14] = 0x00;
    xsMsg.data[15] = FREC_REQ_DATA;
    // Aceleracion
    xsMsg.data[16] = 0x40;
    xsMsg.data[17] = 0x20;
    xsMsg.data[18] = 0x00;
    xsMsg.data[19] = FREC_REQ_DATA;
    // Velocity
    xsMsg.data[20] = 0xD0;
    xsMsg.data[21] = 0x10;
    xsMsg.data[22] = 0x00;
    xsMsg.data[23] = FREC_REQ_DATA;

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
    short writtenBytes = write(canal, msg2send, msg.len + 5); // Enviamos el comando para que comience a enviarnos datos
    waitForAck(msg.mid);

}

void XSensMTi700Driver::waitForAck(unsigned char _mid) {

    bool ackFound = false;
    XsensMsg xsMsg;
    short rcvdBytes;

    while (!ackFound) {

        if (read(canal, &xsMsg.pre, 1) > 0) {

            // PRE
            if (xsMsg.pre == COMMAND_PRE) {
                
                rcvdBytes = read(canal, &xsMsg.bid, 1);

                // BID
                if (xsMsg.bid == COMMAND_BID) {
                    
                    rcvdBytes = read(canal, &xsMsg.mid, 1);
                    
                    // MID
                    switch (xsMsg.mid) {
                        case (COMMAND_MID_GOTOCONFIG + 1):

                            //LEN
                            rcvdBytes = read(canal, &xsMsg.len, 1);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            for (int i = 0; i < xsMsg.len; i++) {
                                rcvdBytes = read(canal, &xsMsg.data[i], 1);
                            }                            
                            // CS
                            rcvdBytes = read(canal, &xsMsg.cs, 1);
                            
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                            }else{
                                printf("ERROR in GotoConfig ACK reception\n");
                            }
                            break;
                        
                        case (COMMAND_MID_GOTOMEASUREMENT + 1):
                            // LEN
                            rcvdBytes = read(canal, &xsMsg.len, 1);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            for (int i = 0; i < xsMsg.len; i++) {
                                rcvdBytes = read(canal, &xsMsg.data[i], 1);
                            }
                            
                            // CS
                            rcvdBytes = read(canal, &xsMsg.cs, 1);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                            }else{
                                printf("ERROR in GoToMeasurement ACK reception\n");
                            }
                            break;
                        
                        case (COMMAND_MID_SETOUTPUTCONFIGURATION + 1):
                            // LEN
                            rcvdBytes = read(canal, &xsMsg.len, 1);
                            
                            // DATA
                            xsMsg.data = (unsigned char*) malloc(xsMsg.len+1);
                            for (int i = 0; i < xsMsg.len; i++) {
                                rcvdBytes = read(canal, &xsMsg.data[i], 1);
                            }
                            
                            // CS
                            rcvdBytes = read(canal, &xsMsg.cs, 1);
                            if (isCheckSumOK(xsMsg)) {
                                ackFound = true;
                            }else{
                                printf("ERROR in SetOutPutConfiguration ACK reception\n");
                            }
                            break;
                    }
                }
            }
        }
    }
}

/*******************************************************************************
 * FUNCION DE TRATAMIENTO DE DATOS 
 ******************************************************************************/

void XSensMTi700Driver::packetMng(dataPacketMT2 dataPacket) {
    //printf("ID Packet found:Group %02X Signal %02X with length %d\n",dataPacket.idGroup,dataPacket.idSignal,dataPacket.len);
    unsigned char *auxBuf;

    //rintf("\n");
    switch (dataPacket.idGroup) {
        
        case 0x10: // Timestamp
            /*printf("Got timestamp packet\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // UTC Time
                    printf("   UTC time %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Packet counter
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
            //printf("\n");
            break;
            
        case 0x08: // Temperature
            /*printf("Got temperature packet\n");
            if ((dataPacket.idSignal & 0xF0) == 0x10) { // Temperature
                printf("TEMPERATURE: %lf ºC\n", hexa2double(auxBuf));
            } else
                printf("   UNKNOWN :: Temperature %d bytes\n", dataPacket.len);
            //printf("\n");*/
            break;
            
        case 0x88: // GPS
            /*printf("Got GPS packet\n");
            switch (dataPacket.idSignal & 0xF0) {
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
            
            //printf("\n");
            break;
            
        case 0x20: // Orientation
            //printf("Got orientation packet\n");
            //printf("ORIENTATION:\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Quaternion
                    //printf("   Quaternion %d bytes\n", dataPacket.len);
                    break;
                case 0x20: // Rotation Matrix
                    //printf("   Rotation Matrix %d bytes\n", dataPacket.len);
                    break;
                case 0x30: // Euler Angles
                    //printf("   Euler Angles %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 0; i < 8; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Roll: %3.8lf º\n", hexa2double(auxBuf));
                    posOriInfo.roll = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 8; i < 16; i++) auxBuf[i-8]=dataPacket.data[i];
                    //printf("   Pitch: %3.8lf º\n", hexa2double(auxBuf));
                    posOriInfo.pitch = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 16; i < 24; i++) auxBuf[i-16]=dataPacket.data[i];
                    //printf("   Yaw: %3.8lf º\n", hexa2double(auxBuf));
                    posOriInfo.yaw = hexa2float(auxBuf);
                    
                    break;
                default:
                    printf("   UNKNOWN :: Orientation %d bytes\n", dataPacket.len);
                    break;
            }
            
            //printf("\n");
            break;
            
        case 0x30: // Pressure
            /*printf("Got pressure packet\n");
            if((dataPacket.idSignal & 0xF0) == 0x10) // Pressure
                printf("   Pressure: %f mbar\n",  (float) hexa2int(dataPacket.data) / 100);
            else
                printf("   UNKNOWN :: Pressure %d bytes \n", dataPacket.len);*/
            //printf("\n");
            break;
            
        case 0x40: // Acceleration
            //printf("Got acceleration packet\n");
           // printf("ACCELERATION:\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Delta V
                    //printf("   Delta V %d bytes\n", dataPacket.len);
                case 0x20: // Acceleration
                    //printf("   Acceleration %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 0; i < 4; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Acc X: %f m/s2\n", hexa2float(auxBuf));
                    posOriInfo.accX = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 4; i < 8; i++) auxBuf[i-4]=dataPacket.data[i];
                    //printf("   Acc Y: %f m/s2\n", hexa2float(auxBuf));
                    posOriInfo.accY = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 8; i < 12; i++) auxBuf[i-8]=dataPacket.data[i];
                    //printf("   Acc Z: %f m/s2", hexa2float(auxBuf));
                    posOriInfo.accZ = hexa2float(auxBuf);
                    
                    break;
                case 0x30: // Free acceleration
                    //printf("   Free acceleration %d bytes\n", dataPacket.len);
                    break;
                default:
                    //printf("   UNKNOWN :: Acceleration %d bytes\n", dataPacket.len);
                    break;
            }
            //printf("\n");
            break;
            
        case 0x50: // Position
            //printf("Got position packet\n");
            //printf("POSITION:\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Altitude MSL
                    //printf("   Altitude MSL %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 0; i < 4; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Altitude: %f m\n", hexa2float(auxBuf));
                    posOriInfo.altitude = hexa2float(auxBuf);
                    
                    break;
                case 0x20: // Altitude Ellipsoid
                    //printf("   Altitude Ellipsoid %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 0; i < 4; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Altitude: %f m\n", hexa2float(auxBuf));
                    posOriInfo.altitude = hexa2float(auxBuf);
                    
                    break;
                case 0x30: // Position ECEF
                    //printf("   Position ECEF %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // LatLon
                    //printf("   LatLon %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 0; i < 8; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Latitude: %2.10lf ºC N\n", hexa2double(auxBuf));
                    posOriInfo.latitude = hexa2double(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(8);
                    for(int i = 8; i < 16; i++) auxBuf[i-8]=dataPacket.data[i];
                    //printf("   Longitude: %2.10lf ºC W\n", hexa2double(auxBuf));
                    posOriInfo.longitude = hexa2double(auxBuf);
                    
                    break;
                default:
                    printf("   UNKNOWN :: position %d bytes\n", dataPacket.len);
                    break;
            }
            //printf("\n");
            break;
            
        case 0x80: // Angular velocity
            //printf("Got angular velocity packet\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x20: // Rate of Turn
                    //printf("   Rate of Turn %d bytes\n", dataPacket.len);
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 0; i < 4; i++) auxBuf[i]=dataPacket.data[i];
                    //printf("   Acc X: %f m/s2\n", hexa2float(auxBuf));
                    posOriInfo.rateX = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 4; i < 8; i++) auxBuf[i-4]=dataPacket.data[i];
                    //printf("   Acc Y: %f m/s2\n", hexa2float(auxBuf));
                    posOriInfo.rateY = hexa2float(auxBuf);
                    
                    auxBuf = (unsigned char *)malloc(4);
                    for(int i = 8; i < 12; i++) auxBuf[i-8]=dataPacket.data[i];
                    //printf("   Acc Z: %f m/s2", hexa2float(auxBuf));
                    posOriInfo.rateZ = hexa2float(auxBuf);
                    
                    break;
                case 0x30: // Delta Q
                    //printf("   Delta Q %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: angular velocity %d bytes\n", dataPacket.len);
                    break;
            }
            //printf("\n");
            break;
            
        case 0xA0: // Sensor component readout
            /*printf("Got sensor component readout packet\n");
            switch (dataPacket.idSignal & 0xF0) {
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
            //printf("\n");
            break;
            
        case 0xB0: // Analog in
            /*printf("Got analog in packet\n");
            switch (dataPacket.idSignal & 0xF0) {
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
            //printf("\n");
            break;

        case 0xC0: // Magnetic
            /*printf("Got magnetic packet\n");
            if ((dataPacket.idSignal & 0xF0) == 0x20) { // Magnetic field{
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
            //printf("\n");
            break;
            
        case 0xD0: // Velocity
            //printf("Got velocity packet\n");
            if((dataPacket.idSignal & 0xF0) == 0x10){ // Velocity XYZ
                //printf("   Velocity XYZ %d bytes\n", dataPacket.len);
                auxBuf = (unsigned char *) malloc(4);
                for (int i = 0; i < 4; i++) auxBuf[i] = dataPacket.data[i];
                //printf("   Vel X: %f m/s\n", hexa2float(auxBuf));
                posOriInfo.velX = hexa2float(auxBuf);
                
                auxBuf = (unsigned char *) malloc(4);
                for (int i = 4; i < 8; i++) auxBuf[i - 4] = dataPacket.data[i];
                //printf("   Vel Y: %f m/s\n", hexa2float(auxBuf));
                posOriInfo.velY = hexa2float(auxBuf);
                
                auxBuf = (unsigned char *) malloc(4);
                for (int i = 8; i < 12; i++) auxBuf[i - 8] = dataPacket.data[i];
                //printf("   Vel Z: %f m/s", hexa2float(auxBuf));
                posOriInfo.velZ = hexa2float(auxBuf);
                
            }else
                printf("   UNKNOWN :: Velocity %d bytes\n", dataPacket.len);     
            //printf("\n");
            break;
            
        case 0xE0: // Status
            /*printf("Got status packet\n");
            switch (dataPacket.idSignal & 0xF0) {
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
            //printf("\n");
            break;
            
            
        default: // Unknown
            printf("Got KNOWN packet: %02X\n", dataPacket.idGroup);
            //printf("\n");
            break;

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

/*******************************************************************************
 * GET DE DATOS OBTENIDOS
 ******************************************************************************/

GPSINSInfo XSensMTi700Driver::getInfo(){ return posOriInfo; }