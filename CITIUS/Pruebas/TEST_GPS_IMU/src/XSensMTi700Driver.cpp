

#include "XSensMTi700Driver.h"

using namespace std;

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
    * DESTRUCTOR DE LA CLASE
 ******************************************************************************/

XSensMTi700Driver::~XSensMTi700Driver() {
    
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
        
        // Cambio de modo configuracion para no sobrecargar el buffer
        usleep(100);
        sendToDevice(goToConfig());
        
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
    
    // Configuracion de dispositivo
    sendToDevice(setOutPutConfiguration());
    
    
    
}

/*******************************************************************************
 * RECOPILACION DE DATOS
 ******************************************************************************/

bool XSensMTi700Driver::getData() {
    /*printf("Vamos a ver que pasa aqui:\n");
    char byte;
    while(true){
        if(read(this->canal,&byte,1)>0)
            printf("%02X ",byte);
    }*/
    
    // Modo stream de medidas
    sendToDevice(goToMeasurement());
    
    vector<unsigned char> frame2;
    
    unsigned char byte, len;
    
    bool dataFound = false;
    
    while (!dataFound){
        
        // HEADER (PRE + BID + MID)
        if(read(canal,&byte,1)>0){
            frame2.push_back(byte);
        }
        
        if(frame2.size() == 3){
            
            // LEN
            if(read(canal,&len,1)>0){

                frame2.push_back(len);
                // DATA + CS
                while (frame2.size() < len + 5) {
                    if (read(canal, &byte, 1) > 0) {
                        frame2.push_back(byte);
                    }
                }

                if (frameMng(frame2)) {
                    dataFound = true;
                }else{
                    frame2.clear();
                    return false;
                }
            }
        }
    }
    
    dataFound = false;
    frame2.clear();
    // Segunda lectura para no coger la primera trama que suele estar corrupta
    while (!dataFound){
        
        // HEADER (PRE + BID + MID)
        if(read(canal,&byte,1)>0){
            frame2.push_back(byte);
        }
        
        if(frame2.size() == 3){
            
            // LEN
            if(read(canal,&len,1)>0){

                frame2.push_back(len);
                // DATA + CS
                while (frame2.size() < len + 5) {
                    if (read(canal, &byte, 1) > 0) {
                        frame2.push_back(byte);
                    }
                }

                if (frameMng(frame2)) {
                    dataFound = true;
                }else{
                    frame2.clear();
                    return false;
                }
            }
        }
    }
    sendToDevice(goToConfig());
    return true;
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
    xsMsg.len = 0x1C;
    // Velocidad angular Rate turn
    xsMsg.data.push_back(0x80);
    xsMsg.data.push_back(0x20);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Orientacion
    xsMsg.data.push_back(0x20);
    xsMsg.data.push_back(0x30);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Posicion (Lat + Lon)
    xsMsg.data.push_back(0x50);
    xsMsg.data.push_back(0x43);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Posicion (Alt) MLS
    xsMsg.data.push_back(0x50);
    xsMsg.data.push_back(0x10);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Aceleracion
    xsMsg.data.push_back(0x40);
    xsMsg.data.push_back(0x20);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Velocity
    xsMsg.data.push_back(0xD0);
    xsMsg.data.push_back(0x10);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);
    // Status
    xsMsg.data.push_back(0xE0);
    xsMsg.data.push_back(0x10);
    xsMsg.data.push_back(0x00);
    xsMsg.data.push_back(FREC_REQ_DATA);

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

bool XSensMTi700Driver::isCheckSumOK(vector<unsigned char> frame) {
    unsigned char cs = 0;

    for(unsigned int i = 1; i < frame.size(); i++){
        cs+=frame[i];
    }

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

    if(write(canal, msg2send, msg.len + 5)>0){
        waitForAck(msg.mid);
    }else{
        printf("Error en escritura");
        exit(-1);
    }
    free(msg2send);   

}

void XSensMTi700Driver::waitForAck(unsigned char _mid) {
    
        
    vector<unsigned char> frame2;
    
    unsigned char byte, len;
    
    bool ackFound = false;
    
    while (!ackFound){
        
        // HEADER (PRE + BID + MID)
        if(read(canal,&byte,1)>0){
            frame2.push_back(byte);
        }
        
        if(frame2.size() == 3){
            
            // LEN
            if(read(canal,&len,1)>0){

                frame2.push_back(len);
                // DATA + CS
                while (frame2.size() < len + 5) {
                    if (read(canal, &byte, 1) > 0) {
                        frame2.push_back(byte);
                    }
                }

                if (isCheckSumOK(frame2) && (_mid + 1 == frame2[2])) {
                    ackFound = true;
                }else{
                    frame2.clear();
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
    vector<unsigned char> auxBuf;

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
                    for(int i = 0; i < 12; i++) auxBuf.push_back(dataPacket.data[i]);
                    
                    posOriInfo.roll = hexa2float(vector<unsigned char>(auxBuf.begin(), auxBuf.begin() + 4));
                    posOriInfo.pitch = hexa2float(vector<unsigned char>(auxBuf.begin() + 4, auxBuf.begin() + 8));
                    posOriInfo.yaw = hexa2float(vector<unsigned char>(auxBuf.begin() + 8,auxBuf.begin() + 12));
                                        
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
                    for(int i = 0; i < 12; i++) auxBuf.push_back(dataPacket.data[i]);
                    
                    posOriInfo.accX = hexa2float(vector<unsigned char>(auxBuf.begin(),auxBuf. begin() + 4));
                    posOriInfo.accY = hexa2float(vector<unsigned char>(auxBuf.begin() + 4 ,auxBuf.begin() + 8));
                    posOriInfo.accZ = hexa2float(vector<unsigned char>(auxBuf.begin() + 8, auxBuf.begin() + 12));
                    
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

                    for(int i = 0; i < 4; i++) auxBuf.push_back(dataPacket.data[i]);

                    posOriInfo.altitude = hexa2float(auxBuf);
                    
                    break;
                case 0x20: // Altitude Ellipsoid
                    //printf("   Altitude Ellipsoid %d bytes\n", dataPacket.len);
                    for(int i = 0; i < 4; i++) auxBuf.push_back(dataPacket.data[i]);

                    posOriInfo.altitude = hexa2float(auxBuf);
                    
                    break;
                case 0x30: // Position ECEF
                    //printf("   Position ECEF %d bytes\n", dataPacket.len);
                    break;
                case 0x40: // LatLon
                    //printf("   LatLon %d bytes\n", dataPacket.len);

                    for(int i = 0; i < 16; i++) auxBuf.push_back(dataPacket.data[i]);
                    
                    posOriInfo.latitude = hexa2double(vector<unsigned char>(auxBuf.begin(),auxBuf.begin() + 8));
                    posOriInfo.longitude = hexa2double(vector<unsigned char>(auxBuf.begin() + 8,auxBuf.begin() + 16));
                    
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
                    for(int i = 0; i < 12; i++) auxBuf.push_back(dataPacket.data[i]);
                    
                    posOriInfo.rateX = hexa2float(vector<unsigned char>(auxBuf.begin(),auxBuf. begin() + 4));
                    posOriInfo.rateY = hexa2float(vector<unsigned char>(auxBuf.begin() + 4 ,auxBuf.begin() + 8));
                    posOriInfo.rateZ = hexa2float(vector<unsigned char>(auxBuf.begin() + 8, auxBuf.begin() + 12));
                    
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
                for(int i = 0; i < 12; i++) auxBuf.push_back(dataPacket.data[i]);
                    
                    posOriInfo.velX = hexa2float(vector<unsigned char>(auxBuf.begin(),auxBuf. begin() + 4));
                    posOriInfo.velY = hexa2float(vector<unsigned char>(auxBuf.begin() + 4 ,auxBuf.begin() + 8));
                    posOriInfo.velZ = hexa2float(vector<unsigned char>(auxBuf.begin() + 8, auxBuf.begin() + 12));
                
            }else
                printf("   UNKNOWN :: Velocity %d bytes\n", dataPacket.len);     
            //printf("\n");
            break;
            
        case 0xE0: // Status
            //printf("Got status packet\n");
            switch (dataPacket.idSignal & 0xF0) {
                case 0x10: // Status Byte
                    //printf("   Status Byte %d bytes\n", dataPacket.len);
                    if((dataPacket.data[0] & 0x04) == 0x04){
                        posOriInfo.positionStatus = 1;
                    }else{
                        posOriInfo.positionStatus = 0;
                    }
                    break;
                case 0x20: // Status Word
                    /*unsigned char * buf;
                    buf = (unsigned char *) malloc(4);
                    for(int i = 0 ; i < 4; i ++) buf[i] = dataPacket.data[i];
                    printf("   Status: %d\n",hexa2int(buf));
                    printf("   Status Word %d bytes\n", dataPacket.len);
                    break;*/
                case 0x40: // RSSI
                    printf("   RSSI %d bytes\n", dataPacket.len);
                    break;
                default:
                    printf("   UNKNOWN :: Status in %d bytes\n", dataPacket.len);
                    break;
            }
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

float XSensMTi700Driver::hexa2float(vector<unsigned char> buffer){
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

int XSensMTi700Driver::hexa2int(vector<unsigned char> buffer){
    union
    {
        int value;
        unsigned char buffer[4];

    }intUnion;

    intUnion.buffer[0] = buffer[3];
    intUnion.buffer[1] = buffer[2];
    intUnion.buffer[2] = buffer[1];
    intUnion.buffer[3] = buffer[0];

    return intUnion.value;
}

double XSensMTi700Driver::hexa2double(vector<unsigned char> buffer){
    union{
        double value;
        unsigned char buffer[8];
    }doubleUnion;

    doubleUnion.buffer[0] = buffer[7];
    doubleUnion.buffer[1] = buffer[6];
    doubleUnion.buffer[2] = buffer[5];
    doubleUnion.buffer[3] = buffer[4];
    doubleUnion.buffer[4] = buffer[3];
    doubleUnion.buffer[5] = buffer[2];
    doubleUnion.buffer[6] = buffer[1];
    doubleUnion.buffer[7] = buffer[0];

    return doubleUnion.value;
}

/*******************************************************************************
 * GET DE DATOS OBTENIDOS
 ******************************************************************************/

GPSINSInfo XSensMTi700Driver::getInfo(){ return posOriInfo; }

bool XSensMTi700Driver::frameMng(vector<unsigned char> frame) {
    if (frame[0] == COMMAND_PRE) {
        if (frame[1] == COMMAND_BID) {
            if (frame[2] == COMMAND_MID_MTDATA2) {
                if (isCheckSumOK(frame)) {
                    
                    unsigned int index = 4;
                    while (index < frame.size() - 1) {
                        dataPacketMT2 pMT2;
                        pMT2.idGroup = frame[index++];
                        pMT2.idSignal = frame[index++];
                        pMT2.len = frame[index++];
                        
                        for (int i = 0; i < pMT2.len; i++) {
                            pMT2.data.push_back(frame[index++]);
                        }
                        packetMng(pMT2);
                    }
                    return true;
                } else {
                    printf("ERROR. Frame discarted\n");
                }
            }
        }
    }
    return false;
}