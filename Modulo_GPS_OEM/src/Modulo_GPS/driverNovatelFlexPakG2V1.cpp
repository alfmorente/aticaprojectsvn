/***********************************************************************************/
/* driverNovatelFlexPakG2V1.cpp  Félix M. Ballester Goytia    Junio, 2012          */
/*---------------------------------------------------------------------------------*/
/* Comunicacion serie con GPS Novatel FlexPak-G2-V1 - Linux Ubuntu                 */
/*---------------------------------------------------------------------------------*/
/* Envio y tratamiento del comando GPGGARTK. Formato de datos NMEA.                */
/* Envio y tratamiento del comando GPVTG. Formato de datos NMEA.    	           */
/***********************************************************************************/

#include "Modulo_GPS/driverNovatelFlexPakG2V1.h"


using namespace std;

driverNovatelFlexPakG2V1::driverNovatelFlexPakG2V1() {
    // Constructor sin parámetros
}

driverNovatelFlexPakG2V1::driverNovatelFlexPakG2V1(int *res, char *serial_name) {
    /******************************************************************************/
    /* Abre el puerto serie                                                       */
    /*----------------------------------------------------------------------------*/
    /* ENTRADA:                                                                   */
    /*   - serial_name: nombre del dispositivo serie, por ejemplo: "/dev/ttyUSB0" */
    /*   - baud: velocidad del puerto serie. En nuestro caso B9600                */
    /*                                                                            */
    /* DEVUELVE:                                                                  */
    /*   - El descriptor del puerto serie                                         */
    /*   -  0 si todo correcto						     						  */
    /*   - -1 si hay un error						      						  */
    /*   - -2 si no se comunico bien con el dispositivo			      			  */
    /******************************************************************************/

    // Constructor: inicializamos el GPS y lo configuramos para recoger datos a 0.1 Herzios

    char comando[25] = "log gpggartk ontime 10\r\n";
    char comando2[25]="log gpgvtg ontime 10\r\n";
    
    int escrito;

    canal = open(serial_name, O_RDWR | O_NOCTTY);
    if (canal < 0) {
        perror(serial_name);
        *res = -1;
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

        escrito = write(canal, comando, strlen(comando) - 1); // Enviamos el comando para que comience a enviarnos datos
        escrito=write(canal,comando2,strlen(comando2)-1); // Enviamos el comando para que comience a enviarnos datos
        
        if ((escrito < 0) | (escrito < (strlen(comando) - 1))) {
            perror(serial_name);
            *res = -2;
            //exit(-2);
        } else
            *res = canal; // Todo fue bien
    }
    if (*res == canal) {
        cout << "Canal abierto en el: " << canal << endl;
    }
}

driverNovatelFlexPakG2V1::~driverNovatelFlexPakG2V1() {
    /********************************************************************/
    /* Destructor                                                       */
    /********************************************************************/

    apagarGPS();
}

bool driverNovatelFlexPakG2V1::checksumXOR(char* response) {
    /***************************************************************************/
    /* Comprueba el Checksum de la cadena leida                                */
    /*-------------------------------------------------------------------------*/
    /* ENTRADA:                                                                */
    /*   - char: cadena leida desde el dispositivo                             */
    /*                                                                         */
    /* DEVUELVE:                                                               */
    /*   - false: si no se cumple el Cheksum                                   */
    /*   - true: si se cumple el Cheksum						               */
    /***************************************************************************/

    int i = 0, comp1i;
    unsigned char XOR;
    char Buff[77] = "", p, comp1c[3], check[3];

    // Elimino el caracter $ y guardo la cadena
    while (*response != '$')
        response++;
    response++;
    // Elimino el caracter *
    while (*response != '*') {
        p = *response;
        Buff[i] = p;
        i++;
        response++;
    }
    response++;
    check[0] = response[0];
    check[1] = response[1];
    check[2] = 0;

    int iLen = strlen(Buff);

    for (XOR = 0, i = 0; i < iLen; i++)
        XOR ^= (unsigned char) Buff[i];

    comp1i = (int) XOR;
    sprintf(comp1c, "%X", comp1i);
    comp1c[2] = 0;

    if (strcmp(comp1c, check) == 0)
        return true;
    else
        return false;
}

void driverNovatelFlexPakG2V1::apagarGPS() {
    /**************************************************************************/
    /* Para el GPS, restaura la conf del puerto serie y cierra el puerto serie*/
    /*------------------------------------------------------------------------*/
    /*                                                                        */
    /**************************************************************************/

    /* mandamos un comando para que deje de mandar datos cada X tiempo  */
    if(write(canal, "log gpggartk \r\n", strlen("log gpggartk \r\n") - 1)<0)
        cout << "Error de escritura serie en el cierre de la aplicación" << endl;
    if(write(canal, "log gpvtg \r\n", strlen("log gpvtg \r\n") - 1)<0)
        cout << "Error de escritura serie en el cierre de la aplicación" << endl;;

    /* restaura la anterior configuracion del puerto  */
    tcsetattr(canal, TCSANOW, &oldtio);

    close(canal);
}

// Lee hasta obtener un frame completo (de $ a checksum) y devuelve la propia trama
char* driverNovatelFlexPakG2V1::getFrame(){
    char *leido = new char[255];
    char aux;
    int countRead = 0;
    bool endOfFrame = false, headerFound = false;

    // Se encuentra la cabecera
    while (!headerFound) {
        if(read(canal, &aux, 1)<0)
            cout << "Error de lectura en la obtencion de la trama" << endl;
        if (aux == '$') {
            headerFound = true;
        }
    }
    // Se obtiene la trama
    while (!endOfFrame) {
        if(read(canal, &leido[countRead], 1)<0)
            cout << "Error de lectura en la obtencion de la trama" << endl;
        if (leido[countRead] == '*') {
            // 2 lecturas mas
            if(read(canal, &leido[++countRead], 1)<0)
                cout << "Error de lectura en la obtencion de la trama" << endl;
            if(read(canal, &leido[++countRead], 1)<0)
                cout << "Error de lectura en la obtencion de la trama" << endl;
            endOfFrame = true;
            countRead += 2;
        } else {
            countRead++;
        }

    }
    leido[countRead + 1] = 0; /* envio de fin de cadena */
    return leido;
}

char *driverNovatelFlexPakG2V1::getHeader(char* frame){
    return strtok(frame, ",");
}
	

int driverNovatelFlexPakG2V1::setFrequency(int frec) {
    /********************************************************************/
    /* Cambia la frecuencia a la que el GPS envia los datos             */
    /*------------------------------------------------------------------*/
    /* ENTRADA:                                                         */
    /*   canal: descriptor del GPS                                      */
    /*   frec : frecuencia a la que se va a configurar el GPS (int)     */
    /* DEVUELVE:                                                        */
    /*   0: Si la frecuencia es correcta y todo fue bien			    */
    /*  -1: Si la frecuencia es incorrecta							    */
    /********************************************************************/
    char comando[30] = "";
    char comando2[30] = "";

    //	1 Herzio	2 Herzios	5 Herzios	10 Herzios
    //	1 Segundo	0,5 segundos	0,2 Segundos	0,1 Segundos

    if ((frec != 1)&&(frec != 2)&&(frec != 5)&&(frec != 10))
        return (-1);
    else {
        if (frec == 1) {
            strcat(comando2, "log gpvtg ontime 1 \r\n");
            strcat(comando, "log gpggartk ontime 1 \r\n");
        } else if (frec == 2) {
            strcat(comando, "log gpggartk ontime 0.5 \r\n");
            strcat(comando2, "log gpvtg ontime 0.5 \r\n");
        } else if (frec == 5) {
            strcat(comando, "log gpggartk ontime 0.2 \r\n");
            strcat(comando2, "log gpvtg ontime 0.2 \r\n");
        } else if (frec == 10) {
            strcat(comando, "log gpggartk ontime 0.1 \r\n");
            strcat(comando2, "log gpvtg ontime 0.1 \r\n");
        }

        // Escritura serie
        if(write(canal, comando, strlen(comando) - 1)<0)
            cout << "Error de escritura serie en el cambio de frecuencia"; // Enviamos el comando para que comience a enviarnos datos
        /* De momento se comenta este comando por ser inutil de cara a los mensajes a enviar
        if(write(canal, comando2, strlen(comando2) - 1)<0)
            cout << "Error de escritura serie en el cambio de frecuencia";; // Enviamos el comando para que comience a enviarnos datos*/
        return 0;
    } // else
}

bool driverNovatelFlexPakG2V1::GPGGAdata( int *hora, int *minutos, int *segundos, int *decsegundos, float *latitude, char *dirlatitude, float *longitude, char *dirlongitude, int *gpsquality, int *numsatellites, float *precision, float *altitude, char *unit, char checksum[2]) {
    char *ptr, s2[2] = ",";
    char tmp[3];
    int i = 0, countValues=0;
    while ((ptr = strtok(NULL, s2)) != NULL) { // Posteriores llamadas
        countValues++;
            switch (i) {
                case 0: // Time of position
                    tmp[0] = ptr[0];
                    tmp[1] = ptr[1];
                    *hora = atoi(tmp);
                    tmp[0] = ptr[2];
                    tmp[1] = ptr[3];
                    *minutos = atoi(tmp);
                    tmp[0] = ptr[4];
                    tmp[1] = ptr[5];
                    *segundos = atoi(tmp);
                    tmp[0] = ptr[7];
                    tmp[1] = ptr[8];
                    *decsegundos = atoi(tmp);
                    break;
                case 1: // Latitude
                    *latitude = atof(ptr);
                    break;
                case 2: // Latitude direction
                    *dirlatitude = *ptr;
                    break;
                case 3: // Longitude
                    *longitude = atof(ptr);
                    break;
                case 4: // Longitude direction
                    *dirlongitude = *ptr;
                    break;
                case 5: // GPS Quality indicator
                    *gpsquality = atoi(ptr);
                    break;
                case 6: // Number os satellites
                    *numsatellites = atoi(ptr);
                    break;
                case 7: // Horizontal dilution of precision
                    *precision = atof(ptr);
                    break;
                case 8: // Antenna altitude sea level
                    *altitude = atof(ptr);
                    break;
                case 9: // Units od the antenna altitude
                    *unit = *ptr;
                    break;
                case 10:// Cheksum
                    checksum[0] = ptr[1];
                    checksum[1] = ptr[2];
                    break;
                default: 
                    break;
            } // switch
        i++;
    } // while
    
    // Comprueba que la trama no viene vacia
    if(countValues<5)
        return false;
    else
        return true;
}

bool driverNovatelFlexPakG2V1::GPVTGdata(float *track_true, char *t_indicator, float *track_mag, char *m_indicator, float *knot_speed, char *k_indicator, float *km_speed, char *km_indicator, char checksum[2]) {
    char *ptr, s2[2] = ",";
    int countValues=0;
    int i = 0;
    while ((ptr = strtok(NULL, s2)) != NULL) // Posteriores llamadas
    {
        countValues++;
        switch (i) {
            case 0: // Track True
                *track_true = atof(ptr);
                break;
            case 1: // Track Indicator
                *t_indicator = *ptr;
                break;
            case 2: // Track Magnetic
                *track_mag = atof(ptr);
                break;
            case 3: // Track Magnetic Indicator
                *m_indicator = *ptr;
                break;
            case 4: // Knot Speed
                *knot_speed = atof(ptr);
                break;
            case 5: // Knot Indicator
                *k_indicator = *ptr;
                break;
            case 6: // Km Speed
                *km_speed = atof(ptr);
                break;
            case 7: // Km Indicator
                *km_indicator = *ptr;
                break;
            case 8:// Cheksum
                checksum[0] = ptr[1];
                checksum[1] = ptr[2];
                break;
        } // switch
        i++;
    } // while
    
    // Comprueba que la trama no viene vacia
    if(countValues<5)
        return false;
    else
        return true;
}