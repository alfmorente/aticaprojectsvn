

#include "CITIUS_Control_Driving/DrivingConnectionManager.h"

using namespace std;

// Rutina de suscripcion a msgcommandVehicle

void fnc_subs_command(CITIUS_Control_Driving::msg_command msg)
{
    char *deviceName = 
    short value = 
}

// Rutina de servicio a nodeStatus

bool fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp)
{
  ROS_INFO("Recibida peticion de servicio de cambio de nodo");
  return true;
}

// Constructor de la clase

DrivingConnectionManager::DrivingConnectionManager(char *serial_name) {
    
    this->channel = open(serial_name, O_RDWR | O_NOCTTY);
    this->portOpened = false;
    
    if (channel < 0) {
        perror(serial_name);
    } else {
        tcgetattr(channel, &oldtio);
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
        tcflush(channel, TCIFLUSH);
        tcsetattr(channel, TCSANOW, &newtio);
        this->portOpened = true;
        
        // Inicializacion de publicadores
        publisher_vehicleInformation = this->nh.advertise<CITIUS_Control_Driving::msg_vehicleInformation>("vehicleInformation", 1000);
        // Inicializacion de suscriptores
        subscriber_command = this->nh.subscribe("command",1000,fnc_subs_command);
        // Inicializacion de servidores
        server_nodeState = this->nh.advertiseService("nodeStateDriving",fcn_serv_nodeStatus);
    }
}

// Configurar la velocidad de transmision serie

void DrivingConnectionManager::setSpeed(int speed){
    newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    tcflush(channel, TCIFLUSH);
    tcsetattr(channel, TCSANOW, &newtio);
}

// Desconexion con Mecanica

void DrivingConnectionManager::disconnect(){
    tcsetattr(channel, TCSANOW, &oldtio);
    close(channel);
}

char *DrivingConnectionManager::createCommand(char* typeOfCommand, short device, int value){
    stringstream ss;
    ss << typeOfCommand << " " << this->obtainDeviceName(device) << " " << this->adjustValue(device,value);
    return (char *)ss.str().c_str();
}

char *DrivingConnectionManager::obtainDeviceName(short deviceID){
    switch(deviceID){
        case ID_BLINKER_RIGHT:
            return (char *)"RIGHT_BLINKER";
            break;
        case ID_BLINKER_LEFT:
            return (char *)"LEFT_BLINKER";
            break;
        case ID_BLINKER_EMERGENCY:
            return (char *)"EMERGENCY_BLINKER";
            break;
        case ID_DIPSP:
            return (char *)"DIPSP";
            break;
        case ID_DIPSS:
            return (char *)"DIPSS";
            break;
        case ID_DIPSR:
            return (char *)"DIPSR";
            break;
        case ID_KLAXON:
            return (char *)"KLAXON";
            break;
        case ID_GEAR:
            return (char *)"GEAR";
            break;
        case ID_THROTTLE:
            return (char *)"THROTTLE";
            break;
        case ID_MOTOR_RPM:
            return (char *)"MOTOR_RPM";
            break;
        case ID_CRUISING_SPEED:
            return (char *)"CRUISING_SPEED";
            break;
        case ID_MOTOR_TEMPERATURE:
            return (char *)"MOTOR_TEMPERATURE";
            break;
        case ID_HANDBRAKE:
            return (char *)"HANDBRAKE";
            break;
        case ID_BRAKE:
            return (char *)"BRAKE";
            break;
        case ID_STEERING:
            return (char *)"STEERING";
            break;
        case ID_ALARMS:
            return (char *)"ALARMS";
            break;
        case MT_BLINKERS:
            return (char *)"MT_BLINKERS";
            break;
        case MT_BRAKE:
            return (char *)"MT_BRAKE";
            break;
        case MT_GEAR:
            return (char *)"MT_GEAR";
            break;
        case MT_HANDBRAKE:
            return (char *)"MT_HANDBRAKE";
            break;
        case MT_LIGHTS:
            return (char *)"MT_LIGHTS";
            break;
        case MT_STEERING:
            return (char *)"MT_STEERING";
            break;
        case MT_THROTTLE:
            return (char *)"MT_THROTTLE";
            break;
        default:
            return (char *)"NO DEVICE FOUND";
            break;
    }
}

int DrivingConnectionManager::adjustValue(short deviceID, int value){
        switch(deviceID){
        case ID_BLINKER_RIGHT:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_BLINKER_LEFT:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_BLINKER_EMERGENCY:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_DIPSP:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_DIPSS:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_DIPSR:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_KLAXON:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_GEAR:
            if(value>MAX_GEAR)
                return 2;
            else if(value<MIN_GEAR)
                return 0;
            else
                return value;
            break;
        case ID_THROTTLE:
            if(value>MAX_PERCENT)
                return 100;
            else if(value<MIN_PERCENT)
                return 0;
            else
                return value;
            break;
        case ID_CRUISING_SPEED:
            if(value>MAX_RPM_SPD)
                return 1000;
            else if(value<MIN_RPM_SPD)
                return 0;
            else
                return value;
            break;
        case ID_HANDBRAKE:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case ID_BRAKE:
            if(value>MAX_PERCENT)
                return 100;
            else if(value<MIN_PERCENT)
                return 0;
            else
                return value;
            break;
        case ID_STEERING:
            if(value>MAX_STEERING)
                return 100;
            else if(value<MIN_STEERING)
                return -100;
            else
                return value;
            break;
        case MT_BLINKERS:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_BRAKE:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_GEAR:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_HANDBRAKE:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_LIGHTS:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_STEERING:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        case MT_THROTTLE:
            if(value>MAX_BOOLEAN)
                return 1;
            else if(value<MIN_BOOLEAN)
                return 0;
            else
                return value;
            break;
        default:
            return value;
            break;
    }
}

bool DrivingConnectionManager::getPortOpened(){
    return this->portOpened;
}
