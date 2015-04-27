/**
 * @file   ProtPelcoD.cpp
 * @brief  Fichero fuente de gestion del protocolo PELCOD
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include <Modulo_Camaras/ProtPelcoD.h>

/**
 * Constructor de la clase
 * @param[in] idCamera Identificador de la camara a usar
 * @param[in] vel_pan Velocidad del PAN en caso de utilizarlo
 * @param[in] vel_tilt Velocidad del TILT en caso de utilizarlo
 */
ProtPelcoD::ProtPelcoD(int idCamera, int vel_pan,int vel_tilt) 
{
    serial=new PortSerial();
    this->idCamera=idCamera;
    this->velPAN=vel_pan;
    this->velTILT=vel_tilt;
}

/**
 * Destructor de la clase
 */
ProtPelcoD::~ProtPelcoD() 
{
    delete serial;
}

/**
 * Metodo que conecta con la camara 
 * @param[in] namePort Nombre del puerto con el que se conecta la camara
 * @param[in] velocity Velocidad de la conexion
 * @return 
 */
bool ProtPelcoD::connect(char* namePort,int velocity)
{
    if(serial->openSerial(namePort))
    {
        serial->configura(velocity);
        return true;
    }
    else
        return false;
    
}

/**
 * Metodo que hace PAN a la camara
 * @param[in] typePAN indica la direccion del PAN
 */
void ProtPelcoD::commandPAN(int typePAN)
{
    char command1,command2,data1,data2;   
    command1=PELCO_ZERO;
    if(typePAN==CAMERA_PAN_RIGHT)
    {
	printf("PAN RIGHT\n");
        command2=PELCO_PAN_RIGHT;
        data1=velPAN;
    }
    else if(typePAN==CAMERA_PAN_LEFT)
    {

	printf("PAN LEFT\n");
        command2=PELCO_PAN_LEFT;
        data1=velPAN;        
    }        
    else //CAMERA_PAN_STOP
    {
	printf("PAN STOP\n");
        command2=PELCO_ZERO;
        data1=VEL_STOP;
    }  
    data2=PELCO_ZERO;
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que hace TILT a la camara
 * @param typeTILT Indica la direccion del TILT
 */
void ProtPelcoD::commandTILT(int typeTILT)
{
    char command1,command2,data1,data2;    
    command1=PELCO_ZERO;
    if(typeTILT==CAMERA_TILT_UP)
    {
	printf("TILT UP\n");
        command2=PELCO_TILT_UP;
        data2=velTILT;
    }
    else if(typeTILT==CAMERA_TILT_DOWN)
    {
	printf("TILT DOWN\n");
        command2=PELCO_TILT_DOWN;
        data2=velTILT;        
    }        
    else //CAMERA_TILT_STOP
    {
	printf("TILT STOP\n");
        command2=PELCO_ZERO;
        data2=VEL_STOP;
    }  
    data1=PELCO_ZERO;
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que hace ZOOM a la camara
 * @param[in] typeZOOM Indica la el tipo de ZOOM 
 */
void ProtPelcoD::commandZOOM(int typeZOOM)
{
    char command1,command2,data1,data2;
    command1=PELCO_ZERO;
    if(typeZOOM==CAMERA_ZOOM_IN)
    {
		printf("ZOOM IN\n");
        command2=PELCO_ZOOM_IN;

    }
    else if(typeZOOM==CAMERA_ZOOM_OUT)
    {
		printf("ZOOM OUT\n");
        command2=PELCO_ZOOM_OUT;
    
    }        
    else //typeZOOM==CAMERA_ZOOM_STOP
    {
		printf("ZOOM STOP\n");
        command1=PELCO_ZERO;        
        command2=PELCO_ZERO;
    
    }     
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que actua sobre el iris de la camara
 * @param[in] typeIRIS Indica si se cierra o se abre el iris
 */
void ProtPelcoD::commandIRIS(int typeIRIS)
{
    char command1,command2,data1,data2;

    if(typeIRIS==CAMERA_IRIS_OPEN)
    {
        command1=PELCO_IRIS_OPEN;

    }
    else // typeIRIS==CAMERA_IRIS_CLOSE
    {
        command1=PELCO_IRIS_OPEN;
    
    }   
    command2=PELCO_ZERO;     
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que actua sobre el focus de la camara
 * @param[in] typeFOCUS Indica si se acerca o aleja el FOCUS
 */
void ProtPelcoD::commandFOCUS(int typeFOCUS)
{
    char command1,command2,data1,data2;
    if(typeFOCUS==CAMERA_FOCUS_FAR)
    {
    	command1=PELCO_ZERO;
        command2=PELCO_FOCUS_FAR;

    }
    else // typeFOCUS==CAMERA_FOCUS_NEAR
    {
    	command1=PELCO_FOCUS_NEAR;
        command2=PELCO_ZERO;
    
    }        
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que actua sobre el modo de funcionamiento de la camara
 * @param[in] typeAUTO Indica el modo de funcionamiento (Manual/Scan)
 */
void ProtPelcoD::commandAUTO(int typeAUTO)
{
    char command1,command2,data1,data2;
    if(typeAUTO==CAMERA_AUTO_MANUAL)
    {
    	command1=PELCO_AUTO_MANUAL;


    }
    else // typeAUTO==CAMERA_AUTO_SCAN
    {
    	command1=PELCO_AUTO_SCAN;
    }   
    command2=PELCO_ZERO;     
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}

/**
 * Metodo que actua sobre el encendido de la camara
 * @param[in] typePOWER Indica si se enciende o no la camara
 */
void ProtPelcoD::commandPOWER(int typePOWER)
{
    char command1,command2,data1,data2;
    if(typePOWER==CAMERA_ON)
    {
    	command1=PELCO_CAMERA_ON;


    }
    else // typePOWER==CAMERA_OFF
    {
    	command1=PELCO_CAMERA_OFF;
    }   
    command2=PELCO_ZERO;     
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}
/**
 * Metodo para enviar otros comandos a la camara
 * @param[in] byte1 Indica el valor del byte 1 del protocolo
 * @param[in] byte2 Indica el valor del byte 2 del protocolo
 * @param[in] byte3 Indica el valor del byte 3 del protocolo
 * @param[in] byte4 Indica el valor del byte 4 del protocolo
 */
void ProtPelcoD::commandOTHER(char byte1,char byte2,char byte3,char byte4)
{        
    sendCommand(byte1,byte2,byte3,byte4);
}

/**
 * Metodo que envia una trama PELCOD a la camara
 * @param[in] command1 Byte 1 del protocolo
 * @param[in] command2 Byte 2 del protocolo
 * @param[in] data1 Byte 3 del protocolo
 * @param[in] data2 Byte 4 del protocolo
 * @return 
 */
bool ProtPelcoD::sendCommand(char command1,char command2,char data1,char data2)
{
    
    char commandCamera[7];
    commandCamera[0]=PELCO_SYNC;
    commandCamera[1]=idCamera;
    commandCamera[2]=command1;
    commandCamera[3]=command2;
    commandCamera[4]=data1;
    commandCamera[5]=data2;
    commandCamera[6]=calcChecksum(commandCamera);
    
    printf("El comando a enviar es: ");
    for(int i=0;i<7;i++)
           printf("%02x ",commandCamera[i] & 0xff);
    printf("\n");
    if(serial->send(commandCamera,7))
        return true;
    else 
        return false;
}

/**
 * Metodo que calcula en ckecksum de la trama PELCOD
 * @param command Trama PELCOD sin el checksum
 * @return 
 */
char ProtPelcoD::calcChecksum(char command[])
{
    char checksum=0;
    for(int i=1;i<6;i++)
        checksum+=command[i];
    checksum=checksum%256;
    return checksum;
    
}
/**
 * Metodo que desconecta de la camara
 */
void ProtPelcoD::disconnect()
{
     serial->closeSerial();
     delete serial;
}
