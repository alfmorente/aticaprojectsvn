/* 
 * File:   ProtPelcoD.cpp
 * Author: atica
 * 
 * Created on 13 de mayo de 2014, 11:52
 */

#include <Modulo_Camaras/ProtPelcoD.h>

ProtPelcoD::ProtPelcoD(int idCamera, int vel_pan,int vel_tilt) 
{
    serial=new PortSerial();
    this->idCamera=idCamera;
    this->velPAN=vel_pan;
    this->velTILT=vel_tilt;
}


ProtPelcoD::~ProtPelcoD() 
{
    delete serial;
}

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
void ProtPelcoD::commandPAN(int typePAN)
{
    char command1,command2,data1,data2;   
    command1=PELCO_ZERO;
    if(typePAN==CAMERA_PAN_RIGHT)
    {
        command2=PELCO_PAN_RIGHT;
        data1=velPAN;
    }
    else if(typePAN==CAMERA_PAN_LEFT)
    {
        command2=PELCO_PAN_LEFT;
        data1=velPAN;        
    }        
    else //CAMERA_PAN_STOP
    {
        command2=PELCO_ZERO;
        data1=VEL_STOP;
    }  
    data2=PELCO_ZERO;
    sendCommand(command1,command2,data1,data2);
}

void ProtPelcoD::commandTILT(int typeTILT)
{
    char command1,command2,data1,data2;    
    command1=PELCO_ZERO;
    if(typeTILT==CAMERA_TILT_UP)
    {
        command2=PELCO_TILT_UP;
        data2=velTILT;
    }
    else if(typeTILT==CAMERA_TILT_DOWN)
    {
        command2=PELCO_TILT_DOWN;
        data2=velTILT;        
    }        
    else //CAMERA_TILT_STOP
    {
        command2=PELCO_ZERO;
        data2=VEL_STOP;
    }  
    data1=PELCO_ZERO;
    sendCommand(command1,command2,data1,data2);
}

void ProtPelcoD::commandZOOM(int typeZOOM)
{
    char command1,command2,data1,data2;
    command1=PELCO_ZERO;
    if(typeZOOM==CAMERA_ZOOM_IN)
    {
        command2=PELCO_ZOOM_IN;

    }
    else // typeZOOM==CAMERA_ZOOM_OUT
    {
        command2=PELCO_ZOOM_OUT;
    
    }        
    data1=PELCO_ZERO;
    data2=PELCO_ZERO;         
    sendCommand(command1,command2,data1,data2);
}

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

void ProtPelcoD::commandOTHER(char byte1,char byte2,char byte3,char byte4)
{        
    sendCommand(byte1,byte2,byte3,byte4);
}
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

char ProtPelcoD::calcChecksum(char command[])
{
    char checksum;
    for(int i=1;i<6;i++)
        checksum+=command[i];
    checksum=checksum%256;
    return checksum;
    
}

void ProtPelcoD::disconnect()
{
     serial->closeSerial();
     delete serial;
}
