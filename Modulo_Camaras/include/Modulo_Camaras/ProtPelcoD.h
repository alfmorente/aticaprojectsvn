/**
 * @file   ProtPelcoD.h
 * @brief  Fichero de cabecera de gestion del protocolo PELCO
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup Camera
 * @{
 */

#ifndef PROTPELCOD_H
#define	PROTPELCOD_H
#include "../../../Common_files/include/Common_files/constant.h"
#include <Modulo_Camaras/PortSerial.h>
#define CAMERA_IRIS_OPEN 1 ///<Constante ROS que indica apertura del iris de la camara
#define CAMERA_IRIS_CLOSE 0 ///<Constante ROS que indica el cierre del iris de la camara
#define CAMERA_FOCUS_FAR 1 ///<Constante ROS que indica focus de la camara lejos 
#define CAMERA_FOCUS_NEAR 0///<Constante ROS que indica focus de la camara cerca 
#define CAMERA_AUTO_MANUAL 0 ///<Constante ROS que indica camara en manual 
#define CAMERA_AUTO_SCAN 1 ///<Constante ROS que indica camara en automatico
#define CAMERA_OFF 0 ///<Constante ROS que indica camara apagada
#define CAMERA_ON 1 ///<Constante ROS que indica camara encendida

#define VEL_PAN 0x20 ///<Constante que indica velocidad del PAN
#define VEL_TILT 0x20 ///<Constante que indica velocidad del TILT 
#define VEL_STOP 0X00 //</Constante que indica velocidad cero

#define PELCO_SYNC 0xFF ///<Constante PELCO que indica bit de sincronizacion
#define PELCO_ZERO 0x00 ///<Constante PELCO que indica bit a cero 
#define PELCO_PAN_RIGHT 0x02 ///<Constante PELCO que indica hacer PAN a la derecha
#define PELCO_PAN_LEFT 0x04 ///<Constante PELCO que indica hacer PAN a la izquierda 
#define PELCO_TILT_UP 0x08 ///<Constante PELCO que indica hacer TILT hacia arriba
#define PELCO_TILT_DOWN 0x10 ///<Constante PELCO que indica hacer TILT hacia abajo
#define PELCO_ZOOM_IN 0x40 ///<Constante PELCO que indica disminuir ZOOM
#define PELCO_ZOOM_OUT 0x80 ///<Constante PELCO que indica aumentar ZOOM
#define PELCO_IRIS_OPEN 0x02 ///<Constante PELCO que indica abrir IRIS
#define PELCO_IRIS_CLOSE 0x04 ///<Constante PELCO que indica cerrar IRIS
#define PELCO_FOCUS_NEAR 0x01 ///<Constante PELCO que indica FOCUS cerca
#define PELCO_FOCUS_FAR 0x80 ///<Constante PELCO que indica FOCUS lejos
#define PELCO_AUTO_MANUAL 0x10 ///<Constante PELCO que indica camara en MANUAL
#define PELCO_AUTO_SCAN 0x90 ///<Constante PELCO que indica camara en SCAN
#define PELCO_CAMERA_ON 0x88 ///<Constante PELCO que indica camara ON
#define PELCO_CAMERA_OFF 0x08 ///<Constante PELCO que indica camara OFF

/**
 * \struct CameraConfig
 * \brief  Estructura con los datos de configuracion de la camara
 */
struct CameraConfig
{
    char portName[20];
    unsigned int portVelocity;
    unsigned int idCamera;
    int velPAN;
    int velTILT;
   
};

/**
 * \class ProtPelcoD
 * \brief Clase que implementa el protocolo de comunicacion de la camara (PELCOD) 
 */
class ProtPelcoD 
{
    private:
        int idCamera ;
        PortSerial* serial;
        int velPAN;
        int velTILT;        
    private:
        char calcChecksum();
    public:
        ProtPelcoD(int ID,int vel_pan,int vel_tilt);
        ~ProtPelcoD();
        bool connect(char* namePort,int velocity);
        void disconnect();
        void commandPAN(int typePAN);
        void commandTILT(int typeTilt);
        void commandZOOM(int typeZOOM);
        void commandIRIS(int typeIRIS);
        void commandFOCUS(int typeFOCUS);
        void commandAUTO(int typeAUTO);
        void commandPOWER(int typePOWER);
	void commandOTHER(char,char,char,char);      
        bool sendCommand(char,char,char,char);
        char calcChecksum(char command[]);
        
};

#endif	/* PROTPELCOD_H */

/**
 *@}
 */