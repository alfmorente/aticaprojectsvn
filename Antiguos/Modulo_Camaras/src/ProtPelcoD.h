/* 
 * File:   ProtPelcoD.h
 * Author: atica
 *
 * Created on 13 de mayo de 2014, 11:52
 */

#

#ifndef PROTPELCOD_H
#define	PROTPELCOD_H
#include "../../../Common_files/include/Common_files/constant.h"
#include <Modulo_Camaras/PortSerial.h>
#define CAMERA_IRIS_OPEN 1
#define CAMERA_IRIS_CLOSE 0
#define CAMERA_FOCUS_FAR 1
#define CAMERA_FOCUS_NEAR 0
#define CAMERA_AUTO_MANUAL 0
#define CAMERA_AUTO_SCAN 1
#define CAMERA_OFF 0
#define CAMERA_ON 1

#define VEL_PAN 0x20
#define VEL_TILT 0x20
#define VEL_STOP 0X00

#define PELCO_SYNC 0xFF
#define PELCO_ZERO 0x00
#define PELCO_PAN_RIGHT 0x02
#define PELCO_PAN_LEFT 0x04
#define PELCO_TILT_UP 0x08
#define PELCO_TILT_DOWN 0x10
#define PELCO_ZOOM_IN 0x40
#define PELCO_ZOOM_OUT 0x80
#define PELCO_IRIS_OPEN 0x02
#define PELCO_IRIS_CLOSE 0x04
#define PELCO_FOCUS_NEAR 0x01
#define PELCO_FOCUS_FAR 0x80
#define PELCO_AUTO_MANUAL 0x10
#define PELCO_AUTO_SCAN 0x90
#define PELCO_CAMERA_ON 0x88
#define PELCO_CAMERA_OFF 0x08



class ProtPelcoD 
{
    private:
        int idCamera ;
        int velPAN;
        int velTILT;
        PortSerial* serial;
    private:
        char calcChecksum();
    public:
        ProtPelcoD(int);
        ~ProtPelcoD();
        bool connect(char* namePort,int velocity);
        //void commandIris();
        //void commandFocus();
        void commandPAN(int typePAN);
        void commandTILT(int typeTilt);
        //void commandHOME();
        void commandZOOM(int typeZOOM);
        void commandIRIS(int typeIRIS);
        void commandFOCUS(int typeFOCUS);
        void commandAUTO(int typeAUTO);
        void commandPOWER(int typePOWER);
	void commandOTHER(char,char,char,char);
        //void commandAUTO();
        //void commandPower();        
        bool sendCommand(char,char,char,char);
        //bool receiveResponse();
        char calcChecksum(char command[]);
        
};

#endif	/* PROTPELCOD_H */

