/* 
 * File:   JausSubsystemVehicle.h
 * Author: atica
 *
 * Created on 28 de abril de 2014, 12:08
 */

#ifndef JAUSSUBSYSTEM_H
#define	JAUSSUBSYSTEM_H

#include <Modulo_Comunicaciones/HandlerJAUS.h>
#include "../../../Common_files/include/Common_files/constant.h"

//Valores JAUS
#define JAUS_HIGH 25
#define JAUS_NEUTRAL_HIGH 76
#define JAUS_REVERSE 127
#define JAUS_NEUTRAL_LOW 178
#define JAUS_LOW 229

//Estado de las comunicaciones
#define COM_OFF 0
#define COM_ON 1
#define COM_LOSED 2

//Presences VectorS
#define PRESENCE_VECTOR_THROTTLE 0X0001
#define PRESENCE_VECTOR_STEER 0X0010
#define PRESENCE_VECTOR_BRAKE 0X0040

#define PRESENCE_VECTOR_ENGINE 0X01
#define PRESENCE_VECTOR_PARKING_BRAKE 0X02
#define PRESENCE_VECTOR_LIGHT_IR 0X22
#define PRESENCE_VECTOR_LIGHT_CONVENTIONAL 0X32
#define PRESENCE_VECTOR_DIFERENTIAL_LOCK 0X42
#define PRESENCE_VECTOR_ENABLE_LASER2D 0X52
#define PRESENCE_VECTOR_GEAR 0X04

#define PRESENCE_VECTOR_PAN 0X01
#define PRESENCE_VECTOR_TILT 0X02
#define PRESENCE_VECTOR_HOME 0X03
#define PRESENCE_VECTOR_ZOOM 0X04

#define NO_ACK 0
#define ACK_MODE 1
#define ACK_ERROR 2
#define ACK_AVAILABLE 3
#define ACK_FUNC_AUX 3

#define TIMEOUT_ACK 5


class JausSubsystemVehicle 
{
    private:
        HandlerJAUS* handler;
        NodeManager* nm;
        OjCmpt compVehicle;
        FileLoader *configData;

        
   public:
        int communicationState;
        int ackReceived;
        /**bool ackMode;
        bool ackError;
        bool ackAvailable;
        bool ackFunctionAuxiliar;  **/
        static JausSubsystemVehicle* subsystemJAUS; 
        static bool instanceJAUSCreate;
    private:
        JausSubsystemVehicle();
        ~JausSubsystemVehicle();
    public:
        static JausSubsystemVehicle* getInstance();
        bool connect();
        void disconnect();
        bool checkConnection();
        int configureJAUS();
        void sendJAUSMessage(JausMessage txMessage,int type);
        static void rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage);
        void losedCommunication();
        bool waitForACK(int type,int timeout);

};

#endif	/* JAUSSUBSYSTEM_H */

