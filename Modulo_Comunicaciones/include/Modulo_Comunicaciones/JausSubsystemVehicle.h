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
#include <Modulo_Comunicaciones/constantCommunication.h>


int redondea(float valor);

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
        void establishedCommunication();        
        bool waitForACK(int type,int timeout);

};

#endif	/* JAUSSUBSYSTEM_H */

