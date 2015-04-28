/**
 * @file   JausSubsystemVehicle.h
 * @brief  Fichero de cabecera para la comunicación JAUS del vehículo
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
 */



#ifndef JAUSSUBSYSTEM_H
#define	JAUSSUBSYSTEM_H

#include <Modulo_Comunicaciones/HandlerJAUS.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include <Modulo_Comunicaciones/constantCommunication.h>

/**
 * \struct ComConfig
 * \brief  Estructura con los datos de configuracion de la comunicacion
 */
struct ComConfig
{
    //Por rellenar
};

int redondea(float valor);

/**
 * \class JausSubsystemVehicle
 * \brief  Clase que gestiona el subsistema JAUS del vehículo
 */
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

/**
 *@}
 */