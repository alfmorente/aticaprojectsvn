/* 
 * File:   RosNose_PositionOrientation.h
 * Author: Carlos Amores
 *
 * Created on 4 de julio de 2014, 9:05
 */

#ifndef ROSNOSE_POSITIONORIENTATION_H
#define	ROSNOSE_POSITIONORIENTATION_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ROSNOSE_POSITIONORIENTATION_H */

#include "CITIUS_Control_PositionOrientation/srv_nodeStatus.h"
#include "CITIUS_Control_PositionOrientation/msg_posOriInfo.h"
#include "constant.h"
#include "XSensMTi700Driver.h"
#include "TraxAHRSModuleDriver.h"

class RosNode_PositionOrientation{
private:
    // Estado del nodo
    short poNodeStatus;
    // Publicador de informacion de Posicion/Orientacion
    ros::Publisher pubPosOriInfo;
    // Cliente de servicio de estado de nodo
    ros::ServiceServer servNodeStatus;
    // Driver de GPS/INS (XSens MTi-G 700)
    XSensMTi700Driver *gpsinsDriver;
    // Driver de Magnetometro (PNI TRAX AHRS Module)
    TraxAHRSModuleDriver *magnetometerDriver;
    // Flags de actividad
    bool magnOK;
    bool gpsinsOK;
    
public:
    // Constructor
    RosNode_PositionOrientation();
    // Destructor
    ~RosNode_PositionOrientation();

    // Inicializador de artefactos ROS
    void initROS();
    // Callbacks ROS
    bool fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &);
    // Getter and Setter necesarios
    ros::Publisher getPubPosOriInfo();
    short getPONodeStatus();
    void setPONodeStatus(short);
    XSensMTi700Driver *getXSensManager();
    TraxAHRSModuleDriver *getMagnetometerManager();
    bool getGpsStatus();
    bool getMagnStatus();
    void setGpsStatus(bool);
    void setMagnStatus(bool);
    // Publicador de la informacion
    void publishInformation();
    // Configuracion de dispositivos
    void configureDevices();
};



