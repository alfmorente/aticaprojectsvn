/* 
 * File:   RosNodePositionOrientation.cpp
 * Author: Carlos Amores
 *
 * Created on 4 de julio de 2014, 16:46
 */

#include <ros/node_handle.h>

#include "RosNose_PositionOrientation.h"

using namespace std;

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_PositionOrientation::RosNode_PositionOrientation(){
    setPONodeStatus(NODESTATUS_INIT);
    gpsinsDriver = new XSensMTi700Driver();
    magnetometerDriver = new TraxAHRSModuleDriver();
    magnOK = false;
    gpsinsOK = false;
}

/*******************************************************************************
 * DESTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_PositionOrientation::~RosNode_PositionOrientation(){
    delete gpsinsDriver;
    delete magnetometerDriver;
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_PositionOrientation::initROS(){
    ros::NodeHandle nh;
    pubPosOriInfo = nh.advertise<CITIUS_Control_PositionOrientation::msg_posOriInfo>("posOriInfo",1000);
    servNodeStatus = nh.advertiseService("poNodeStatus", &RosNode_PositionOrientation::fcn_serv_nodeStatus,this);
}

/*******************************************************************************
 * CONFIGURACION DE DISPOSITIVOS
 ******************************************************************************/

void RosNode_PositionOrientation::configureDevices(){
    if(gpsinsOK){
        gpsinsDriver->configureDevice();
        ROS_INFO("[Control] Position / Orientation - Configuracion de GPS/INS establecida");
    }
        
    if(magnOK){
        magnetometerDriver->configureDevice();
        ROS_INFO("[Control] Position / Orientation - Configuracion de Magnetometro establecida");
    }
}

/*******************************************************************************
 * CALLBACKS NECESARIOS
 ******************************************************************************/

bool RosNode_PositionOrientation::fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &rq, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &rsp) {
    if (rq.status == NODESTATUS_OK) {
        
        if (poNodeStatus != NODESTATUS_OK) {
            poNodeStatus = NODESTATUS_OK;
            rsp.confirmation = true;
            
        } else {
            
            rsp.confirmation = false;
            
        }

    } else if (rq.status == NODESTATUS_OFF) {
        
        poNodeStatus = NODESTATUS_OFF;
        rsp.confirmation = true;
        
    } else {
        
        rsp.confirmation = false;
        
    }

    return true;
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

short RosNode_PositionOrientation::getPONodeStatus(){ return poNodeStatus; }

void RosNode_PositionOrientation::setPONodeStatus(short newPONodeStatus){ poNodeStatus = newPONodeStatus; }

ros::Publisher RosNode_PositionOrientation::getPubPosOriInfo(){ return pubPosOriInfo; }

XSensMTi700Driver *RosNode_PositionOrientation::getXSensManager(){ return gpsinsDriver; }

TraxAHRSModuleDriver *RosNode_PositionOrientation::getMagnetometerManager(){ return magnetometerDriver; }

bool RosNode_PositionOrientation::getGpsStatus() { return gpsinsOK; }

bool RosNode_PositionOrientation::getMagnStatus() { return magnOK; }

void RosNode_PositionOrientation::setGpsStatus(bool status) { gpsinsOK = status; }

void RosNode_PositionOrientation::setMagnStatus(bool status) { magnOK = status;
}

/*******************************************************************************
 * PUBLICADOR DE INFORMACION
 ******************************************************************************/

void RosNode_PositionOrientation::publishInformation() {
    
    if (gpsinsOK && !magnOK) {
        if (gpsinsDriver->getData()) {
            // Conversion a mensaje ROS y publicacion
            
            CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;
            GPSINSInfo information = gpsinsDriver->getInfo();
            
            
            msgSnd.positionStatus = information.positionStatus;
            msgSnd.orientationStatus = information.orientationStatus;
            msgSnd.latitude = information.latitude;
            msgSnd.longitude = information.longitude;
            msgSnd.altitude = information.altitude;
            msgSnd.roll = information.roll;
            msgSnd.pitch = information.pitch;
            msgSnd.yaw = information.yaw;
            msgSnd.velX = information.velX;
            msgSnd.velY = information.velY;
            msgSnd.velZ = information.velZ;
            msgSnd.accX = information.accX;
            msgSnd.accY = information.accY;
            msgSnd.accZ = information.accZ;
            msgSnd.rateX = information.rateX;
            msgSnd.rateY = information.rateY;
            msgSnd.rateZ = information.rateZ;

            pubPosOriInfo.publish(msgSnd);
        }
        
    } else if (!gpsinsOK && magnOK) {
        if (magnetometerDriver->getData()) {
            // Conversion a mensaje ROS y publicacion
            CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;
            TraxMeasurement information = magnetometerDriver->getInfo();
            
            msgSnd.positionStatus = 0;
            msgSnd.orientationStatus = information.heading_status;
            msgSnd.latitude = 0;
            msgSnd.longitude = 0;
            msgSnd.altitude = 0;
            msgSnd.roll = information.roll;
            msgSnd.pitch = information.pitch;
            msgSnd.yaw = information.heading;
            msgSnd.velX = 0;
            msgSnd.velY = 0;
            msgSnd.velZ = 0;
            msgSnd.accX = information.accX;
            msgSnd.accY = information.accY;
            msgSnd.accZ = information.accZ;
            msgSnd.rateX = information.gyrX;
            msgSnd.rateY = information.gyrY;
            msgSnd.rateZ = information.gyrZ;
            
            pubPosOriInfo.publish(msgSnd);
        }
        
    } else if (gpsinsOK && magnOK) {
        if (gpsinsDriver->getData()) {
            // Conversion a mensaje ROS y publicacion
            
            CITIUS_Control_PositionOrientation::msg_posOriInfo msgSnd;
            
            GPSINSInfo informationG = gpsinsDriver->getInfo();
            
            msgSnd.positionStatus = informationG.positionStatus;
            msgSnd.latitude = informationG.latitude;
            msgSnd.longitude = informationG.longitude;
            msgSnd.altitude = informationG.altitude;
            msgSnd.velX = informationG.velX;
            msgSnd.velY = informationG.velY;
            msgSnd.velZ = informationG.velZ;

            if (magnetometerDriver->getData()) {
                
                TraxMeasurement informationM = magnetometerDriver->getInfo();
                
                if (informationM.heading_status == 1) {
                    
                    msgSnd.orientationStatus = informationM.heading_status;
                    msgSnd.roll = informationM.roll;
                    msgSnd.pitch = informationM.pitch;
                    msgSnd.yaw = informationM.heading;
                    msgSnd.accX = informationM.accX;
                    msgSnd.accY = informationM.accY;
                    msgSnd.accZ = informationM.accZ;
                    msgSnd.rateX = informationM.gyrX;
                    msgSnd.rateY = informationM.gyrY;
                    msgSnd.rateZ = informationM.gyrZ;
                    
                } else {
                    
                    msgSnd.orientationStatus = informationG.orientationStatus;
                    msgSnd.roll = informationG.roll;
                    msgSnd.pitch = informationG.pitch;
                    msgSnd.yaw = informationG.yaw;
                    msgSnd.accX = informationG.accX;
                    msgSnd.accY = informationG.accY;
                    msgSnd.accZ = informationG.accZ;
                    msgSnd.rateX = informationG.rateX;
                    msgSnd.rateY = informationG.rateY;
                    msgSnd.rateZ = informationG.rateZ;
                }
                
            } else {
                
                msgSnd.orientationStatus = informationG.orientationStatus;
                msgSnd.roll = informationG.roll;
                msgSnd.pitch = informationG.pitch;
                msgSnd.yaw = informationG.yaw;
                msgSnd.accX = informationG.accX;
                msgSnd.accY = informationG.accY;
                msgSnd.accZ = informationG.accZ;
                msgSnd.rateX = informationG.rateX;
                msgSnd.rateY = informationG.rateY;
                msgSnd.rateZ = informationG.rateZ;
                
            }

            pubPosOriInfo.publish(msgSnd);
        }
        
    }
    // TODO MAGNETOMETRO



}