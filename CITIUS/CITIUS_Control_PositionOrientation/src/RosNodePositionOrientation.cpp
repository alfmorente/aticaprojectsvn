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
    magnetometerDriver = new TraxAHRSDriver();
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
 * CALLBACKS NECESARIOS
 ******************************************************************************/

bool RosNode_PositionOrientation::fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &rq, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        
        if(poNodeStatus!=NODESTATUS_OK){
            
            poNodeStatus = NODESTATUS_OK;
            
            // Configuracion de dispositivo magnetometro
            magnetometerDriver->configureDevice();
            ROS_INFO("[Control] Position / Orientation - Configuracion de Magnetometro establecida");
            
            // Configuracion de dispositivo GPS/INS
            gpsinsDriver->configureDevice();
            ROS_INFO("[Control] Position / Orientation - Configuracion de GPS/INS establecida");
            
            rsp.confirmation = true;
            
        }else
            rsp.confirmation = false;   
        
    }else if(rq.status == NODESTATUS_OFF){
        
        // Desconectar dispositivo Magnetometro
        magnetometerDriver->disconnectDevice();
        ROS_INFO("[Control] Position / Orientation - Desconectado Magnetometro");
        
        // Desconectar dispositivo GPS/INS
        gpsinsDriver->disconnectDevice();
        ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");
        
        poNodeStatus = NODESTATUS_OFF;
        
        rsp.confirmation = true;
        
    }else
        rsp.confirmation = false;
    
    
    return true;
}

/*******************************************************************************
 * GETTER Y SETTER NECESARIOS
 ******************************************************************************/

short RosNode_PositionOrientation::getPONodeStatus(){ return poNodeStatus; }

void RosNode_PositionOrientation::setPONodeStatus(short newPONodeStatus){ poNodeStatus = newPONodeStatus; }

ros::Publisher RosNode_PositionOrientation::getPubPosOriInfo(){ return pubPosOriInfo; }

XSensMTi700Driver *RosNode_PositionOrientation::getXSensManager(){ return gpsinsDriver; }

TraxAHRSDriver *RosNode_PositionOrientation::getTraxManager(){ return magnetometerDriver; }