/* 
 * File:   Manager.cpp
 * Author: atica
 * 
 * Created on 17 de julio de 2014, 11:47
 */

#include "Manager.h"

/*******************************************************************************
                        CONSTRUCTOR                                             
 ******************************************************************************/

Manager::Manager() {
    
    positionOrientationOK = false;
    frontCameraOK = false;
    rearCameraOK = false;
    drivingOK = false;
    irCameraOK = false;
    lrfOK = false;
    tvCameraOK = false;
    positionerOK = false;
    currentSwitcher = SWITCHER_INIT;
    
}

/*******************************************************************************
                        DESTRUCTOR                                             
 ******************************************************************************/

Manager::~Manager() {
}

/*******************************************************************************
                        INICIO ROS                                             
 ******************************************************************************/

void Manager::initROS(){
    
    ros::NodeHandle nh;
    
    poNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("poNodeStatus");
    fcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("fcNodeStatus");
    rcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("rcNodeStatus");
    drNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("vmNodeStatus");
    irNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateIRCamera");
    lrfNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateLRF");
    tvNosdeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateTVCamera");
    ptNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStatePanTilt");
    
    serverVehicleStatus = nh.advertiseService("vehicleStatus",&Manager::fcv_serv_vehicleStatus,this);
    
    switcherLocalTelecontrol = nh.subscribe("switcher", 1000, &Manager::fnc_subs_switcher, this);

}

/*******************************************************************************
             MAQUINA DE ESTADOS - SERVICIO vehicleStatus                                           
 ******************************************************************************/

bool Manager::fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp){

    // Manejador ROS
    ros::NodeHandle nh;
    
    // Lectura del estado actual
    int currentStatus;
    nh.getParam("vehicleStatus",currentStatus);
    
    // Numero de reintentos de conexion
    short numOfAttemps = 0;
    
    // Servicio demanda arranque/apagado del sistema
    CITIUS_Control_Manager::srv_nodeStatus service;
    
    // Maquina de estados
    switch(rq.status){
        
        case OPERATION_MODE_INICIANDO:
                        
            // Encendido del sistema. Se mandan a arrancar el resto de los modulos
            // Position/Orientation - FCamera - RCamera - Driving
            
            service.request.status= NODESTATUS_OK;
            
            // Activacion Position/Orientation
            
            while(!poNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo Position/Orientation...");
                numOfAttemps++;
                
            }
            
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    
                    ROS_INFO("[Control] Manager - Nodo Position/Orientation no pudo arrancar");
                
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo Position/Orientation arrancado correctamente");
                    positionOrientationOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo Position/Orientation no pudo arrancar. Cumplido numero maximo de reintentos");
           
            }
            
            numOfAttemps = 0;
            
            // Activacion FrontCamera
            
            while(!fcNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo FrontCamera...");
                numOfAttemps++;
                
            }
            
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo FrontCamera no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo FrontCamera arrancado correctamente");
                    frontCameraOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo FrontCamera no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }
            
            numOfAttemps = 0;
            /*
            // Activacion RearCamera
            
            while(!rcNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo RearCamera...");
                numOfAttemps++;
                
            }
            
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo RearCamera no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo RearCamera arrancado correctamente");
                    rearCameraOK = true;
                }
                
            } else {
                
                ROS_INFO("[Control] Manager - Nodo RearCamera no pudo arrancar. Cumplido numero maximo de reintentos");
           
            }
            
            numOfAttemps = 0;
            */
            // Activacion Driving
            
            while(!drNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo Driving...");
                numOfAttemps++;
                
            }
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo Driving no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo Driving arrancado correctamente");
                    drivingOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo Driving no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }
            
            numOfAttemps = 0;
            
            // Activacion IR Camera
            
            while(!irNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo IR Camera...");
                numOfAttemps++;
                
            }
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo IR Camera no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo IR Camera arrancado correctamente");
                    irCameraOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo IR Camera no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }
            
            numOfAttemps = 0;
            
            // Activacion LRF (Telemetro)
            
            while(!lrfNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo LRF...");
                numOfAttemps++;
                
            }
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo LRF no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo LRF arrancado correctamente");
                    lrfOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo LRF no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }
            
            numOfAttemps = 0;
            
            // Activacion TV Camera
            
            while(!tvNosdeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo Camera TV...");
                numOfAttemps++;
                
            }
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo Camera TV no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo Camera TV arrancado correctamente");
                    tvCameraOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo Camera TV no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }
            
            numOfAttemps = 0;
            
            // Activacion Positioner
            
            while(!ptNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS){
                
                ROS_INFO("[Control] Manager - Reintentando conexion con nodo Positioner...");
                numOfAttemps++;
                
            }
            if (numOfAttemps < MAX_ATTEMPS) {
                
                if (!service.response.confirmation) {
                    
                    // Position/Orientation no ha sido activado
                    ROS_INFO("[Control] Manager - Nodo Positioner no pudo arrancar");
                    
                } else {
                    
                    ROS_INFO("[Control] Manager - Nodo Positioner arrancado correctamente");
                    positionerOK = true;
                    
                }
                
            }else{
                
                ROS_INFO("[Control] Manager - Nodo Positioner no pudo arrancar. Cumplido numero maximo de reintentos");
            
            }

            
            // Comprobacion de errores
            // No se ha conectado con Driving. Se finaliza la ejecucion tras informar
            if(!drivingOK){
                
                ROS_INFO("[Control] Manager - Sin nodo Driving no hay vehiculo. Finalizado");
                nh.setParam("vehicleStatus",OPERATION_MODE_APAGANDO);
                rsp.confirmation = false;
                
            } else {
                if(!positionOrientationOK || !frontCameraOK || !rearCameraOK || !irCameraOK || !lrfOK || !tvCameraOK || !positionerOK){
                    
                    ROS_INFO("[Control] Manager - Algun dispositivo fallo pero se continua con la ejecucion");              
                
                }else{
                    
                    ROS_INFO("[Control] Manager - Todos los nodos arrancaron correctamente");
               
                }
                
                if (rq.posSwitcher == SWITCHER_LOCAL) {
                    
                    ROS_INFO("[Control] Manager - Modo de operacion LOCAL activado");
                    nh.setParam("vehicleStatus", OPERATION_MODE_LOCAL);
                    
                } else if (rq.posSwitcher == SWITCHER_TELECONTROL) {
                    
                    ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
                    nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
                }
                
                rsp.confirmation = true;
            }
            
            break;
            
        case OPERATION_MODE_LOCAL:
            
            // Se atiende siempre a traves del suscriptor al conmutador msg_switcher
            
            ROS_INFO("[Control] Manager - Peticion de entrada en modo LOCAL denegada");
            rsp.confirmation = false;
            
            break;
            
        case OPERATION_MODE_CONDUCCION:
            
            // Se atiende siempre que el estado actual sea OBSERVACION
            // La otra transicion a conduccion se recibe mediante un topic en la posicion local/teleoperado
            
            if(currentStatus == OPERATION_MODE_OBSERVACION){
                
                nh.setParam("vehicleStatus",OPERATION_MODE_CONDUCCION);
                ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
                rsp.confirmation = true;
            
            }else{
                
                rsp.confirmation = false;
                ROS_INFO("[Control] Manager - Peticion de entrada en modo CONDUCCION denegada");
            
            }
            
            break;
            
        case OPERATION_MODE_OBSERVACION:
            
            // Se atiende siempre que el estado actual sea CONDUCCION
            
            if(currentStatus == OPERATION_MODE_CONDUCCION){
                
                nh.setParam("vehicleStatus",OPERATION_MODE_OBSERVACION);
                rsp.confirmation = true;
                ROS_INFO("[Control] Manager - Modo de operacion OBSERVACION activado");
                
            }else{
                
                rsp.confirmation = false;
                ROS_INFO("[Control] Manager - Peticion de entrada en modo OBSERVACION denegada");
            
            } 
            
            break;
            
        case OPERATION_MODE_APAGANDO:
            
            // Apagado ordenado del sistema. Se mandan a arrancar el resto de los modulos
            // Position/Orientation - FCamera - RCamera - Driving
            
            service.request.status= NODESTATUS_OFF;
            
            // Apagado Position/Orientation
            if (positionOrientationOK) {
                while (!poNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo Position/Orientation no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo Position/Orientation apagado correctamente");
                }
            }
            
            // Apagado FrontCamera
            if (frontCameraOK) {
                while (!fcNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo FrontCamera no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo FrontCamera apagado correctamente");
                }
            }
            /*
            // Apagado RearCamera
            if (rearCameraOK) {
                while (!rcNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo RearCamera no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo RearCamera apagado correctamente");
                }
            }
            */
            // Camara IR
            if (irCameraOK) {
                while (!irNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo IRCamera no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo IRCamera apagado correctamente");
                }
            }

            // LRF
            if (lrfOK) {
                while (!lrfNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo LRF no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo LRF apagado correctamente");
                }
            }

            // Camara TV
            if (tvCameraOK) {
                while (!tvNosdeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo TVCamera no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo TVCamera apagado correctamente");
                }
            }

            // Positioner
            if (positionerOK) {
                while (!ptNodeStatus.call(service));
                if (!service.response.confirmation) {
                    // Position/Orientation no ha sido apagado
                    ROS_INFO("[Control] Manager - Nodo Positioner no se pudo apagar");
                } else {
                    ROS_INFO("[Control] Manager - Nodo Positioner apagado correctamente");
                }
            }
            
            // Apagado Driving
            if(drivingOK) {
                while (!drNodeStatus.call(service))
                    if (!service.response.confirmation) {
                        // Position/Orientation no ha sido apagado
                        ROS_INFO("[Control] Manager - Nodo Driving no se pudo apagar");
                    } else {
                        ROS_INFO("[Control] Manager - Nodo Driving apagado correctamente");
                    }
            }
            
            rsp.confirmation = true;
            nh.setParam("vehicleStatus",OPERATION_MODE_APAGANDO);
            break;
            
        default:
            rsp.confirmation = false;
            break;
    };
    
    return true;
}

/*******************************************************************************
                        MAQUINA DE ESTADOS - switcher                                             
 ******************************************************************************/

void Manager::fnc_subs_switcher(CITIUS_Control_Manager::msg_switcher msg){
        
    // Solo tiene efecto en estado CONDUCCION/LOCAL/OBSERVACION
    
    ros::NodeHandle nh;
    int status;
    nh.getParam("vehicleStatus", status);
    
    if (status == OPERATION_MODE_CONDUCCION || status == OPERATION_MODE_LOCAL || status == OPERATION_MODE_OBSERVACION) {
        
        if (msg.switcher == SWITCHER_LOCAL) {
            
            ROS_INFO("[Control] Manager - Modo de operacion LOCAL activado");
            nh.setParam("vehicleStatus", OPERATION_MODE_LOCAL);
            currentSwitcher = SWITCHER_LOCAL;
            
        } else if (msg.switcher == SWITCHER_TELECONTROL) {
            
            ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
            nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
            currentSwitcher = SWITCHER_TELECONTROL;
            
        }
        
    }

}