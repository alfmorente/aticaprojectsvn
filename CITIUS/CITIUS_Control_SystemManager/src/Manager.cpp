/* 
 * File:   main.cpp
 * Author: Carlos Amores (AICIA)
 *
 * Created on 13 de mayo de 2014, 12:34
 */

#include <cstdlib>
#include "CITIUS_Control_SystemManager/Manager.h"

using namespace std;


/*******************************************************************************
 *******************************************************************************
 *          IMPLEMENTACION DE LA CLASE QUE IMPLEMENTA AL NODO DRIVING          *
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 *                  CALLBACKS DE SUSCRIPTORES Y SERVICIOS                     *
 ******************************************************************************/

bool Manager::fcn_serv_nodeStatus(CITIUS_Control_SystemManager::srv_nodeStatus::Request &rq, CITIUS_Control_SystemManager::srv_nodeStatus::Response &rsp) {
    
    bool allNodesReadyToTurnOff = true;
    
    if(rq.status == NODESTATUS_OFF){
        
        ROS_INFO("[Control] Manager :: Solicitud de cambio finalizacion de nodos recibida");
        CITIUS_Control_SystemManager::srv_nodeStatus srv;
        srv.request.status = NODESTATUS_OFF;
        
        // Llamada a apagado del nodo Driving
        if(this->client_driving_nodeStatus.call(srv)){
            if(!srv.response.confirmation){
                allNodesReadyToTurnOff = false;
                ROS_INFO("[Control] Manager :: Imposible finalizar el nodo Driving");
            }else{
                ROS_INFO("[Control] Manager :: Nodo Driving listo para apagar");
            }
        }else{
            allNodesReadyToTurnOff = false;
            ROS_INFO("[Control] Manager :: Respuesta no recibida de Driving para finalizar nodo");
        }
        
        // Llamada a apagado del nodo FrontCamera
        if(this->client_fCamera_nodeStatus.call(srv)){
            if(!srv.response.confirmation){
                allNodesReadyToTurnOff = false;
                ROS_INFO("[Control] Manager :: Imposible finalizar el nodo FrontCamera");
            }else{
                ROS_INFO("[Control] Manager :: Nodo FrontCamera listo para apagar");
            }
        }else{
            allNodesReadyToTurnOff = false;
            ROS_INFO("[Control] Manager :: Respuesta no recibida de FrontCamera para finalizar nodo");
        }

        // Llamada a apagado del nodo RearCamera
        if(this->client_rCamera_nodeStatus.call(srv)){
            if(!srv.response.confirmation){
                allNodesReadyToTurnOff = false;
                ROS_INFO("[Control] Manager :: Imposible finalizar el nodo RearCamera");
            }else{
                ROS_INFO("[Control] Manager :: Nodo RearCamera listo para apagar");
            }
        }else{
            allNodesReadyToTurnOff = false;
            ROS_INFO("[Control] Manager :: Respuesta no recibida de RearCamera para finalizar nodo");
        }         
        
        // Llamada a apagado del nodo Posicion/Orientacion
        if(this->client_posOri_nodeStatus.call(srv)){
             if(!srv.response.confirmation){
                 allNodesReadyToTurnOff = false;
                 ROS_INFO("[Control] Manager :: Imposible finalizar el nodo Position/Orientation");
            }else{
                ROS_INFO("[Control] Manager :: Nodo Position/Orientation listo para apagar");
            }
        }else{
            allNodesReadyToTurnOff = false;
            ROS_INFO("[Control] Manager :: Respuesta no recibida de Position/Orientation para finalizar nodo");
        }  
        
    }else{
        
        ROS_INFO("[Control] Manager :: Solicitud de cambio en el estado del nodo distinto a OFF");
    
    }
    
    rsp.confirmation = allNodesReadyToTurnOff;
    
    return true;
}

bool Manager::fcn_serv_vehicleStatus(CITIUS_Control_SystemManager::srv_vehicleStatus::Request &rq, CITIUS_Control_SystemManager::srv_vehicleStatus::Response &rsp) {

    switch (rq.status) {
        case OPERATION_MODE_LOCAL:
            
            // Recibida una peticion de entrar en modo LOCAL
            // Se conmuta sin condiciones
                    this->nh.setParam("vehicleStatus", OPERATION_MODE_LOCAL);
            ROS_INFO("[Control] Manager :: Modo de operacion del vehiculo cambiado a LOCAL");
            break;

        case OPERATION_MODE_CONDUCCION:

            // Recibida una peticion de entrar en modo CONDUCCION
            // Solo se realiza la transcicion si el estado de operacion actual es OBSERVACION
            // La transicion de LOCAL a CONDUCCION se realiza en Driving
            int presentStatus;
            this->nh.getParam("vehicleStatus", presentStatus);
            if (presentStatus == OPERATION_MODE_OBSERVACION) {
                this->nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
                ROS_INFO("[Control] Manager :: Modo de operacion del vehiculo cambiado a CONDUCCION");
            } else {
                ROS_INFO("[Control] Manager :: No se puede cambiar a modo CONDUCCION. El conmutador no está en modo automatico");
            }
            break;

        case OPERATION_MODE_OBSERVACION:

            // Recibida una peticion de entrar en modo OBSERVACION
            // Solo se realiza la transicion si el estado de operacion actual es CONDUCCION
            this->nh.getParam("vehicleStatus", presentStatus);
            if (presentStatus == OPERATION_MODE_CONDUCCION) {
                this->nh.setParam("vehicleStatus", OPERATION_MODE_OBSERVACION);
                ROS_INFO("[Control] Manager :: Modo de operacion del vehiculo cambiado a OBSERVACION");
            } else {
                ROS_INFO("[Control] Manager :: No se puede cambiar a modo OBSERVACION. El conmutador no está en modo automatico");
            }
            break;

        case OPERATION_MODE_APAGANDO:

            // Recibida una peticion de entrar en modo APAGADO
            // Se conmuta sin condiciones
            this->nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);
            ROS_INFO("[Control] Manager :: Modo de operacion del vehiculo cambiado a APAGADO");
            break;

        default:
            break;
    }
    return true;
}

/*******************************************************************************
 *                       CONSTRUCTOR -> INICIALIZADOR                          *
 ******************************************************************************/

// Constructor de la clase
Manager::Manager() {
    
    ROS_INFO("[Control] Manager :: Iniciando nodo Manager");
    
    // Inicializacion de servidores
    this->server_nodeStatus = this->nh.advertiseService("veNodeStatus", &Manager::fcn_serv_nodeStatus,this);
    this->server_vehicleStatus = this->nh.advertiseService("vehicleStatus,",&Manager::fcn_serv_vehicleStatus,this);
    
    // Inicializacion de clientes
    this->client_driving_nodeStatus = nh.serviceClient<CITIUS_Control_SystemManager::srv_nodeStatus>("vmNodeStatus");
    this->client_fCamera_nodeStatus = nh.serviceClient<CITIUS_Control_SystemManager::srv_nodeStatus>("fcNodeStatus");
    this->client_rCamera_nodeStatus = nh.serviceClient<CITIUS_Control_SystemManager::srv_nodeStatus>("rcNodeStatus");
    this->client_posOri_nodeStatus = nh.serviceClient<CITIUS_Control_SystemManager::srv_nodeStatus>("poNodeStatus");
    
    ROS_INFO("[Control] Manager :: Nodo iniciado satisfactoriamente");
    
}