/* 
 * File:   main.cpp
 * Author: Carlos Amores (AICIA)
 *
 * Created on 13 de mayo de 2014, 12:34
 */

#include <cstdlib>
#include "CITIUS_Control_Manager/Manager.h"

using namespace std;

// Servicio para el cambio de estado / subestado

bool callback_status(CITIUS_Control_Manager::srv_status::Request &rq, CITIUS_Control_Manager::srv_status::Response &rs){
    ros::NodeHandle localNh;
    int estado_actual;
    ROS_INFO("===================================================");
    localNh.getParam("status",estado_actual);
    ROS_INFO("El estado actual es %d\n", estado_actual);
    localNh.setParam("status",rq.status);
    localNh.getParam("status",estado_actual);
    ROS_INFO("Cambiado a: %d\n", estado_actual);
    ROS_INFO("===================================================");
    rs.confirmation = true;
    return true;
}

int main(int argc, char** argv) {
    
    ros::init(argc,argv,"CITIUS_Manager");
    ros::NodeHandle nh;
    
    nh.setParam("status",0);
    
    /* Definicion e inicializacion de servicios cliente / servidor
     * SERVIDORES:
     *          - Cambio de estado (service_status)
     * CLIENTES:
     *          - Obtencion información de vehículo (service_vehicle)
     *          - Obtención información eléctrica (service_electric)
     *          - Obtención de posicionamiento camara delantera (service_front_camera)
     *          - Obtención de posicionamiento camara trasera (service_rear_camera)
     */

    ros::ServiceServer serverStatus = nh.advertiseService("service_status", callback_status);
    ros::ServiceClient clientVehicle = nh.serviceClient<CITIUS_Control_Manager::srv_vehicle>("service_vehicle");
    ros::ServiceClient clientElectric = nh.serviceClient<CITIUS_Control_Manager::srv_electric>("service_electric");
    ros::ServiceClient clientFrontCamera = nh.serviceClient<CITIUS_Control_Manager::srv_frontcam>("service_front_camera");
    ros::ServiceClient clientRearCamera = nh.serviceClient<CITIUS_Control_Manager::srv_rearcam>("service_rear_camera");
    
    /* Definición e inicialización de mensajeria suscriptor / publicador
     * - PUBLICADORES:
     *          - Estado camara delantera alto nivel (/front_camera_hl) a Comunicaciones 
     *          - Estado camara trasera alto nivel (/rear_camera_hl) a Comunicaciones
     *          - Datos GPS+IMU alto nivel (/imugps_hl) a Comunicaciones
     *          - Información vehículo (/vehicle_hl) a Comunicaciones
     *          - Información eléctrica (/electric_hl) a Comunicaciones
     *          - Datos magnetómetro digital alto nivel (/magnetometer_hl) a Comunicaciones
     *          - Control cámara delantera (/ctrl_front_camera) a Camara_delantera
     *          - Control cámara trasera (/ctrl_rear_camera) a Camara_trasera
     *          - Comando de telecontrol (/command) a Vehculo
     *          -  
     * - SUSCRIPTORES:
     *          - Control camara delantera alto nivel (/ctrl_front_camera_hl) de Comunicaciones
     *          - Control camara trasera alto nivel (/ctrl_rear_camera_hl) de Comunicaciones
     *          - Comando de telecontrol (/command_hl) de Comunicaciones
     *          - Datos GPS+IMU (/imugps)
     *          - Datos magnetómetro digital (/magnetometer)
     */
    
    ros::Publisher publisherFrontCamHl
    ros::Publisher publisherRearCamHl
    ros::Publisher publisherImuGpsHl
    ros::Publisher publisherVehicleHl
    ros::Publisher publisherElectricHl
    ros::Publisher publisherMagnetometerHl
    ros::Publisher publisherCtrlFrontCam
    ros::Publisher publisherCtrlRearCam
    ros::Publisher publisherCommand
    ros::Subscriber subscriberCtrlFrontCamHl
    ros::Subscriber subscriberCtrlRearCamHl
    ros::Subscriber subscriberCommandHl
    ros::Subscriber subscriberImuGps
    ros::Subscriber subscriberMagnetometer
    
    ROS_INFO("Preparado para dar servicio al cambio de estado...");
    ros::spin();
    return 0;
}