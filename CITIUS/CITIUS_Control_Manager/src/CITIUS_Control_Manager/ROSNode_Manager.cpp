
#include "CITIUS_Control_Manager/ROSNode_Manager.h"

// Suscriptor para comando de teleoperacion
// Limpieza de valores del comando y reenvio

void fcn_sub_Command(CITIUS_Control_Manager::msg_command msg){

}

// Servicio para el cambio de estado / subestado

bool statusServer(CITIUS_Control_Manager::srv_status::Request &rq, CITIUS_Control_Manager::srv_status::Response &rs){
    ros::NodeHandle localNh;
    int status;
    ROS_INFO("==================");
    localNh.getParam("status", status);
    ROS_INFO("El estado actual es %d", status);
    localNh.setParam("status", rq.status);
    localNh.getParam("status", status);
    ROS_INFO("Cambiado a: %d", status);
    ROS_INFO("==================");
    rs.confirmation = true;
    return true;
}

// Constructor

ROSNode_Manager::ROSNode_Manager() {
    
}

// Inicializacion

void ROSNode_Manager::initialize(){
        
    /* Definicion e inicializacion de servicios cliente / servidor
     * SERVIDORES:
     *          - Cambio de estado (service_status)
     * CLIENTES:
     *          - Obtencion información de vehículo (service_vehicle)
     *          - Obtención información eléctrica (service_electric)
     *          - Obtención de posicionamiento camara delantera (service_front_camera)
     *          - Obtención de posicionamiento camara trasera (service_rear_camera)
     */
    serverStatus = nh.advertiseService("service_status", statusServer);
    clientVehicle = nh.serviceClient<CITIUS_Control_Manager::srv_vehicle>("service_vehicle");
    clientElectric = nh.serviceClient<CITIUS_Control_Manager::srv_electric>("service_electric");
    clientFrontCamera = nh.serviceClient<CITIUS_Control_Manager::srv_frontcam>("service_front_camera");
    clientRearCamera = nh.serviceClient<CITIUS_Control_Manager::srv_rearcam>("service_rear_camera");
    
    /* Definición e inicialización de mensajeria suscriptor / publicador
     * - PUBLICADORES:
     *          - Estado camara delantera  (/front_camera) a Comunicaciones 
     *          - Estado camara trasera  (/rear_camera) a Comunicaciones
     *          - Información vehículo (/vehicle) a Comunicaciones
     *          - Información eléctrica (/electric) a Comunicaciones
     *          - Comando de telecontrol (/command) a Vehculo
     *          -  
     * - SUSCRIPTORES:
     *          - Comando de telecontrol (/command_hl) de Comunicaciones
     */
    
    publisherFrontCamStatus = nh.advertise<CITIUS_Control_Manager::msg_cameraStatus>("front_camera", 1000);
    publisherRearCamStatus = nh.advertise<CITIUS_Control_Manager::msg_cameraStatus>("rear_camera", 1000);
    publisherVehicleInfo  = nh.advertise<CITIUS_Control_Manager::msg_vehicleInfo>("vehicle", 1000);
    publisherElectricInfo  = nh.advertise<CITIUS_Control_Manager::msg_electricInfo>("electric", 1000);
    publisherCommand  = nh.advertise<CITIUS_Control_Manager::msg_command>("cleanCommand", 1000);
    subscriberCommand = nh.subscribe("command", 1000, fcn_sub_Command);
    
    // Estado de operacion
    nh.setParam("status",0);
}

void ROSNode_Manager::run(){
    ros::spin();
}
