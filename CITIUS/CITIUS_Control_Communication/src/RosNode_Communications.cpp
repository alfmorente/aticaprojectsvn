
#include "RosNode_Communications.h"

RosNode_Communications::RosNode_Communications() {

}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS ROS                        *
 *******************************************************************************
 ******************************************************************************/
void RosNode_Communications::initROS() {
    ros::NodeHandle nh;
    // Inicializacion de publicadores
    this->pubCommand = nh.advertise<CITIUS_Control_Communication::msg_command>("command", 1000);
    this->pubCtrlFrontCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlFrontCamera>("ctrlFrontCamera", 1000);
    this->pubCtrlRearCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlRearCamera>("ctrlRearCamera", 1000);
    // Inicializacion de suscriptores
    this->subsElectricInfo = nh.subscribe("electricInfo", 1000, &RosNode_Communications::fnc_subs_electricInfo, this);
    this->subsVehicleInfo = nh.subscribe("vehicleInfo", 1000, &RosNode_Communications::fnc_subs_vehicleInfo, this);
    this->subsFrontCameraInfo = nh.subscribe("frontCameraInfo", 1000, &RosNode_Communications::fnc_subs_frontCameraInfo, this);
    this->subsRearCameraInfo = nh.subscribe("rearCameraInfo", 1000, &RosNode_Communications::fnc_subs_rearCameraInfo, this);
    this->subsPosOriInfo = nh.subscribe("posOriInfo", 1000, &RosNode_Communications::fnc_subs_posOriInfo, this);
    // Inicializacion de servidores
    this->clientStatus = nh.serviceClient<CITIUS_Control_Communication::srv_vehicleStatus>("nodeStateDriving");
}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS JAUS                        *
 *******************************************************************************
 ******************************************************************************/
void RosNode_Communications::initJAUS() {
    // Primitive Driver
    this->primitiveDriverComponent = ojCmptCreate("Primitive Driver",JAUS_PRIMITIVE_DRIVER,1);
    
}

/*******************************************************************************
 *******************************************************************************
 *                     GETTER AND SETTER DE ATRIBUTOS                          *
 *******************************************************************************
 ******************************************************************************/
ros::Publisher RosNode_Communications::getPublisherFrontCamera() {
    return this->pubCtrlFrontCamera;
}

ros::Publisher RosNode_Communications::getPublisherRearCamera() {
    return this->pubCtrlRearCamera;
}

ros::Publisher RosNode_Communications::getPublisherCommand() {
    return this->pubCommand;
}

/*******************************************************************************
 *******************************************************************************
 *                     CALLBACKS                         *
 *******************************************************************************
 ******************************************************************************/

/* 
 * INFORMACION DE CAMARA DELANTERA
 * CORRESPONDENCIA JAUS: REPORT CAMERA POSE
 */
void RosNode_Communications::fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de camara delantera");
}

/* 
 * INFORMACION DE CAMARA TRASERA
 * CORRESPONDENCIA JAUS: REPORT CAMERA POSE
 */
void RosNode_Communications::fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de camara trasera");
}

/* 
 * INFORMACION DE VEHICULO
 * CORRESPONDENCIA JAUS: REPORT DISCRETE DEVICE / REPORT WRENCH EFFORT / REPORT SIGNALING ELEMENTS
 */
void RosNode_Communications::fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de vehiculo");
    
    JausMessage jMsg;
    
    JausAddress address = jausAddressCreate();
    address->subsystem = 0;
    address->node = 0;
    address->component = 0;
    address->instance = 0;
    
    if(msg.id_device == THROTTLE){
        ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
        rwem->propulsiveLinearEffortXPercent = msg.value;
        jausAddressCopy(rwem->destination,address);
        jMsg = reportWrenchEffortMessageToJausMessage(rwem);
        reportWrenchEffortMessageDestroy(rwem);
    }
    
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION ELECTRICA
 * CORRESPONDENCIA JAUS: UGV INFO
 */
void RosNode_Communications::fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion electrica");
}

/* 
 * INFORMACION DE POSICION / ORIENTACION
 * CORRESPONDENCIA JAUS: REPORT GLOBAL POSE / REPORT VELOCITY STATE / ADDITIONAL GPS/INS INFO
 */
void RosNode_Communications::fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de posicionamiento");
}

