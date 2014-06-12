
#include "CITIUS_Control_Communication/Communication.h"

Communications::Communications() {

  // Inicializacion de publicadores
  this->pubCommand = this->nh.advertise<CITIUS_Control_Communication::msg_command>("command", 1000);
  this->pubCtrlFrontCamera = this->nh.advertise<CITIUS_Control_Communication::msg_ctrlFrontCamera>("ctrlFrontCamera", 1000);
  this->pubCtrlRearCamera = this->nh.advertise<CITIUS_Control_Communication::msg_ctrlRearCamera>("ctrlRearCamera", 1000);
  // Inicializacion de suscriptores
  this->subsElectricInfo = this->nh.subscribe("electricInfo", 1000, &Communications::fnc_subs_electricInfo, this);
  this->subsVehicleInfo = this->nh.subscribe("vehicleInfo", 1000, &Communications::fnc_subs_vehicleInfo, this);
  this->subsFrontCameraInfo = this->nh.subscribe("frontCameraInfo", 1000, &Communications::fnc_subs_frontCameraInfo, this);
  this->subsRearCameraInfo = this->nh.subscribe("rearCameraInfo", 1000, &Communications::fnc_subs_rearCameraInfo, this);
  this->subsPosOriInfo = this->nh.subscribe("posOriInfo", 1000, &Communications::fnc_subs_posOriInfo, this);
  // Inicializacion de servidores
  this->clientStatus = nh.serviceClient<CITIUS_Control_Communication::srv_vehicleStatus>("nodeStateDriving");
}

/*******************************************************************************
 *******************************************************************************
 *                     GETTER AND SETTER DE ATRIBUTOS                          *
 *******************************************************************************
 ******************************************************************************/
ros::Publisher Communications::getPublisherFrontCamera() {
    return this->pubCtrlFrontCamera;
}

ros::Publisher Communications::getPublisherRearCamera() {
    return this->pubCtrlRearCamera;
}

ros::Publisher Communications::getPublisherCommand() {
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
void Communications::fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg) {
}

/* 
 * INFORMACION DE CAMARA TRASERA
 * CORRESPONDENCIA JAUS: REPORT CAMERA POSE
 */
void Communications::fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg) {
}

/* 
 * INFORMACION DE VEHICULO
 * CORRESPONDENCIA JAUS: REPORT DISCRETE DEVICE / REPORT WRENCH EFFORT / REPORT SIGNALING ELEMENTS
 */
void Communications::fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg) {
}

/* 
 * INFORMACION ELECTRICA
 * CORRESPONDENCIA JAUS: UGV INFO
 */
void Communications::fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg) {
}

/* 
 * INFORMACION DE POSICION / ORIENTACION
 * CORRESPONDENCIA JAUS: REPORT GLOBAL POSE / REPORT VELOCITY STATE / ADDITIONAL GPS/INS INFO
 */
void Communications::fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg) {
}

