
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

