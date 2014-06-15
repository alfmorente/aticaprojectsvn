#include "DrivingCameraManager.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

DrivingCameraManager::DrivingCameraManager() {

}

/*******************************************************************************
 * MANEJO DE LA CAMARA
 ******************************************************************************/

// Conexión con dispositivo
bool DrivingCameraManager::connect(){
    return true;
}

// Desconexión con dispositivo
bool DrivingCameraManager::disconnect(){
    return true;
}

// Mandar comando de control
void DrivingCameraManager::setParam(short idParam, float value){

}

// Solicitar informacion mediante comando
float DrivingCameraManager::getParam(short idParam){
    return 0;
}
