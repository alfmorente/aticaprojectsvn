/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 30 de julio de 2014, 10:55
 */

#define ST_FRONT_CAM 1
#define ST_REAR_CAM 2
#define ST_DISCONNECTED 3
#define ST_EXIT 4

#include <cstdlib>
#include <iostream>

#include "AxisP3364LveDriver.h"

using namespace std;

void printMenu(int);
int cinValue();

/*
 * 
 */
int main(int argc, char** argv) {
    
    // Estado del menu desplegable
    int status = ST_DISCONNECTED;
    // Almacenamiento de estado auxiliar
    int auxStatus;
    // Driver a probar
    AxisP3364LveDriver *driverFrontCam = new AxisP3364LveDriver();
    AxisP3364LveDriver *driverRearCam = new AxisP3364LveDriver();
    
    while(status!=ST_EXIT){
        
        printMenu(status);
        
        cin >> auxStatus;
        
        switch(auxStatus){
            case 1:
                if(status == ST_DISCONNECTED){
                    status = auxStatus;
                }else if(status == ST_FRONT_CAM){
                    system("firefox http://192.168.24.120/jpg/image.jpg");
                }else if(status == ST_REAR_CAM){
                    system("firefox http://192.168.24.121/jpg/image.jpg");
                }else{
                    cout << "Opcion invalida" << endl;
                }
                break;
            case 2:
                if(status == ST_DISCONNECTED){
                    status = auxStatus;
                }else if(status == ST_FRONT_CAM){
                    int value = cinValue();
                    driverFrontCam->sentSetToDevice(ORDER_PAN,value, ST_FRONT_CAM);
                }else if(status == ST_REAR_CAM){
                    int value = cinValue();
                    driverRearCam->sentSetToDevice(ORDER_PAN,value, ST_REAR_CAM);
                }else{
                    cout << "Opcion invalida" << endl;
                }
                break;
            case 3:
                if(status == ST_DISCONNECTED){
                    status = ST_EXIT;
                }else if(status == ST_FRONT_CAM){
                    int value = cinValue();
                    driverFrontCam->sentSetToDevice(ORDER_TILT,value, ST_FRONT_CAM);
                }else if(status == ST_REAR_CAM){
                    int value = cinValue();
                    driverRearCam->sentSetToDevice(ORDER_TILT,value, ST_REAR_CAM);
                }else{
                    cout << "Opcion invalida" << endl;
                }
                break;
            case 4:
                if(status == ST_FRONT_CAM){
                    int value = cinValue();
                    if(driverFrontCam->sentSetToDevice(ORDER_ZOOM,value,ST_FRONT_CAM)){
                    
                    }
                }else if(status == ST_REAR_CAM){
                    int value = cinValue();
                    driverRearCam->sentSetToDevice(ORDER_ZOOM,value,ST_REAR_CAM);
                }else{
                    cout << "Opcion invalida" << endl;
                }
                break;
            case 5:
                if(status == ST_FRONT_CAM){
                    status = ST_DISCONNECTED;
                }else if(status == ST_REAR_CAM){
                    status = ST_DISCONNECTED;
                }
                break;
            default:
                break;
        }
        
    }
    
    return 0;
}

void printMenu(int status){
    
    cout << "*******************************************" << endl;
    cout << "TEST_SW :: TEST_CAMERAS" << endl;
    cout << "*******************************************" << endl;
    cout << "Selecciona una opcion y pulsa INTRO:" << endl;

    switch(status){

        case ST_FRONT_CAM:
            
            cout << "1. Mostrar imagen" << endl;
            cout << "2. Control de PAN" << endl;
            cout << "3. Control de TILT" << endl;
            cout << "4. Control de ZOOM" << endl;
            cout << "5. Volver" << endl;
            
            break;
            
        case ST_REAR_CAM:
            
            cout << "1. Mostrar imagen" << endl;
            cout << "2. Control de PAN" << endl;
            cout << "3. Control de TILT" << endl;
            cout << "4. Control de ZOOM" << endl;
            cout << "5. Volver" << endl;
            
            
            break;
            
        case ST_DISCONNECTED:
            
            cout << "1. Camara delantera" << endl;
            cout << "2. Camara trasera" << endl;
            cout << "3. Salir" << endl;
            
            break;
            
        default:
            
            break;
            
            
    }
    cout << "*******************************************" << endl;
}

int cinValue(){
    int ret = -50000;
    int aux;
    while(ret == -50000){
        cout << "Introduce un valor para el parametro requerido:" << endl;
        cin >> aux;
        if(aux < -100 || aux > 100){
            cout << "valor invalido. Intervalo -100..100" << endl;
        }else{
            ret= aux;
        }
    }
    return ret*(5000/100);
}