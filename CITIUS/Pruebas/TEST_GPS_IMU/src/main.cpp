/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 16 de julio de 2014, 10:59
 */

#define ST_CONNECTED 1
#define ST_DISCONNECTED 3
#define ST_EXIT 4


#include <cstdlib>
#include "XSensMTi700Driver.h"

using namespace std;

void printMenu(int);

/*
 * 
 */
int main(int argc, char** argv) {
    
    // Estado del menu desplegable
    int status = ST_DISCONNECTED;
    // Almacenamiento de estado auxiliar
    int auxStatus;
    // Driver a probar
    XSensMTi700Driver *driver = new XSensMTi700Driver();

    while (status != ST_EXIT) {

        printMenu(status);
        
        cin >> auxStatus;
        
        switch (auxStatus){           
            case 1:
                if(status != ST_CONNECTED){
                    
                    if(driver->connectToDevice()){
                        driver->configureDevice();
                        cout << "Conexion establecida y dispositivo configurado" << endl;
                        status = ST_CONNECTED;
                    }else{
                        cout << "No se ha detectado el dispositivo" << endl;
                        status = ST_EXIT;
                    }
                }else{
                    cout << "Ya esta conectado" << endl;
                }
                break;
                
            case 2:
                
                if(status == ST_CONNECTED){
                   
                    if(driver->getData()){
                        
                        GPSINSInfo info = driver->getInfo();
                        cout << "Datos: " << endl;
                        cout << "Estado posicion: " << info.positionStatus << endl;
                        cout << "Lat: " << info.latitude << " Lon: " << info.longitude << " Alt: " << info.altitude << endl;
                        cout << "Roll: " << info.roll << " Pitch: " << info.pitch << " Yaw: " << info.yaw << endl;
                        cout << "Vx: " << info.velX << " Vy: " << info.velY << " Vz: " << info.velZ << endl;
                        cout << "AccX: " << info.accX << " AccY: " << info.accY << " AccZ: " << info.accZ << endl;
                        cout << "GyrX: " << info.rateX << " GyrY: " << info.rateY << "GyrZ: " << info.rateZ << endl;
                    }else{
                        cout << "No se han podido obtener datos" << endl;
                    }
                }else{
                    cout << "No se puede solicitar datos sin haber conectado previamente" << endl;
                }
                break;
                
            case 3:
                
                if(status != ST_CONNECTED){
                    cout << "Para desconectar hay que estar conectado" << endl;
                }else{
                    driver->disconnectDevice();
                    cout << "Dispositivo desconectado correctamente" << endl;
                    status = ST_DISCONNECTED;
                }
                
                break;

            case 4:
                if (status == ST_CONNECTED) {
                    cout << "Desconecte antes el dispositivo" << endl;
                } else {
                    status = ST_EXIT;
                }

                break;
                
            default:
                
                cout << "Opcion incorrecta" << endl;
                break;
        };
        
        
        

    }
    
    return 0;
}

// Impresion del menu

void printMenu(int status){
    
    cout << "*******************************************" << endl;
    cout << "TEST_SW :: TEST_GPS_IMU" << endl;
    cout << "*******************************************" << endl;
    cout << "Selecciona una opcion y pulsa INTRO:" << endl;

    switch(status){

        case ST_CONNECTED:
            
            cout << "1. Conectar :: No disponible. Ya conectado." << endl;
            cout << "2. Solicitad datos" << endl;
            cout << "3. Desconectar" << endl;
            cout << "4. Salir" << endl;
            
            break;
            
        case ST_DISCONNECTED:
            
            cout << "1. Conectar" << endl;
            cout << "2. Solicitad datos :: No disponible. Conecta antes" << endl;
            cout << "3. Desconectar :: No disponible. Conecta antes" << endl;
            cout << "4. Salir" << endl;
            
            break;
            
        default:
            
            break;
            
            
    }
    cout << "*******************************************" << endl;
}
