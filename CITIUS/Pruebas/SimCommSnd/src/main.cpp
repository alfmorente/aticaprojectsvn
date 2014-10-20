/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 9 de septiembre de 2014, 12:11
 */

#include "JausController.h"
#include <cstdlib>
#include <iostream>

using namespace std;

int printfMenu();

/*
 * 
 */
int main(int argc, char** argv) {
    

    JausController *nodeComm = JausController::getInstance();

    nodeComm->initJAUS();
    
    bool exit = false;
    
    int newSt = 0;
    
    while(!exit){
        newSt = printfMenu();
        if(newSt == 0){ // Opcion invalida
            cout << "Opcion invalida. Prueba de nuevo" << endl;
        }else if(newSt == 13){ // Salir
            exit = true;
        }else{ // Opcion valida
            nodeComm->sendMessage(newSt);
        }
    }
    
    nodeComm->endJAUS();
    
    return 0;
}

int printfMenu(){
    
    cout << "****************************************" << endl;
    cout << "1 ) Enviar mensaje RUN MISSION" << endl;
    cout << "2 ) Enviar mensaje SET WRENCH EFFORT" << endl;
    cout << "3 ) Enviar mensaje SET DISCRETE DEVICE" << endl;
    cout << "4 ) Enviar mensaje SET TRAVEL SPEED" << endl;
    cout << "5 ) Enviar mensaje SET CAMERA POSE" << endl;
    cout << "6 ) Enviar mensaje PAUSE MISSION" << endl;
    cout << "7 ) Enviar mensaje RESUMEN MISSION" << endl;
    cout << "8 ) Enviar mensaje TELEMETER INFO" << endl;
    cout << "9 ) Enviar mensaje SET SIGNALING ELEMENTS" << endl;
    cout << "10 ) Enviar mensaje SET POSITIONER" << endl;
    cout << "11) Enviar mensaje SET DAT-TIME CAMERA" << endl;
    cout << "12) Enviar mensaje SET NIGHT-TIME CAMERA" << endl;
    cout << "13) Salir" << endl;
    cout << "****************************************" << endl;
    cout << " Selecciona una opcion:" << endl;

    int intAux;
    cin >> intAux;

    if(intAux < 1 || intAux > 13){
        cout << "Opcion no valida" << endl;
        return 0;
    }else{
        return intAux;
    }
    
}

