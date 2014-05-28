/* 
 * File:   MenuHandler.cpp
 * Author: atica
 *
 * Created on 28 de mayo de 2014, 11:39
 */

#include "CITIUS_Control_Driving/MenuHandler.h"

using namespace std;
/*
 * 
 */

MenuHandler::MenuHandler() {

}

short MenuHandler::printMainMenu(){
    short selectedOption = 0;
    while(selectedOption<=0 || selectedOption>=15){
        cout << "*********************************************" << endl;
        cout << "Seleccione la opción que desee y pulse INTRO:" << endl;
        cout << "*********************************************" << endl;
        cout << "0  - Intermitente derecho" << endl;
        cout << "1  - Intermitente izquierdo" << endl;
        cout << "2  - Luces de emergencia" << endl;
        cout << "3  - Luces de posicion" << endl;
        cout << "4  - Luces de cruce" << endl;
        cout << "5  - Luces de carretera" << endl;
        cout << "6  - Claxon" << endl;
        cout << "7  - Marcha" << endl;
        cout << "8  - Acelerador" << endl;
        cout << "10 - Velocidad de crucero" << endl;
        cout << "12 - Freno de mano" << endl;
        cout << "13 - Freno" << endl;
        cout << "14 - Direccion" << endl;
        cout << "*********************************************" << endl;
        cout << "Opción: ";
        cin >> selectedOption;
    }
    return selectedOption;    
 }

short MenuHandler::enterValueForDevice() {
    short value = 0;
    cout << endl;
    cout << "*********************************************" << endl;
    cout << "Seleccione un valor para el dispositivo:" << endl;
    cout << "*********************************************" << endl;
    cout << "Valor: ";
    cin >> value;
    return value;    
 }
