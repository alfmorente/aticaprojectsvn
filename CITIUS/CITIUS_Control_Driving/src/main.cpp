/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 27 de mayo de 2014, 13:38
 */

#include <cstdlib>
#include "CITIUS_Control_Driving/DrivingConnectionManager.h"

using namespace std;

int main(int argc, char** argv) {

    DrivingConnectionManager drivingManager = new DrivingConnectionManager("/dev/ttyUSB0");
    
    return 0;
}

void printMainMenu(){
    cout << "*******************************" << endl;
    cout << "Seleccione la opción que desee:" << endl;
    cout << "*******************************" << endl;
    cout << "1  - Intermitente derecho" << endl;
    cout << "2  - Intermitente izquierdo" << endl;
    cout << "3  - Luces de emergencia" << endl;
    cout << "4  - Luces de posicion" << endl;
    cout << "5  - Luces de cruce" << endl;
    cout << "6  - Luces de carretera" << endl;
    cout << "7  - Claxon" << endl;
    cout << "8  - Marcha" << endl;
    cout << "9  - Intermitente izquierdo" << endl;
    cout << "10 - Intermitente izquierdo" << endl;
 }

