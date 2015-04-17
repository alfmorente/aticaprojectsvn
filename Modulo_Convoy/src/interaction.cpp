/**
  @file interaction.cpp
  @brief 

 * Archivo utilizado para elegir la versión del software en desarrollo
 * DEBUG, RELEASE o SIMULATION

  @author Carlos Amores
  @date 10/03/2014

*/

#include "../include/Modulo_Convoy/interaction.h"
#include "../../../src/Common_files/include/Common_files/constant.h"

using namespace std;

int getOperationMode(int argc, char **argv){
    if(argc!=2){
        printCorrectSyntax();
        return 0;
    }
    int a = atoi(argv[1]);
    switch (a) {
        case OPERATION_MODE_DEBUG:
            cout << "ATICA CONVOY :: Mode DEBUG enabled" << endl;
            break;
        case OPERATION_MODE_RELEASE:
            cout << "ATICA CONVOY :: Mode RELEASE enabled" << endl;
            break;
        case OPERATION_MODE_SIMULATION:
            cout << "ATICA CONVOY :: Mode SIMULATION enabled" << endl;
            break;
        default:
            printCorrectSyntax();
            return 0;
    }
    return a;
}

void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./gest_errores [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}
