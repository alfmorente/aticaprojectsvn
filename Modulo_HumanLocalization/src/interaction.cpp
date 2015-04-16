/**
  @file interaction.cpp
  @brief 

 * Archivo utilizado para elegir la versión del software en desarrollo
 * DEBUG, RELEASE o SIMULATION

  @author Carlos Amores
  @date 10/03/2014

*/
#include "../include/Modulo_HumanLocalization/interaction.h"
#include "../../../src/Common_files/include/Common_files/constant.h"

using namespace std;

/**
 * Función principal que elige una versión del software dependiendo del 
 * argumento de entrada
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Devuelve 0 si hay errores o número de versión si no hay errores
 */
int getOperationMode(int argc, char **argv){
    if(argc!=2){
        printCorrectSyntax();
        return 0;
    }
    int a = atoi(argv[1]);
    switch (a) {
        case OPERATION_MODE_DEBUG:
            cout << "ATICA ERROR_MANAGEMENT :: Mode DEBUG enabled" << endl;
            break;
        case OPERATION_MODE_RELEASE:
            cout << "ATICA ERROR_MANAGEMENT :: Mode RELEASE enabled" << endl;
            break;
        case OPERATION_MODE_SIMULATION:
            cout << "ATICA ERROR_MANAGEMENT :: Mode SIMULATION enabled" << endl;
            break;
        default:
            printCorrectSyntax();
            return 0;
    }
    return a;
}

/**
 * Muestra por pantalla que se ha producido un error al elegir versión.
 * Presenta la manera de ejecutar el programa correctamente
 */
void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./gest_errores [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}
