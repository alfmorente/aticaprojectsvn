#include "Modulo_Laser2D_Frontal/interaction.h"
#include "../../Common_files/include/Common_files/constant.h"

using namespace std;

int getOperationMode(int argc, char **argv){
    if(argc!=4){
        printCorrectSyntax();
        return 0;
    }
    int a = atoi(argv[1]);
    switch (a) {
        case OPERATION_MODE_DEBUG:
            cout << "ATICA SYSTEM MANAGEMENT :: Mode DEBUG enabled" << endl;
            break;
        case OPERATION_MODE_RELEASE:
            cout << "ATICA SYSTEM MANAGEMENT :: Mode RELEASE enabled" << endl;
            break;
        case OPERATION_MODE_SIMULATION:
            cout << "ATICA SYSTEM MANAGEMENT :: Mode SIMULATION enabled" << endl;
            break;
        default:
            printCorrectSyntax();
            return 0;
    }
    return a;
}

void printCorrectSyntax() {
    cout << "Invalid option. Syntax: ./gps [mode option]" << endl;
    cout << "Options: " << endl;
    cout << "1: Debug" << endl;
    cout << "2: Release" << endl;
    cout << "3: Simulation" << endl;
    cout << "---------------" << endl;
    cout << "Try again" << endl;
}
