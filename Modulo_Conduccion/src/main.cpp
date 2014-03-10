/* 
 * File:   main.cpp
 * Author: Sergio Doctor López
 *
 * Created on 5 de febrero de 2014, 12:03
 */


#include <libpcan.h>
#include <iostream>
#include "../include/Modulo_Conduccion/Central.hpp"

int main(int argc, char** argv) {

    Central c;
    
    // Si se han podido establecer las comunicaciones se inicia el programa
    if (c.CENTRAL_ACTIVE)
        c.Main(argc, argv);
    else
        cout << "\n\n IMPOSIBLE ESTABLECER COMUNICACIONES \n\n";
    
    cout << "\n\n Procediendo a finalización ordenada del programa \n";

    return 0;
    
    
}
