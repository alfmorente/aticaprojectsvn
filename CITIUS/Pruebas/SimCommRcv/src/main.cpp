/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 9 de septiembre de 2014, 12:11
 */

#include "JausController.h"
#include <cstdlib>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
    

    JausController *nodeComm = JausController::getInstance();

    nodeComm->initJAUS();

    return 0;
}
