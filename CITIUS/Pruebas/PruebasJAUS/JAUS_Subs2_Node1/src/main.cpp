/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 20 de noviembre de 2014, 12:37
 */

#include <cstdlib>
#include "JausController.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
  
  JausController *nodeComm = JausController::getInstance();

  nodeComm->initJAUS();
  
  while(true){
    usleep(1000000);
  }

  return 0;
}