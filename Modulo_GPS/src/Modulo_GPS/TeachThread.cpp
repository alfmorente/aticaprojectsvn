/* 
 * File:   ConduccionThread.cpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#include "Modulo_GPS/TeachThread.hpp"
#include <queue>
#include <iostream>

using namespace std;

TeachThread::TeachThread() {
    this->mode_active = true;
    teachSt = new TeachString();
    ready = false;
}

TeachThread::~TeachThread() {

}

void TeachThread::DoWork() {
    //teaches.clear();  
    ready = false;
    //printf("Arrancando hilo de teach!\n");
    teachSt->includeNameOfTeachFileLine();
    //printf("Incluida cabecera teach");
    int countData = 0;
    TeachData rcvData;
    while (mode_active) {
        if (!queueGPSdata.empty()) {
            rcvData = queueGPSdata.front();
            queueGPSdata.pop();
            if (countData == 0) {
                teachSt->includeWPLine(rcvData.latitude, rcvData.longitude, true);
            } else {
                teachSt->includeWPLine(rcvData.latitude, rcvData.longitude, false);
            }
            //printf("Incluido punto %d\n", countData);
            countData++;
        }
    }
    //printf("Modulo desactivado\n");
    teachSt->includeFinalLine();
    //printf("Incluida linea final teach\n");
    teachSt->divideTeach();
    //printf("Dividido string teach\n");
    teaches = teachSt->getTeachStrings();
    //printf("Relleno variable teaches\n");
    ready = true;
    //printf("Ready = true\n");
}

void TeachThread::setMode(bool in) {
    this->mode_active = in;
}

vector<string> TeachThread::getTeaches() {
    return teaches;
}

bool TeachThread::dataReady() {
    return ready;
}
