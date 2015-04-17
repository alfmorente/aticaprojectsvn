/**
  @file TeachThread.cpp
  @brief Implementación de la clase TeachThread
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "Modulo_GPS/TeachThread.hpp"
#include <queue>
#include <iostream>

using namespace std;

/**
 * Constructor de la clase
 */
TeachThread::TeachThread() {
    this->mode_active = true;
    teachSt = new TeachString();
    ready = false;
}

/**
 * Destructor de la clase
 */
TeachThread::~TeachThread() {

}

/**
 * Método principal de ejecución del hilo
 */
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

/**
 * Método público modificador del atributo mode_active 
 * @param in Booleano que indica si activar o desactivar el modo Teach (y por lo
 * tanto la ejecución del hilo)
 */
void TeachThread::setMode(bool in) {
    this->mode_active = in;
}

/**
 * Consultor del atributo teaches
 * @return Atributo teaches
 */
vector<string> TeachThread::getTeaches() {
    return teaches;
}

/**
 * Consultor del atributo ready que indica si se ha realizado la segmentación y
 * por lo tanto la ruta en sus distintas partes está lista para ser publicada
 * @return Atributo ready
 */
bool TeachThread::dataReady() {
    return ready;
}
