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
}

TeachThread::~TeachThread() {

}

void TeachThread::DoWork() {
  teachSt->includeNameOfTeachFileLine();
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
      countData++;
    }
  }
  teachSt->includeFinalLine();
  teachSt->divideTeach();
  teaches = teachSt->getTeachStrings();
}

void TeachThread::setMode(bool in) {
  this->mode_active = in;
}

vector<string> TeachThread::getTeaches() {
  return teaches;
}
