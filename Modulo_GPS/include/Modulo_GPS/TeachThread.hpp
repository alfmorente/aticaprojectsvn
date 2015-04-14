/* 
 * File:   ConduccionThread.hpp
 * Author: Sergio Doctor López
 *
 * Created on 6 de febrero de 2014
 */

#ifndef TEACHTHREAD_HPP
#define	TEACHTHREAD_HPP



#include "Thread.hpp"
#include <queue>
#include <cstdlib>
#include <tinyxml.h>
#include <sstream>
#include <string>
#include "TeachString.h"

typedef struct {
  float latitude;
  float longitude;
} TeachData;

class TeachThread : public Thread {
public:
  TeachThread();
  virtual ~TeachThread();
  virtual void DoWork();
  void setMode(bool);

  // Atributos cola de datos
  queue<TeachData> queueGPSdata;
  vector<string> getTeaches();


private:
  TeachString *teachSt;
  bool mode_active; // Flag de estado
  vector<string> teaches;

};

#endif	/* TEACHTHREAD_HPP */

