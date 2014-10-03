
/** @file  TranslatorROSJAUS.h
 * @brief Declara el tipo de la clase "TranslatorROSJAUS"
        - La clase implementa la traducción entre mensajes ROS y JAUS como puente
        entre el Subsistema C2 y UGV de la arquitectura JAUS.
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef TRANSLATORROSJAUS_H
#define	TRANSLATORROSJAUS_H

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"

#endif	/* TRANSLATORROSJAUS_H */


class TranslatorROSJAUS{
public:
  TranslatorROSJAUS();
  JausMessage getJausMsgFromWrenchEffortInfo(int subDest, int nodDest, short steer, short throttle, short brake);
  JausMessage getJausMsgFromDiscreteDeviceInfo(int subDest, int nodDest, bool parkingbrake, short gear);
  JausMessage getJausMsgFromTravelSpeedInfo(int subDest, int nodDest, short speed);
  JausMessage getJausMsgFromUGVInfo(int subDest, int nodDest, short motorRPM, short motorTemperature);
  JausMessage getJausMsgFromSignalingInfo(int subDest, int nodDest, bool blinker_left, bool blinker_right, bool dipsp, bool dipss, bool dipsr, bool klaxon);
  
  JausMessage getJausMsgFromElectricInfo(int subDest, int nodDest, short bat_level, short bat_voltage, short bat_current, short bat_temp, short alarms);
  JausMessage getJausMsgFromCameraInfo(int subDest, int nodDest, short id_camera, short pan, short tilt, short zoom);
  JausMessage getJausMsgFromIRCameraInfo(int subDest, int nodDest, short zoom, short polarity);
  JausMessage getJausMsgFromTelemeterInfo(int subDest, int nodDest, short *ecs);
  JausMessage getJausMsgFromTVCamera(int subDest, int nodDest, short zoom, short focus, bool autofocus);
  JausMessage getJausMsgFromPositioner(int subDest, int nodDest, short pan, short tilt);
private:
  
};
