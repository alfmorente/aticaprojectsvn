/* 
 * File:   TranslatorROSJAUS.h
 * Author: Carlos Amores
 *
 * Created on 17 de junio de 2014, 17:40
 */

#ifndef TRANSLATORROSJAUS_H
#define	TRANSLATORROSJAUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* TRANSLATORROSJAUS_H */

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"

// Set/Report Wrench Effort
#define PRESENCE_VECTOR_THROTTLE 0x0001
#define PRESENCE_VECTOR_STEER 0x0010
#define PRESENCE_VECTOR_BRAKE 0x0040

// Set/Report Discrete devices
#define PRESENCE_VECTOR_GEAR 0x04
#define PRESENCE_VECTOR_PARKING_BRAKE 0x02

// Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_PAN 0x0040
#define PRESENCE_VECTOR_CURRENT_TILT 0x0020
#define PRESENCE_VECTOR_CURRENT_ZOOM 0x0008

// Set Camera Pose
#define PRESENCE_VECTOR_PAN 0x20
#define PRESENCE_VECTOR_TILT 0x10
#define PRESENCE_VECTOR_ZOOM 0x04

// UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_LEVEL 0x01
#define PRESENCE_VECTOR_BATTERY_VOLTAGE 0x02
#define PRESENCE_VECTOR_BATTERY_CURRENT 0x04
#define PRESENCE_VECTOR_BATTERY_TEMPERATURE 0x08
#define PRESENCE_VECTOR_MOTOR_TEMPERATURE 0x10
#define PRESENCE_VECTOR_MOTOR_RPM 0x20
#define PRESENCE_VECTOR_ALARMS 0x40

// Report Velocity State
#define PRESENCE_VECTOR_CURRENT_CRUISING_SPEED 0x0008

// Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_RIGHT 0x10
#define PRESENCE_VECTOR_BLINKER_LEFT 0x80
#define PRESENCE_VECTOR_KLAXON 0x20
#define PRESENCE_VECTOR_DIPSP 0x02
#define PRESENCE_VECTOR_DIPSR 0x04
#define PRESENCE_VECTOR_DIPSS 0x01

class TranslatorROSJAUS{
public:
  TranslatorROSJAUS();
  JausMessage getJausMsgFromVehicleInfo(int subDest, int nodDest, short id_device, short value);
  JausMessage getJausMsgFromElectricInfo(int subDest, int nodDest, short id_device, short value);
  JausMessage getJausMsgFromCameraInfo(int subDest, int nodDest, short id_camera, short pan, short tilt, short zoom);
  JausMessage getJausMsgFromIRCameraInfo(int subDest, int nodDest, short zoom, short polarity);
  JausMessage getJausMsgFromTelemeterInfo(int subDest, int nodDest, short *ecs);
  JausMessage getJausMsgFromTVCamera(int subDest, int nodDest, short zoom, short focus, bool autofocus);
  JausMessage getJausMsgFromPositioner(int subDest, int nodDest, short pan, short tilt);
private:
  
};
