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

//Presences Vectors
#define PRESENCE_VECTOR_THROTTLE 0X0001
#define PRESENCE_VECTOR_STEER 0X0010
#define PRESENCE_VECTOR_BRAKE 0X0040
 
#define PRESENCE_VECTOR_ENGINE 0X01
#define PRESENCE_VECTOR_PARKING_BRAKE 0X02
#define PRESENCE_VECTOR_LIGHT_IR 0X22
#define PRESENCE_VECTOR_LIGHT_CONVENTIONAL 0X32
#define PRESENCE_VECTOR_DIFERENTIAL_LOCK 0X42
#define PRESENCE_VECTOR_ENABLE_LASER2D 0X52
#define PRESENCE_VECTOR_GEAR 0X04
 
#define PRESENCE_VECTOR_PAN 0X01
#define PRESENCE_VECTOR_TILT 0X02
#define PRESENCE_VECTOR_HOME 0X03
#define PRESENCE_VECTOR_ZOOM 0X04

class TranslatorROSJAUS{
public:
  TranslatorROSJAUS();
  JausMessage getJausMsgFromVehicleInfo(int subDest, int nodDest, short id_device, short value);
  JausMessage getJausMsgFromElectricInfo(int subDest, int nodDest, short id_device, short value);
  JausMessage getJausMsgFromCameraInfo(int subDest, int nodDest, short id_camera, short pan, short tilt);
  JausMessage getJausMsgFromPosOriInfo(int subDest, int nodDest, double lat, double lon, double alt, double roll, double yaw, double pitch);
  JausMessage getJausMsgFromStatus(int subDest, int nodDest, int status);
private:
  
};
