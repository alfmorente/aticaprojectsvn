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
