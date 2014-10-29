
/** 
 * @file  TranslatorROSJAUS.h
 * @brief Declara el tipo de la clase "TranslatorROSJAUS"
 * - La clase implementa la traducción entre mensajes ROS y JAUS como puente
 * entre el Subsistema C2 y UGV de la arquitectura JAUS.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup JAUSModule
 * @{
 */

#ifndef TRANSLATORROSJAUS_H
#define	TRANSLATORROSJAUS_H

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"
#include "constant.h"
#include "message/experimental/ugvInfo12Message.h"
#include "message/experimental/reportSignalingElements25Message.h"

/**
 * \class TranslatorROSJAUS
 * \brief Clase que representa el generador de mensajes JAUS globales a partir
 * de los datos concretos que componen un mensaje específico
 */
class TranslatorROSJAUS {
public:
  TranslatorROSJAUS();
  // Obtencion de mensajes JAUS en base a informacion previa
  JausMessage getJausMsgFromWrenchEffortInfo(JausSubsystemID subDest, JausNodeID nodDest, short steer, short throttle, short brake);
  JausMessage getJausMsgFromDiscreteDeviceInfo(JausSubsystemID subDest, JausNodeID nodDest, bool parkingbrake, short gear);
  JausMessage getJausMsgFromTravelSpeedInfo(JausSubsystemID subDest, JausNodeID nodDest, short speed);
  JausMessage getJausMsgFromUGVInfo(JausSubsystemID subDest, JausNodeID nodDest, short motorRPM, short motorTemperature);
  JausMessage getJausMsgFromSignalingInfo(JausSubsystemID subDest, JausNodeID nodDest, bool blinker_left, bool blinker_right, bool dipsp, bool dipss, bool dipsr, bool klaxon);
  JausMessage getJausMsgFromElectricInfo(JausSubsystemID subDest, JausNodeID nodDest, short bat_level, short bat_voltage, short bat_current, short bat_temp, short alarms);
  JausMessage getJausMsgFromCameraInfo(JausSubsystemID subDest, JausNodeID nodDest, CameraID id_camera, short pan, short tilt, short zoom);
  JausMessage getJausMsgFromIRCameraInfo(JausSubsystemID subDest, JausNodeID nodDest, short zoom, short polarity);
  JausMessage getJausMsgFromTelemeterInfo(JausSubsystemID subDest, JausNodeID nodDest, short *ecs);
  JausMessage getJausMsgFromTVCamera(JausSubsystemID subDest, JausNodeID nodDest, short zoom, short focus, bool autofocus);
  JausMessage getJausMsgFromPositioner(JausSubsystemID subDest, JausNodeID nodDest, short pan, short tilt);
};

#endif	/* TRANSLATORROSJAUS_H */

/**
 * @}
 */