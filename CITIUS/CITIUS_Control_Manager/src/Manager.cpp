
/** 
 * @file  Manager.cpp
 * @brief Implementación de la clase "Manager"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "Manager.h"

/**
 * Constructor de la clase. Inicializa los flags de estados de nodos secundarios
 */
Manager::Manager() {
  communication = false;
  positionOrientationOK = false;
  frontCameraOK = false;
  rearCameraOK = false;
  electricOK = false;
  irCameraOK = false;
  lrfOK = false;
  tvCameraOK = false;
  positionerOK = false;
  currentSwitcher = SWITCHER_INIT;
  turnOffChecker = new TurnOffAlright();
}

/**
 * Destructor de la clase
 */
Manager::~Manager() {
  delete(turnOffChecker);
}

/**
 * Método público inicializador de artefactos ROS de la clase
 */
void Manager::initROS() {
  ros::NodeHandle nh;
  cmNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("cmNodeStatus");
  poNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("poNodeStatus");
  fcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("fcNodeStatus");
  rcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("rcNodeStatus");
  drNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("drNodeStatus");
  elNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("elNodeStatus");
  irNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateIRCamera");
  lrfNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateLRF");
  tvNosdeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStateTVCamera");
  ptNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("nodeStatePanTilt");
  serverVehicleStatus = nh.advertiseService("vehicleStatus", &Manager::fcv_serv_vehicleStatus, this);
  switcherLocalTelecontrol = nh.subscribe("switcher", 1000, &Manager::fnc_subs_switcher, this);
  pubLastExec =  nh.advertise<CITIUS_Control_Manager::msg_lastExec>("lastExec", 1000);
}

/**
 * Método privado que implementa el servidor de estados del vehículo. Recibe 
 * las peticiones de transición y las ejecuta/deniega segun la lógica establecida
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición de servicio
 */
bool Manager::fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp) {
  ros::NodeHandle nh;
  int currentStatus;
  nh.getParam("vehicleStatus", currentStatus);
  short numOfAttemps = 0;
  CITIUS_Control_Manager::srv_nodeStatus service;
  switch (rq.status) {
    case OPERATION_MODE_INICIANDO:
      ROS_INFO("[Control] Manager - Modo de operacion INICIANDO activado");
      nh.setParam("vehicleStatus", OPERATION_MODE_INICIANDO);
      turnOffChecker->clearFile();
      turnOffChecker->setStatusLine("INICIANDO");
      service.request.status = NODESTATUS_OK;
      while (!cmNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo Communication...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Communication no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Communication arrancado correctamente");
          communication = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo Communication no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!poNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo Position/Orientation...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Position/Orientation no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Position/Orientation arrancado correctamente");
          positionOrientationOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo Position/Orientation no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!fcNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo FrontCamera...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo FrontCamera no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo FrontCamera arrancado correctamente");
          frontCameraOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo FrontCamera no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!rcNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo RearCamera...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo RearCamera no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo RearCamera arrancado correctamente");
          rearCameraOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo RearCamera no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!elNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo Electric...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Electric no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Electric arrancado correctamente");
          electricOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo Driving no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!irNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo IR Camera...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo IR Camera no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo IR Camera arrancado correctamente");
          irCameraOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo IR Camera no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!lrfNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo LRF...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo LRF no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo LRF arrancado correctamente");
          lrfOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo LRF no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!tvNosdeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo Camera TV...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Camera TV no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Camera TV arrancado correctamente");
          tvCameraOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo Camera TV no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      numOfAttemps = 0;
      while (!ptNodeStatus.call(service) && numOfAttemps < MAX_ATTEMPS) {
        ROS_INFO("[Control] Manager - Reintentando conexion con nodo Positioner...");
        numOfAttemps++;
      }
      if (numOfAttemps < MAX_ATTEMPS) {
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Positioner no pudo arrancar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Positioner arrancado correctamente");
          positionerOK = true;
        }
      } else {
        ROS_INFO("[Control] Manager - Nodo Positioner no pudo arrancar. Cumplido numero maximo de reintentos");
      }
      // Comprobacion de errores
      if (!electricOK) {
        ROS_INFO("[Control] Manager - Sin nodo Electric no hay vehiculo. Finalizado");
        nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);
        turnOffChecker->setStatusLine("APAGANDO");
        rsp.confirmation = false;
      } else {
        if (!positionOrientationOK || !frontCameraOK || !rearCameraOK || !irCameraOK || !lrfOK || !tvCameraOK || !positionerOK) {
          ROS_INFO("[Control] Manager - Algun dispositivo fallo pero se continua con la ejecucion");
        } else {
          ROS_INFO("[Control] Manager - Todos los nodos arrancaron correctamente");
        }
        if (rq.posSwitcher == SWITCHER_LOCAL) {
          ROS_INFO("[Control] Manager - Modo de operacion LOCAL activado");
          nh.setParam("vehicleStatus", OPERATION_MODE_LOCAL);
          turnOffChecker->setStatusLine("LOCAL");
        } else if (rq.posSwitcher == SWITCHER_TELECONTROL) {
          ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
          nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
          turnOffChecker->setStatusLine("CONDUCCION");
        }
        rsp.confirmation = true;
      }
      break;
    case OPERATION_MODE_LOCAL:
      ROS_INFO("[Control] Manager - Peticion de entrada en modo LOCAL denegada");
      rsp.confirmation = false;
      break;
    case OPERATION_MODE_CONDUCCION:
      if (currentStatus == OPERATION_MODE_OBSERVACION) {
        nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
        turnOffChecker->setStatusLine("CONDUCCION");
        ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
        rsp.confirmation = true;
      } else {
        rsp.confirmation = false;
        ROS_INFO("[Control] Manager - Peticion de entrada en modo CONDUCCION denegada");
      }
      break;
    case OPERATION_MODE_OBSERVACION:
      if (currentStatus == OPERATION_MODE_CONDUCCION) {
        nh.setParam("vehicleStatus", OPERATION_MODE_OBSERVACION);
        turnOffChecker->setStatusLine("OBSERVACION");
        rsp.confirmation = true;
        ROS_INFO("[Control] Manager - Modo de operacion OBSERVACION activado");
      } else {
        rsp.confirmation = false;
        ROS_INFO("[Control] Manager - Peticion de entrada en modo OBSERVACION denegada");
      }
      break;
    case OPERATION_MODE_APAGANDO:
      service.request.status = NODESTATUS_OFF;
      if (communication) {
        ROS_INFO("[Control] Manager - Apagando nodo Communication");
        while (!cmNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Communication no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Communication apagado correctamente");
        }
      }
      ROS_INFO("[Control] Manager - Apagando nodo Driving");
        while (!drNodeStatus.call(service))
          if (!service.response.confirmation) {
            ROS_INFO("[Control] Manager - Nodo Driving no se pudo apagar");
          } else {
            ROS_INFO("[Control] Manager - Nodo Driving apagado correctamente");
          }
      if (positionOrientationOK) {
        ROS_INFO("[Control] Manager - Apagando nodo Position/Orientation");
        while (!poNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Position/Orientation no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Position/Orientation apagado correctamente");
        }
      }
      if (frontCameraOK) {
        ROS_INFO("[Control] Manager - Apagando nodo FrontCamera");
        while (!fcNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo FrontCamera no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo FrontCamera apagado correctamente");
        }
      }
      if (rearCameraOK) {
        ROS_INFO("[Control] Manager - Apagando nodo RearCamera");
        while (!rcNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo RearCamera no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo RearCamera apagado correctamente");
        }
      }
      if (irCameraOK) {
        ROS_INFO("[Control] Manager - Apagando nodo IRCamera");
        while (!irNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo IRCamera no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo IRCamera apagado correctamente");
        }
      }
      if (lrfOK) {
        ROS_INFO("[Control] Manager - Apagando nodo LRF");
        while (!lrfNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo LRF no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo LRF apagado correctamente");
        }
      }
      if (tvCameraOK) {
        ROS_INFO("[Control] Manager - Apagando nodo TVCamera");
        while (!tvNosdeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo TVCamera no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo TVCamera apagado correctamente");
        }
      }
      if (positionerOK) {
        ROS_INFO("[Control] Manager - Apagando nodo Positioner");
        while (!ptNodeStatus.call(service));
        if (!service.response.confirmation) {
          ROS_INFO("[Control] Manager - Nodo Positioner no se pudo apagar");
        } else {
          ROS_INFO("[Control] Manager - Nodo Positioner apagado correctamente");
        }
      }
      rsp.confirmation = true;
      nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);
      turnOffChecker->setStatusLine("APAGANDO");
      break;
    default:
      rsp.confirmation = false;
      break;
  };
  return true;
}

/**
 * Método privado receptor de mensajes ROS ante cambios en la posición del 
 * conmutador local / teleoperado. Cambios en el conmutador acarrean cambio de 
 * estado en el modo de operación
 * @param[in] msg Mensaje ROS con la nueva posición del conmutador
 */
void Manager::fnc_subs_switcher(CITIUS_Control_Manager::msg_switcher msg) {
  ros::NodeHandle nh;
  int status;
  nh.getParam("vehicleStatus", status);
  if (status == OPERATION_MODE_CONDUCCION || status == OPERATION_MODE_LOCAL || status == OPERATION_MODE_OBSERVACION) {
    if (msg.switcher == SWITCHER_LOCAL) {
      ROS_INFO("[Control] Manager - Modo de operacion LOCAL activado");
      nh.setParam("vehicleStatus", OPERATION_MODE_LOCAL);
      turnOffChecker->setStatusLine("LOCAL");
      currentSwitcher = SWITCHER_LOCAL;
    } else if (msg.switcher == SWITCHER_TELECONTROL) {
      ROS_INFO("[Control] Manager - Modo de operacion CONDUCCION activado");
      nh.setParam("vehicleStatus", OPERATION_MODE_CONDUCCION);
      turnOffChecker->setStatusLine("CONDUCCION");
      currentSwitcher = SWITCHER_TELECONTROL;
    }
  }
}

/**
 * Método público que comprueba que la última ejecución del programa llegó a su
 * fin de manera ordenada y envía un mensaje de alarmas para conocimiento del
 * controlador en caso contrario
 * @return 
 */
void Manager::checkPreviousExec() {
  if(!turnOffChecker->checkCorrectTurnedOff()){
    CITIUS_Control_Manager::msg_lastExec msg;
    msg.badExec = true;
    pubLastExec.publish(msg);
  }
}
