
#include "MessageDispatcher.h"
#include "constant.h"

MessageDispatcher::MessageDispatcher() {

}

MessageDispatcher::~MessageDispatcher() {

}

// Throttle

void MessageDispatcher::sendThrottleInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para THROTTLE (-1:100):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = THROTTLE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Brake

void MessageDispatcher::sendBrakeInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para BRAKE (-1:100):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BRAKE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Handbrake

void MessageDispatcher::sendHandbrakeInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para HANDBRAKE (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = HANDBRAKE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Steering

void MessageDispatcher::sendSteeringInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para STEERING (-100:100):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = STEERING;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Gear

void MessageDispatcher::sendGearInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para GEAR (-1:2):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = GEAR;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Steering Alarms

void MessageDispatcher::sendSteeringAlarmsInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para STEERING ALARMS (0:65565):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = STEERING_ALARMS;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Drive Alarms

void MessageDispatcher::sendDriveAlarmsInfo(int socketDescriptor) {

  short value = 0;
  while((value < 1) || (value > 11)){
    cout << "Selecciona la alarma a enviar:" << endl;
    cout << "1 ) Todas las alarmas a 0" << endl;
    cout << "2 ) Fallo de conexion" << endl;
    cout << "3 ) Fallo en la direccion" << endl;
    cout << "4 ) Fallo de conexion con freno" << endl;
    cout << "5 ) Fallo en freno" << endl;
    cout << "6 ) Fallo de conexion con freno de estacionamiento" << endl;
    cout << "7 ) Fallo en freno de estacionamiento" << endl;
    cout << "8 ) Temperatura de motor" << endl;
    cout << "9 ) Fallo en testigos" << endl;
    cout << "10) Fallo en aceleracion" << endl;
    cout << "11) Fallo en cambio de marchas" << endl;
    cin >> value;
  }
  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_ALARMS;
  if (value == 1) {
    frame.value = MASK_NOT_ALARMS;
  } else if (value == 2) {
    frame.value = MASK_ALARMS_CONNECTION_STEERING_FAILED;
  } else if (value == 3) {
    frame.value = MASK_ALARMS_STEERING_FAILED;
  } else if (value == 4) {
    frame.value = MASK_ALARMS_BRAKE_CONNECTION_FAILED;
  } else if (value == 5) {
    frame.value = MASK_ALARMS_BRAKE_FAILED;
  } else if (value == 6) {
    frame.value = MASK_ALARMS_HANDBRAKE_CONNECTION_FALED;
  } else if (value == 7) {
    frame.value = MASK_ALARMS_HANDBRAKE_FAILED;
  } else if (value == 8) {
    frame.value = MASK_ALARMS_MOTOR_TEMPERATURE;
  } else if (value == 9) {
    frame.value = MASK_ALARMS_FLAGS_FAILED;
  } else if (value == 10) {
    frame.value = MASK_ALARMS_ACC_FAILED;
  } else if (value == 11) {
    frame.value = MASK_ALARMS_GEAR_FAILED;
  }
  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Blinker left

void MessageDispatcher::sendBlinkerLeftInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para BLINKER LEFT (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BLINKER_LEFT;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Blinker right

void MessageDispatcher::sendBlinkerRightInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para BLINKER RIGHT (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BLINKER_RIGHT;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Blinker emergency

void MessageDispatcher::sendEmergencyBlinkerInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para EMERGENCY BLINKER (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BLINKER_EMERGENCY;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Dipss

void MessageDispatcher::sendDipssInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para DIPSS (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DIPSS;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Dipsr

void MessageDispatcher::sendDipsrInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para DIPSR (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DIPSR;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Dipsp

void MessageDispatcher::sendDipspInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para DIPSP (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DIPSP;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Klaxon

void MessageDispatcher::sendKlaxonInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para KLAXON (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = KLAXON;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt throttle

void MessageDispatcher::sendMTThrottleInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_THOTTLE (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_THROTTLE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt brake

void MessageDispatcher::sendMTBrakeInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_BRAKE (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_BRAKE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt handbrake

void MessageDispatcher::sendMTHandbrakeInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_HANDBRAKE (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_HANDBRAKE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt steering

void MessageDispatcher::sendMTSteeringInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_STEERING (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_STEERING;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt gear

void MessageDispatcher::sendMTGearInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_GEAR (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_GEAR;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt blinkers

void MessageDispatcher::sendMTBlinkersInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_BLINKERS (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_BLINKERS;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Mt lights

void MessageDispatcher::sendMTLightsInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MT_LIGHTS (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MT_LIGHTS;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Motor rpm 

void MessageDispatcher::sendMotorRPMInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MOTOR RPM (-1:1000):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MOTOR_RPM;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Motor temperature 

void MessageDispatcher::sendMotorTemperatureInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para MOTOR TEMPERATURE (-1:400):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = MOTOR_TEMPERATURE;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}

// Crussing speed

void MessageDispatcher::sendCruissingSpeedInfo(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para CRUISSING SPEED (-1:1000):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CRUISING_SPEED;
  frame.value = value;

  //Buffer de envio 
  char buff[8];
  memcpy(&buff[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&buff[2], &frame.id_instruction, sizeof (frame.id_instruction));
  memcpy(&buff[4], &frame.element, sizeof (frame.element));
  memcpy(&buff[6], &frame.value, sizeof (frame.value));
  send(socketDescriptor, buff, sizeof (buff), 0);
  usleep(100);

}