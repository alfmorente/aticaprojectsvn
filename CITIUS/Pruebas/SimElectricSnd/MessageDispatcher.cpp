
#include "MessageDispatcher.h"
#include "constant.h"

MessageDispatcher::MessageDispatcher() {

}

MessageDispatcher::~MessageDispatcher(){

}

void MessageDispatcher::sendBatteryLevelMsg(int socketDescriptor) {
  short value;
  cout << "Selecciona el valor para BATTERY LEVEL (-1:100):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BATTERY_LEVEL;
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

void MessageDispatcher::sendBatteryVoltageMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para BATTERY VOLTAGE (-1:100):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BATTERY_VOLTAGE;
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

void MessageDispatcher::sendBatteryCurrentMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para BATTERY CURRENT (-1:300):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BATTERY_CURRENT;
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

void MessageDispatcher::sendBatteryTemperatureMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para BATTERY TEMPERATURE (-1:200):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = BATTERY_TEMPERATURE;
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

void MessageDispatcher::sendSupplyCheckMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para SUPPLY CHECK (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_CHECK;
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

void MessageDispatcher::sendTurnOffMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para TURN OFF (1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = TURN_OFF;
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

void MessageDispatcher::sendSupply5Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para SUPPLY 5 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_5;
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

void MessageDispatcher::sendSupply12Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para SUPPLY 12 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_12;
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

void MessageDispatcher::sendSupply24DriveMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para SUPPLY 24 DRIVE (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_24_DRIVE;
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

void MessageDispatcher::sendSupply24OCCMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para SUPPLY 24 OCC (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_24_OCC;
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

void MessageDispatcher::sendControlSystemSupplyMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para CONTROL SYSTEM SUPPLY (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CONTROL_SYSTEM_SUPPLY;
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

void MessageDispatcher::sendControlSystemSupply5Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para CONTROL SYSTEM SUPPLY 5 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CONTROL_SYSTEM_SUPPLY_5;
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

void MessageDispatcher::sendControlSystemSupply12Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para CONTROL SYSTEM SUPPLY 12 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CONTROL_SYSTEM_SUPPLY_12;
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

void MessageDispatcher::sendControlSystemSupply24Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para CONTROL SYSTEM SUPPLY 24 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CONTROL_SYSTEM_SUPPLY_24;
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

void MessageDispatcher::sendControlSystemSupply48Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para CONTROL SYSTEM SUPPLY 48 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = CONTROL_SYSTEM_SUPPLY_48;
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

void MessageDispatcher::sendDriveSystemSupplyMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para DRIVE SYSTEM SUPPLY (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_SYSTEM_SUPPLY;
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

void MessageDispatcher::sendDriveSystemSupply5Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para DRIVE SYSTEM SUPPLY 5 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_SYSTEM_SUPPLY_5;
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

void MessageDispatcher::sendDriveSystemSupply12Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para DRIVE SYSTEM SUPPLY 12 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_SYSTEM_SUPPLY_12;
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

void MessageDispatcher::sendDriveSystemSupply24Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para DRIVE SYSTEM SUPPLY 24 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_SYSTEM_SUPPLY_24;
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

void MessageDispatcher::sendDriveSystemSupply48Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para DRIVE SYSTEM SUPPLY 48 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = DRIVE_SYSTEM_SUPPLY_48;
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

void MessageDispatcher::sendCommSystemSupplyMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para COMM SYSTEM SUPPLY (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = COMM_SYSTEM_SUPPLY;
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

void MessageDispatcher::sendCommSystemSupply5Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para COMM SYSTEM SUPPLY 5 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = COMM_SYSTEM_SUPPLY_5;
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

void MessageDispatcher::sendCommSystemSupply12Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para COMM SYSTEM SUPPLY 12(-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = COMM_SYSTEM_SUPPLY_12;
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

void MessageDispatcher::sendCommSystemSupply24Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para COMM SYSTEM SUPPLY 24 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = COMM_SYSTEM_SUPPLY_24;
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

void MessageDispatcher::sendCommSystemSupply48Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para COMM SYSTEM SUPPLY 48 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = COMM_SYSTEM_SUPPLY_48;
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

void MessageDispatcher::sendObservationSystemSupplyMsg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para OBSERVATION SYSTEM SUPPLY (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = OBSERVATION_SYSTEM_SUPPLY;
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

void MessageDispatcher::sendObservationSystemSupply5Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para OBSERVATION SYSTEM SUPPLY 5 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = OBSERVATION_SYSTEM_SUPPLY_5;
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

void MessageDispatcher::sendObservationSystemSupply12Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para OBSERVATION SYSTEM SUPPLY 12 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = OBSERVATION_SYSTEM_SUPPLY_12;
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

void MessageDispatcher::sendObservationSystemSupply24Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para OBSERVATION SYSTEM SUPPLY 24 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = OBSERVATION_SYSTEM_SUPPLY_24;
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

void MessageDispatcher::sendObservationSystemSupply48Msg(int socketDescriptor){
  short value;
  cout << "Selecciona el valor para OBSERVATION SYSTEM SUPPLY 48 (-1:1):" << endl;
  cin >> value;

  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = OBSERVATION_SYSTEM_SUPPLY_48;
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

void MessageDispatcher::sendSupplyAlarmsMsg(int socketDescriptor){
    short value = 0;
  while((value < 1) || (value > 13)){
    cout << "Selecciona la alarma a enviar:" << endl;
    cout << "1 ) Todos los bits a 0" << endl;
    cout << "2 ) Temperatura de baterias" << endl;
    cout << "3 ) Tension de baterias" << endl;
    cout << "4 ) Corriente de baterias" << endl;
    cout << "5 ) Fallo DC/DC 48V" << endl;
    cout << "6 ) Fallo DC/DC 24V (conduccion)" << endl;
    cout << "7 ) Fallo DC/DC 24V (resto)" << endl;
    cout << "8 ) Fallo DC/DC 12V" << endl;
    cout << "9 ) Fallo DC/DC 5V" << endl;
    cout << "10) Fallo en Subs. Control" << endl;
    cout << "11) Fallo en Subs. Payload conduccion" << endl;
    cout << "12) Fallo en Subs. Comunicaciones" << endl;
    cout << "13) Fallo en Subs. Payload observacion" << endl;
    cin >> value;
  }
  FrameDriving frame;
  frame.instruction = INFO;
  frame.id_instruction = -1;
  frame.element = SUPPLY_ALARMS;
  if (value == 1) {
    frame.value = MASK_NOT_ALARMS;
  } else if (value == 2) {
    frame.value = MASK_ALARMS_BATTERY_TEMPERATURE;
  } else if (value == 3) {
    frame.value = MASK_ALARMS_BATTERY_VOLTAGE;
  } else if (value == 4) {
    frame.value = MASK_ALARMS_BATTERY_CURRENT;
  } else if (value == 5) {
    frame.value = MASK_ALARMS_48V_FAILED;
  } else if (value == 6) {
    frame.value = MASK_ALARMS_24V_DRIVING_FAILED;
  } else if (value == 7) {
    frame.value = MASK_ALARMS_24V_FAILED;
  } else if (value == 8) {
    frame.value = MASK_ALARMS_12V_FAILED;
  } else if (value == 9) {
    frame.value = MASK_ALARMS_5V_FAILED;
  } else if (value == 10) {
    frame.value = MASK_ALARMS_CONTROL_FAILED;
  } else if (value == 11) {
    frame.value = MASK_ALARMS_DRIVING_FAILED;
  } else if (value == 12) {
    frame.value = MASK_ALARMS_COMM_FAILED;
  } else if (value == 13) {
    frame.value = MASK_ALARMS_OBSERVATION_FAILED;
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
