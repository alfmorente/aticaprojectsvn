
#include "RosNode_Joystick.h"

RosNode_Joystick::RosNode_Joystick() {
  estados.aceleracion = 0;
  estados.arranque.valor = 0;
  estados.arranque.changing = false;
  estados.claxon.valor = 0;
  estados.claxon.changing = false;
  estados.direccion = 0;
  estados.freno_mano.valor = 0;
  estados.freno_mano.changing = false;
  estados.freno_servicio = 0;
  estados.hidraulico.valor = 0;
  estados.hidraulico.changing = false;
  estados.int_derecho.valor = 0;
  estados.int_derecho.changing = false;
  estados.int_izquierdo.valor = 0;
  estados.int_izquierdo.changing = false;
  estados.luces_cortas.valor = 0;
  estados.luces_cortas.changing = false;
  estados.luces_largas.valor = 0;
  estados.luces_largas.changing = false;
  estados.luces_posicion.valor = 0;
  estados.luces_posicion.changing = false;
  estados.marcha.valor = 0;
  estados.marcha.changingUp = false;
  estados.marcha.changingDown = false;
}

RosNode_Joystick::~RosNode_Joystick(){

}

void RosNode_Joystick::initROS(){
  ros::NodeHandle nh;
  subsJoystick = nh.subscribe("joy",1000,&RosNode_Joystick::fcn_sub_joy,this);
  pubCommand = nh.advertise<Common_files::msg_com_teleop>("commands_clean",1000);

}

void RosNode_Joystick::fcn_sub_joy(sensor_msgs::Joy msg){
  // Acelerador
  if(msg.axes.at(EJE_ACELERADOR)!=estados.aceleracion){
    printf("Cambio en el acelerador/freno:\n");
    if(msg.axes.at(EJE_ACELERADOR)!=1){
      //estados.aceleracion = msg.axes.at(EJE_ACELERADOR)*ACC_MAX;
        estados.aceleracion = (1-msg.axes.at(EJE_ACELERADOR))*ACC_MAX_XBOX;
      estados.freno_servicio = 0;
    }else if(msg.axes.at(EJE_FRENO)!=1){
      //estados.freno_servicio = -msg.axes.at(EJE_ACELERADOR)*ACC_MAX;
        estados.freno_servicio = (1-msg.axes.at(EJE_FRENO))*ACC_MAX_XBOX;
        estados.aceleracion = 0; 
    }else{
      estados.freno_servicio = 0;
      estados.aceleracion = 0; 
    }   
    Common_files::msg_com_teleop msg;
    msg.id_element = ID_REMOTE_THROTTLE;
    msg.value = estados.aceleracion;
    pubCommand.publish(msg);
    msg.id_element = ID_REMOTE_BRAKE;
    msg.value = estados.freno_servicio;
    pubCommand.publish(msg);
    printf("Acelerador a: %d\n",estados.aceleracion);
    printf("Freno de servicio a: %d\n\n",estados.freno_servicio);
  }
  
  // Direccion
  if(msg.axes.at(EJE_DIRECCION)!=estados.direccion){
    printf("Cambio en la direccion:\n");
    estados.direccion = -msg.axes.at(EJE_DIRECCION)*ACC_MAX;
    printf("Direccion a: %d\n\n",estados.direccion);
    Common_files::msg_com_teleop msg;
    msg.id_element = ID_REMOTE_STEER;
    msg.value = estados.direccion;
    pubCommand.publish(msg);
  }
  
  // Luces cortas
  if(msg.buttons.at(BTN_LUCES_CORTAS)==1){
    estados.luces_cortas.changing = true;
  }else{
    if (estados.luces_cortas.changing) {
      estados.luces_cortas.changing = false;
      estados.luces_cortas.valor = !estados.luces_cortas.valor;
      printf("Estado de luces cortas: %d\n\n", estados.luces_cortas.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_LIGHT_CONVENTIONAL;
      msg.value = estados.luces_cortas.valor;
      pubCommand.publish(msg);
    }
  }
  
  // Luces posicion
  if(msg.buttons.at(BTN_LUCES_POSICION)==1){
    estados.luces_posicion.changing = true;
  }else{
    if(estados.luces_posicion.changing){
      estados.luces_posicion.changing = false;
      estados.luces_posicion.valor = !estados.luces_posicion.valor;
      //printf("Estado de luces posicion: %d\n\n",estados.luces_posicion.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = DIPSP;
      msg.value = estados.luces_posicion.valor;
      //pubCommand.publish(msg);
    }
  }
  
  // Luces largas
  if(msg.buttons.at(BTN_LUCES_LARGAS)==1){
    estados.luces_largas.changing = true;
  }else{
    if(estados.luces_largas.changing){
      estados.luces_largas.changing = false;
      estados.luces_largas.valor = !estados.luces_largas.valor;
      //printf("Estado de luces largas: %d\n\n",estados.luces_largas.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = DIPSR;
      msg.value = estados.luces_largas.valor;
      //pubCommand.publish(msg);
    }
  }
  
  // Claxon
  if(msg.buttons.at(BTN_CLAXON)==1){
    estados.claxon.changing = true;
  }else{
    if(estados.claxon.changing){
      estados.claxon.changing = false;
      estados.claxon.valor = !estados.claxon.valor;
      //printf("Estado de claxon: %d\n\n",estados.claxon.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = KLAXON;
      msg.value = estados.claxon.valor;
      //pubCommand.publish(msg);
    }
  }
  
  // Freno de mano
  if(msg.buttons.at(BTN_FRENOMANO)==1){
    estados.freno_mano.changing = true;
  }else{
    if(estados.freno_mano.changing){
      estados.freno_mano.changing = false;
      estados.freno_mano.valor = !estados.freno_mano.valor;
      printf("Estado freno de mano: %d\n\n",estados.freno_mano.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_HANDBRAKE;
      msg.value = estados.freno_mano.valor;
      pubCommand.publish(msg);
    }
  }
  
  // Arranque
  if(msg.buttons.at(BTN_ARRANQUE)==1){
    estados.arranque.changing = true;
  }else{
    if(estados.arranque.changing){
      estados.arranque.changing = false;
      estados.arranque.valor = !estados.arranque.valor;
      printf("Estado de arranque: %d\n\n",estados.arranque.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_ENGINE;
      msg.value = estados.arranque.valor;
      pubCommand.publish(msg);
    }
  }
  
  // Hidraulico
  if(msg.buttons.at(BTN_HIDRAULICO)==1){
    estados.hidraulico.changing = true;
  }else{
    if(estados.hidraulico.changing){
      estados.hidraulico.changing = false;
      estados.hidraulico.valor = !estados.hidraulico.valor;
      printf("Estado de hidraulico: %d\n\n",estados.hidraulico.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_DIFF;
      msg.value = estados.hidraulico.valor;
      pubCommand.publish(msg);
    }
  }
  
  // Intermitente
  if(msg.axes.at(EJE_INTERMITENTE)==1){
    estados.int_izquierdo.changing = true;
  }else if(msg.axes.at(EJE_INTERMITENTE)==-1){
    estados.int_derecho.changing = true;
  }else{ 
    if(estados.int_izquierdo.changing){
      estados.int_izquierdo.changing = false;
      estados.int_izquierdo.valor = !estados.int_izquierdo.valor;
      //printf("Estado de intermitente izdo: %d\n\n",estados.int_izquierdo.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = BLINKER_LEFT;
      msg.value = estados.int_izquierdo.valor;
      //pubCommand.publish(msg);
    }
    if(estados.int_derecho.changing){
      estados.int_derecho.changing = false;
      estados.int_derecho.valor = !estados.int_derecho.valor;
      //printf("Estado de intermitente dcho: %d\n\n",estados.int_derecho.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = BLINKER_RIGHT;
      msg.value = estados.int_derecho.valor;
      //pubCommand.publish(msg);
    }
  }
  
  // Marcha
  if(msg.buttons.at(BTN_BAJAR_MARCHA)==1){
    estados.marcha.changingDown = true;
  }else{
    if(estados.marcha.changingDown){
      estados.marcha.changingDown = false;
      estados.marcha.valor--;
      if(estados.marcha.valor<0) estados.marcha.valor = 0;
      printf("Estado la marcha: %d\n\n",estados.marcha.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_GEAR;
      msg.value = estados.marcha.valor;
      pubCommand.publish(msg);
    }
  }
  if(msg.buttons.at(BTN_SUBIR_MARCHA)==1){
    estados.marcha.changingUp = true;
  }else{
    if(estados.marcha.changingUp){
      estados.marcha.changingUp = false;
      estados.marcha.valor++;
      if(estados.marcha.valor>2) estados.marcha.valor = 2;
      printf("Estado la marcha: %d\n\n",estados.marcha.valor);
      Common_files::msg_com_teleop msg;
      msg.id_element = ID_REMOTE_GEAR;
      msg.value = estados.marcha.valor;
      pubCommand.publish(msg);
    }
  }
  
  
}

