

#include "simuladorDriving.h"

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

    // Generación de publicadores
  pub_com_teleop = n.advertise<Common_files::msg_com_teleop>("commands_clean", 1000);
  pub_fcn_aux = n.advertise<Common_files::msg_fcn_aux>("engBrake", 1000);
  pub_emergency_stop = n.advertise<Common_files::msg_emergency_stop>("emergSet", 1000);
  
  
    // Generación de subscriptores  
  sub_error = n.subscribe("error", 1000, fcn_sub_error);
  sub_switch = n.subscribe("switch",1000,fcn_switch);
  sub_backup = n.subscribe("backup",1000,fcn_backup);
  sub_info_stop = n.subscribe("infoStop",1000,fcn_sub_info_stop); 
  sub_emergency_stop = n.subscribe("emergInfo",1000,fcn_sub_emergency_stop);
  
  sleep (1);
  inicializaVariables();
      
  cont = 0;
  while (ros::ok()) {
      
      publicaComTeleop();
      //sleep(1);
      //if (cont_emergency == 1000){
         // sleep (10);
           //   publicaEmergencyStop();
             // cout << "\n\n\n\n\n\n\n";
             // cout << "Publicado\n\n\n\n";
             // cout << "\n\n\n\n\n\n\n";
      //}
      //cont_emergency++;
      //cout << "Cuenta: " << cont_emergency << endl;
      
      /*
      if (cont <= 1000) {
          publicaComTeleop();
          publicaEmergencyStop();
          cont++;
      }
      else if (cont > 1000){
          publicaFcnAux();
          cont++;
      }
      */
    ros::spinOnce();

  }

  return 0;
}

//FUNCIONES PROPIAS

void inicializaVariables() {
        acelerador = 100;
        velocidad = 0;
        freno = OFF;
        direccion = -100;
        marcha = 0;
        freno_mano = OFF;
        motor = OFF;
        lucesIR = OFF;
        luces = OFF;
        diferenciales = OFF;
        activacionLaser = OFF;
        cont_emergency = OFF;
        cont_engine_brake = OFF;
}

void publicaComTeleop() {
    ROS_INFO("PUBLICA COM TELEOP");
    for (int i=0; i <= 10; i++) {
          msg_com_teleop.id_element = i;
          switch (i) {
              case ID_REMOTE_THROTTLE:  // Acelerador;
                if (acelerador < OFF)
                  acelerador = 100;            
                msg_com_teleop.value = acelerador;
                acelerador = acelerador - 10;
                cout << "acelerador = " << msg_com_teleop.value << "\n";
                break;
                           
              case ID_REMOTE_BRAKE:  // Freno de servicio;
                if (freno > 100)
                    freno = OFF;            
                msg_com_teleop.value = freno;
                freno = freno + 10;
                cout << "Freno = " << msg_com_teleop.value << "\n";
                break;
              
              case ID_REMOTE_STEER:   // Direccion
                if (direccion > 100)
                    direccion = -100;            
                msg_com_teleop.value = direccion;
                direccion = direccion + 10;
                cout << "Direccion = " << msg_com_teleop.value << "\n";
                break;
          
              case ID_REMOTE_GEAR:   //Marcha
                if (marcha > 4) 
                    marcha = 0;
                msg_com_teleop.value = marcha;
                marcha = marcha + 1;
                cout << "Marcha = " << msg_com_teleop.value << "\n";
                break;
              
              case ID_REMOTE_HANDBRAKE:   //freno de mano
                if (freno_mano > ON)
                    freno_mano = OFF;
                msg_com_teleop.value = freno_mano;
                freno_mano = freno_mano + 1;
                cout << "Freno mano = " << msg_com_teleop.value << "\n";
                break;
              
              case ID_REMOTE_ENGINE:   //arrancar motor
                if (motor > ON)
                    motor = OFF;
                msg_com_teleop.value = motor;
                motor = motor + 1;
                cout << "Arrancar motor = " << msg_com_teleop.value << "\n";
                break;  
                
              case ID_REMOTE_LIGHT_IR:   //Luces IR
                if (lucesIR > ON)
                    lucesIR = OFF;
                msg_com_teleop.value = lucesIR;
                lucesIR = lucesIR + 1;
                cout << "Luces IR = " << msg_com_teleop.value << "\n";
                break;  
                
             case ID_REMOTE_LIGHT_STANDARD:   //Luces 
                if (luces > ON)
                    luces = OFF;
                msg_com_teleop.value = luces;
                luces = luces + 1;
                cout << "Luces = " << msg_com_teleop.value << "\n";
                break; 

             case ID_REMOTE_DIFF:   //Diferenciales 
                if (diferenciales > ON)
                    diferenciales = OFF;
                msg_com_teleop.value = diferenciales;
                diferenciales = diferenciales + 1;
                cout << "Diferenciales = " << msg_com_teleop.value << "\n";
                break; 

             case ID_REMOTE_ACT_LASER2D:   //Activacion Laser 
                if (activacionLaser > ON)
                    activacionLaser = OFF;
                msg_com_teleop.value = activacionLaser;
                activacionLaser = activacionLaser + 1;
                cout << "Laser = " << msg_com_teleop.value << "\n";
                break;                 
          }
          
       pub_com_teleop.publish(msg_com_teleop);   
      
      }
    cout << "\n";
}

void publicaEmergencyStop(){
    //ROS_INFO("PUBLICA EMERGENCY STOP");
       //cont_emergency++;
       //if (cont_emergency == 500){
           ROS_INFO("PUBLICA EMERGENCY STOP ------------------------------------------------------------------------------------- ON");
           msg_emergency_stop.value = ON;
           pub_emergency_stop.publish(msg_emergency_stop);   
       //} 
           /*
       else if(cont_emergency == 1000){
           ROS_INFO("PUBLICA EMERGENCY STOP ----------------- OFF");
           msg_emergency_stop.value = OFF;
           cont_emergency = OFF;
           pub_emergency_stop.publish(msg_emergency_stop);   
       }   */ 
    
}

void publicaFcnAux() {
    ROS_INFO("PUBLICA FCN AUX");
    switch (cont_engine_brake){
        case 0:
            ROS_INFO("ENGINE -------------- OFF Y BRAKE --------------- OFF");
            msg_fcn_aux.type_msg = OFF; // Engine
            msg_fcn_aux.value = OFF; // OFF
            pub_fcn_aux.publish(msg_fcn_aux); 

            //sleep (1000);

            msg_fcn_aux.type_msg = ON; // Brake
            msg_fcn_aux.value = OFF; // OFF
            pub_fcn_aux.publish(msg_fcn_aux); 
            break;
        
        case 1:
            ROS_INFO("ENGINE -------------- ON");
            msg_fcn_aux.type_msg = OFF; // Engine
            msg_fcn_aux.value = ON; // ON
            pub_fcn_aux.publish(msg_fcn_aux); 
            break;
    
        case 2:       // ACTIVO EL BRAKE Y DESACTIVO EL ENGINE
            ROS_INFO("ENGINE -------------- OFF Y BRAKE --------------- ON");
            msg_fcn_aux.type_msg = ON; // Brake
            msg_fcn_aux.value = ON; // ON
            pub_fcn_aux.publish(msg_fcn_aux); 

            //sleep (1000);

            msg_fcn_aux.type_msg = OFF; // Engine
            msg_fcn_aux.value = OFF; // OFF
            pub_fcn_aux.publish(msg_fcn_aux); 
            break;
    
        case 3:        // ACTIVO EL ENGINE (EL BRAKE SIGUE ACTIVO)
            ROS_INFO("ENGINE -------------- ON Y BRAKE --------------- ON");
            msg_fcn_aux.type_msg = OFF; // Brake
            msg_fcn_aux.value = ON; // ON
            pub_fcn_aux.publish(msg_fcn_aux);
            break;           
    
        case 4:        // DESACTIVO EL ENGINE Y DESACTIVO EL BRAKE
            ROS_INFO("ENGINE -------------- OFF Y BRAKE --------------- OFF");
            msg_fcn_aux.type_msg = OFF; // Engine
            msg_fcn_aux.value = OFF; // OFF
            pub_fcn_aux.publish(msg_fcn_aux); 

            //sleep (1000);

            msg_fcn_aux.type_msg = ON; // Brake
            msg_fcn_aux.value = OFF; // OFF
            pub_fcn_aux.publish(msg_fcn_aux);      
            break;
            
        case 5:
            cont = 0;
            cont_engine_brake = -1;
            break;
            
    }
    
    cont_engine_brake++;
    
}

//FUNCIONES DE SUBSCRIPCIONN

void fcn_sub_error (Common_files::msg_error msg_error) {
    
}

void fcn_switch (Common_files::msg_switch msg_switch){
    
}

void fcn_backup (Common_files::msg_backup msg_backup) {
    
}

void fcn_sub_info_stop (Common_files::msg_info_stop msg_info_stop) {
    
}

void fcn_sub_emergency_stop (Common_files::msg_emergency_stop msg_emergency_stop) {
    
}