#include "../include/Modulo_Navegacion/navegacion.h"

using namespace std;

// Publicadores globales
ros::Publisher pub_modo;
ros::Publisher pub_gest_navegacion;
ros::Publisher pub_errores;

// Estructura con el valor de los sensores
Sensors sensorsValues;
// Cola de waypoints
queue <vector<float> > queueWPs;

// Variable de salida del modulo
bool exitModule;
bool exitNavigation;

// Variables de los sensores


int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "navegacion");

  // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_navegacion",estado_actual);
  }
  cout << "Atica Navegacion :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_modo = n.advertise<Modulo_Navegacion::msg_modo>("mode", 1000);
  pub_gest_navegacion = n.advertise<Modulo_Navegacion::msg_gest_navegacion>("pre_navigation",1000);
  pub_errores = n.advertise<Modulo_Navegacion::msg_errores>("error",1000);

  // Generacion de suscriptores
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_hab_modulo = n.subscribe("module_activation", 1000, fcn_sub_hab_modulos);
  ros::Subscriber sub_gps = n.subscribe("gps",1000,fcn_sub_gps);
  ros::Subscriber sub_waypoint = n.subscribe("waypoint",1000,fcn_sub_waypoint);
  
  // Variables globales
  exitModule=false;
  exitNavigation=false;
  
  // Variables globales de los sensores
  sensorsValues.lat=0;
  sensorsValues.lon=0;
  sensorsValues.alt=0;
  sensorsValues.pitch=0;
  sensorsValues.yaw=0;
  sensorsValues.roll=0;
  for(int i=0;i<100;i++){
      sensorsValues.angles[i]=0;
      sensorsValues.distances[i]=0;
  }

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_navegacion",STATE_OK);
  cout << "Atica Laser2D :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule){
      n.getParam("estado_modulo_navegacion",estado_actual);
      if(estado_actual== STATE_ERROR || estado_actual==STATE_OFF){
           exitModule=true;
      }
  }

  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gps
void fcn_sub_hab_modulos(const Modulo_Navegacion::msg_habilitacion_modulo msg)
{
    if(msg.id_modulo==ID_MOD_NAVEGACION){
        if(!msg.activo){
            // Se sale de la navegacion
            exitNavigation=true;
            // Se borra la lista de WPs
            while(!queueWPs.empty()){
                queueWPs.pop();
            }
        }else{
            exitNavigation=false;
            Modulo_Navegacion::msg_gest_navegacion msg_gn;
            vector<float> currentWP;
            
            switch (msg.submodo){
                case SUBMODE_NAV_COME_TO_ME:
                    // Se activa el modo COME TO ME
                    // Obtencion del siguiente WP
                    currentWP=queueWPs.front();
                    queueWPs.pop();

                    //Especificar el GOAL para move_base  (objetivo)
                    // TODO

                    // Mientras no llegue al WP (ni se finalice el seguimiento), envia informacion necesaria
                    while(!hasReachedToWP(currentWP, sensorsValues.lat, sensorsValues.lon) || !exitNavigation){
                        msg_gn = adaptSensorValues(sensorsValues);
                        pub_gest_navegacion.publish(msg_gn);
                    }

                    // Se ha llegado al destino, fin de la navegacion
                    exitNavigation=true;
                    break;
                    
                case SUBMODE_NAV_FOLLOW_ME:
                    // Se activa el modo FOLLOW ME
                    while(!exitNavigation){
                        if(!checkEndListWaypoints()){
                            // Obtencion del siguiente WP
                            currentWP=queueWPs.front();
                            queueWPs.pop();
                            //Especificar el GOAL para move_base  (objetivo)
                            // TODO

                            // Mientras no llegue al WP (ni se finalice el seguimiento), envia informacion necesaria
                            while(!hasReachedToWP(currentWP, sensorsValues.lat, sensorsValues.lon) && !exitNavigation){
                                msg_gn = adaptSensorValues(sensorsValues);
                                pub_gest_navegacion.publish(msg_gn);
                            }
                        }
                    }
                    // Se ha llegado al destino, fin de la navegacion
                    exitNavigation=true;
                    break;
                case SUBMODE_NAV_PLAN:
                    // Se activa el modo PLAN
                    while(!checkEndListWaypoints() || !exitModule){
                        // Obtencion del siguiente WP
                        currentWP=queueWPs.front();
                        queueWPs.pop();
                        //Especificar el GOAL para move_base  (objetivo)
                        // TODO
                        // Mientras no llegue al WP (ni se finalice el seguimiento), envia informacion necesaria
                        while(!hasReachedToWP(currentWP, sensorsValues.lat, sensorsValues.lon) && !exitNavigation){
                            msg_gn = adaptSensorValues(sensorsValues);
                            pub_gest_navegacion.publish(msg_gn);
                        }
                    }
                    // Se ha llegado al destino, fin de la navegacion
                    exitNavigation=true;
                    break;
                default:
                    break;
            }
            // Siempre que acaba la navegacion se pasa a modo neutro
            Modulo_Navegacion::msg_modo msg_md;
            msg_md.modo=MODE_NEUTRAL;
            pub_modo.publish(msg_md);
        }
    }
}

// Suscriptor de errores
void fcn_sub_laser(const Modulo_Navegacion::msg_laser msg)
{
    sensorsValues.angles=msg.angulos;
    sensorsValues.distances=msg.distancias;
}

// Suscriptor de teleoperado
void fcn_sub_gps(const Modulo_Navegacion::msg_gps msg)
{
    sensorsValues.alt=msg.altitud;
    sensorsValues.lon=msg.longitud;
    sensorsValues.alt=msg.altitud;
    sensorsValues.pitch=msg.pitch;
    sensorsValues.yaw=msg.yaw;
    sensorsValues.roll=msg.roll;
}

// Suscriptor de waypoints
void fcn_sub_waypoint(const Modulo_Navegacion::msg_waypoint msg)
{
    vector<float> wp;
    wp[0]=msg.waypoints[0];
    wp[1]=msg.waypoints[1];
    queueWPs.push(wp);
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias
Modulo_Navegacion::msg_gest_navegacion adaptSensorValues(Sensors sensorsValues){
    // TODO
    Modulo_Navegacion::msg_gest_navegacion msg_gn;
    return msg_gn;
}

bool hasReachedToWP(vector<float> currentWP, float currentLat, float currentLon){
    // TODO
    return true;
}
bool checkEndListWaypoints(){
    return queueWPs.empty();
}

