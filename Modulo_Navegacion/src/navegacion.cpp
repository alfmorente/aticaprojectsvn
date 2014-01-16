#include "../include/Modulo_Navegacion/navegacion.h"

using namespace std;

// Publicadores globales
ros::Publisher pub_mode;
ros::Publisher pub_manage_navigation;
ros::Publisher pub_error;

// Estructura con el valor de los sensores
Sensors sensorsValues;
mode_nav modeNavValue;

// Cola de waypoints
queue <float> queueWPsLatitude;
queue <float> queueWPsLongitude;

//queueWPsLatitude.push(2);

// Variable de salida del modulo
int statusModule;
bool exitModule;

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
  pub_mode = n.advertise<Modulo_Navegacion::msg_mode>("mode", 1000);
  pub_manage_navigation = n.advertise<Modulo_Navegacion::msg_gest_navegacion>("pre_navigation",1000);
  pub_error = n.advertise<Modulo_Navegacion::msg_error>("error",1000);

  // Generacion de suscriptores
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_module_enable = n.subscribe("module_enable", 1000, fcn_sub_module_enable);
  ros::Subscriber sub_gps = n.subscribe("gps",1000,fcn_sub_gps);
  ros::Subscriber sub_waypoints = n.subscribe("waypoints",1000,fcn_sub_waypoints);
  
  // Variables globales

  
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
      else if(modeNavValue.status==NAVIGATION_ON)
      {
          switch(modeNavValue.submodule)
          {
              case SUBMODE_NAV_COME_TO_ME:
                  fcn_mng_CTME();
                  break;
              case SUBMODE_NAV_PLAN:
                  fcn_mng_PLAN();
                  break;
              case SUBMODE_NAV_FOLLOW_ME:
                  fcn_mng_FLME();
                  break;
              default:
                  break;
          }

      }
  }

  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de gestion de sistema
void fcn_sub_module_enable(const Modulo_Navegacion::msg_module_enable msg)
{
    if(msg.id_modulo==ID_MOD_NAVEGACION)
    {
        modeNavValue.submodule=msg.submodo;
        modeNavValue.status=msg.activo;
    }
}

// Suscriptor de errores
void fcn_sub_laser(const Modulo_Navegacion::msg_laser msg)
{
    sensorsValues.angles=msg.angulos;
    sensorsValues.distances=msg.distancias;
}

// Suscriptor de gps
void fcn_sub_gps(const Modulo_Navegacion::msg_gps msg)
{
    sensorsValues.alt=msg.height;
    sensorsValues.lon=msg.longitude;
    sensorsValues.alt=msg.latitude;
    sensorsValues.pitch=msg.pitch;
    sensorsValues.yaw=msg.yaw;
    sensorsValues.roll=msg.roll;
}

// Suscriptor de waypoints
void fcn_sub_waypoints(const Modulo_Navegacion::msg_waypoints msg)
{
    for(int i=0;i<msg.num_waypoints;i++)
    {
        queueWPsLatitude.push(msg.waypoint_lat[i]);
        queueWPsLongitude.push(msg.waypoint_lon[i]);
    }
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
    msg_gn.posicion
    return msg_gn;
}

bool hasReachedToWP(float currentWPLongitude,float currentWPLatitude, float currentLat, float currentLon){
    // TODO

    return true;
}
bool checkEndListWaypoints()
{
    if(queueWPsLatitude.empty() || queueWPsLongitude.empty())
        return true;
    else
        return false;
}

void fcn_mng_CTME()
{
    // Se activa el modo COME TO ME
    bool endCTME;
    float currentWPLatitude;
    float currentWPLongitude;
    Modulo_Navegacion::msg_gest_navegacion msg_gn;


    //Espero waypoint
    while(checkEndListWaypoints() && statusModule!=NAVIGATION_OFF)
        ros::spinOnce();

    if(statusModule!=NAVIGATION_OFF)
    {
        currentWPLatitude=queueWPsLatitude.front();
        queueWPsLatitude.pop();

        currentWPLongitude=queueWPsLongitude.front();
        queueWPsLongitude.pop();

        msg_gn = adaptSensorValues(sensorsValues);
        pub_manage_navigation.publish(msg_gn);
        while(!endCTME)
        {
            switch(modeNavValue.status)
            {
                case NAVIGATION_OFF:
                    //Mandar stop a move base
                    endCTME=true;
                    break;
                case NAVIGATION_PAUSE:
                    //Se manda pause a move base
                    break;
                case NAVIGATION_ON:
                    // Obtencion del siguiente WP
                    if(hasReachedToWP(currentWPLatitude,currentWPLongitude, sensorsValues.lat, sensorsValues.lon))
                    {
                          Modulo_Navegacion::msg_mode mode;
                          mode.mode=MODE_PLAN;
                          mode.status=MODE_FINISH;
                          pub_mode.publish(mode);

                          modeNavValue.submodule=MODE_COME_TO_ME;
                          modeNavValue.status=NAVIGATION_OFF;

                    }
                    break;
                default:
                    break;

            }
        }
    }

    else
    {
        // Se borra la lista de WPs
        while(!queueWPsLongitude.empty())
            queueWPsLongitude.pop();
        while(!queueWPsLatitude.empty())
            queueWPsLatitude.pop();

        // Siempre que acaba la navegacion se publica modoe finalizado
        Modulo_Navegacion::msg_mode mode;
        mode.mode=MODE_COME_TO_ME;
        mode.status=MODE_FINISH;
        pub_mode.publish(mode);
    }

}


void fcn_mng_PLAN()
{
    // Se activa el modo PLAN
    bool endPLAN;
    float currentWPLatitude;
    float currentWPLongitude;
    Modulo_Navegacion::msg_gest_navegacion msg_gn;

    //Espero waypoint
    while(checkEndListWaypoints() && statusModule!=NAVIGATION_OFF)
        ros::spinOnce();

    if(statusModule!=NAVIGATION_OFF)
    {
        currentWPLatitude=queueWPsLatitude.front();
        queueWPsLatitude.pop();

        currentWPLongitude=queueWPsLongitude.front();
        queueWPsLongitude.pop();

        msg_gn = adaptSensorValues(sensorsValues);
        pub_manage_navigation.publish(msg_gn);
        while(!endPLAN)
        {
            switch(modeNavValue.status)
            {
                case NAVIGATION_OFF:
                    //Mandar stop a move base
                    // Se borra la lista de WPs
                    while(!queueWPsLongitude.empty())
                        queueWPsLongitude.pop();
                    while(!queueWPsLatitude.empty())
                        queueWPsLatitude.pop();
                    endPLAN=true;
                    break;
                case NAVIGATION_PAUSE:
                    //Se manda pause a move base
                    break;
                case NAVIGATION_ON:
                    // Obtencion del siguiente WP
                    if(hasReachedToWP(currentWPLatitude,currentWPLongitude, sensorsValues.lat, sensorsValues.lon))
                    {
                        if(checkEndListWaypoints())
                        {
                                Modulo_Navegacion::msg_mode mode;
                                mode.mode=MODE_PLAN;
                                mode.status=MODE_FINISH;
                                pub_mode.publish(mode);

                                modeNavValue.status=NAVIGATION_OFF;
                        }
                        else
                        {
                                currentWPLatitude=queueWPsLatitude.front();
                                queueWPsLatitude.pop();

                                currentWPLongitude=queueWPsLongitude.front();
                                queueWPsLongitude.pop();

                                msg_gn = adaptSensorValues(sensorsValues);
                                pub_manage_navigation.publish(msg_gn);
                        }

                    }
                    break;
                default:
                    break;
            }
        }
    }
    else
    {
        // Se borra la lista de WPs
        while(!queueWPsLongitude.empty())
            queueWPsLongitude.pop();
        while(!queueWPsLatitude.empty())
            queueWPsLatitude.pop();

        // Siempre que acaba la navegacion se publica modoe finalizado
        Modulo_Navegacion::msg_mode mode;
        mode.mode=MODE_PLAN;
        mode.status=MODE_FINISH;
        pub_mode.publish(mode);
    }

}

void fcn_mng_FLME()
{
    // Se activa el modo PLAN
    bool endFLME;
    float currentWPLatitude;
    float currentWPLongitude;
    Modulo_Navegacion::msg_gest_navegacion msg_gn;
    while(!endFLME)
     {
        switch(modeNavValue.status)
        {
            case NAVIGATION_OFF:
                //Mandar stop a move base
                endFLME=true;
                break;
            case NAVIGATION_PAUSE:
                //Se manda pause a move base
                break;
            case NAVIGATION_ON:
                // Obtencion del siguiente WP
                if(!checkEndListWaypoints())
                {
                    currentWPLatitude=queueWPsLatitude.front();
                    queueWPsLatitude.pop();

                    currentWPLongitude=queueWPsLongitude.front();
                    queueWPsLongitude.pop();

                    msg_gn = adaptSensorValues(sensorsValues);
                    pub_manage_navigation.publish(msg_gn);
                }
                break;
            default:
                break;

        }
    }

}