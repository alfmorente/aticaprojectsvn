#include "../include/Modulo_TeachMapping/teachMapping.h"

using namespace std;

// Publicadores
ros::Publisher pub_errores;

//Variables globales de gestion del modulo
bool exitModule;
bool mappingActive;
bool teachActive;

// Cola de WP para guardar en fichero cuando este activo el modo TEACH
queue <vector<float> > wps;
// Fichero con WPs para el modo TEACH que se enviara a UCR cuando finalice el modo
ofstream fileStream;

int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "teachMapping");
  
  cout << "hola mundooooooooooooooooooooooooooo" << endl;

    // Manejador ROS
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int estado_actual=STATE_OFF;
  while(estado_actual!=STATE_CONF){
          n.getParam("estado_modulo_teachMapping",estado_actual);
  }
  cout << "Atica TEACH & MAPPING :: Iniciando configuración..." << endl;

  // Generación de publicadores
  pub_errores = n.advertise<Modulo_TeachMapping::msg_errores>("error", 1000);

  // Generacion de suscriptores
  ros::Subscriber sub_laser = n.subscribe("laser", 1000, fcn_sub_laser);
  ros::Subscriber sub_gps = n.subscribe("gps", 1000, fcn_sub_gps);
  ros::Subscriber sub_habilitacion_modulo = n.subscribe("module_activation", 1000, fcn_sub_hab_modulos);

  // Variables globales
  exitModule=false;
  mappingActive=false;
  teachActive=false;

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  n.setParam("estado_modulo_teachMapping",STATE_OK);
  cout << "Atica TEACH & MAPPING :: Configurado y funcionando" << endl;

  while (ros::ok() && !exitModule)
  {
      n.getParam("estado_modulo_teachMapping",estado_actual);
      if(estado_actual==STATE_ERROR || estado_actual== STATE_OFF){
          exitModule=true;
      }
      ros::spinOnce();
  }
  cout << "Atica TEACH & MAPPING :: Módulo finalizado" << endl;
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// Suscriptor de laser
void fcn_sub_laser(const Modulo_TeachMapping::msg_laser msg)
{
  ROS_INFO("I heard a LASER message");
}

// Suscriptor de gps
void fcn_sub_gps(const Modulo_TeachMapping::msg_gps msg)
{
  // Si esta activo el modo TEACH, guarda la coordenada en la cola de WP
    if(teachActive){
        vector<float> wp;
        wp[0]=msg.latitud;
        wp[1]=msg.longitud;
        wps.push(wp);
    }
}

// Suscriptor de habilitacion de modulos
void fcn_sub_hab_modulos(const Modulo_TeachMapping::msg_habilitacion_modulo msg)
{
    switch(msg.id_modulo){
        case ID_MOD_MAPPING:
            if(msg.activo){
                mappingActive=true;
                while(mappingActive){
                    // Guardar datos en fichero
                    //TODO
                }
            }else{
                mappingActive=false;
            }
            break;
        case ID_MOD_TEACH:
            if(msg.activo){
                teachActive=true;
                // Apertura de fichero
                fileStream.open("dataTeach.txt",fstream::in);
                fileStream.clear();
                while(teachActive){
                    // Guardar WPs en fichero
                    if(!wps.empty()){
                        if(!saveDataGPS()){
                            cout << "Atica TEACH :: Problemas en la escritura de datos en el fichero" << endl;
                        }
                    }
                }
            }else{
                teachActive=false;
                // Se cierra el fichero y se envia a la UCR
                fileStream.close();
                //TODO (enviar a UCR)
            }
            break;
        default:
            break;
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

bool saveDataLaser(){return true;}

// Devuelve error si ha habido fallos en la escritura del fichero
bool saveDataGPS(){
    vector<float> wp = wps.front();
    wps.pop();

    // El formato será LATITUD LONGITUD\n
    fileStream << wp[0] << " " << wp[1] << endl;
    return !fileStream.fail();
}

