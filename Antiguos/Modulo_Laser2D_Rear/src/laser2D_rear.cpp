#include <Modulo_Laser2D_Rear/laser2D_rear.h>

using namespace std;


int main(int argc, char **argv)
{

  // Inicio de ROS
  ros::init(argc, argv, "laser2D_rear");
  ros::NodeHandle n;

  // Espera activa de inicio de modulo
  int current_state=STATE_OFF;
  do
  {
      current_state=getStateModule(n);
      usleep(100000);
  }  while(current_state!=STATE_CONF);
  ROS_INFO("Atica Laser2D Rear :: Iniciando configuración...");
  
  // Generación de publicadores
  pub_data = n.advertise<sensor_msgs::LaserScan>("laser", 1000);
  pub_error = n.advertise<Common_files::msg_error>("error", 1000);

  //Gestion del ficheros 
  //Apertura
  int response=file.openFiles();
  if(response!=NO_ERROR)
  {
      if(response==LASER_LOG_FILE_ERROR)
          ROS_INFO("Laser: Error al abrir el fichero Log");
      else
          file.writeErrorInLOG(response,"Fichero LOG de datos");
      publicErrorToROS(response);
      setStateModule(n,STATE_ERROR);
      return 1;
  }
  //Lectura de la configuración
  response=file.readConfig(&configParameters,&configOutput,&configConection);
  if(response!=NO_ERROR)
  {
      file.writeErrorInLOG(response,"Fichero de Configuracion");
      publicErrorToROS(response);
      setStateModule(n,STATE_ERROR);
      return 1;
  }
  
  //Conexion con el dispositivo
  response=connect();
  if(response!=NO_ERROR)
  {
      file.writeErrorInLOG(response,"configureLaser");
      publicErrorToROS(response);
      setStateModule(n,STATE_ERROR);
      return 1;
  }  
          
  //Configuracion del dispositivo
  response=configure();
  if(response!=NO_ERROR)
  {
      publicErrorToROS(response);
      setStateModule(n,STATE_ERROR);
      miLaser.disconnect();
      return 1;
  }      

  // Todo esta correcto, lo especificamos con el correspondiente parametro
  setStateModule(n,STATE_OK);
  ROS_INFO("Atica Laser2D Rear :: Configurado y funcionando");
  file.writeDataInLOG("Atica Laser2D Rear :: Configurado y funcionando");

  int error;

  //Empiezo a recoger datos del laser
  int numErrorResponses=0;	
  int timeout=configOutput.outputInterval/configParameters.scanningFrecuency;
  if(timeout <1)
      timeout=1;
  
  ROS_INFO("Recogiendo Datos del laser 2D frontal");
  while (ros::ok() && getStateModule(n)!=STATE_OFF)
  {
        error=recvData(&scanData,timeout);
        if(error==NO_ERROR)
        {
            numErrorResponses=0;
            publicDataToROS(scanData);
        }
        else
        {	
            //Se mira si es error de comunicacion
            if(isAlive(error))
            {
                file.writeErrorInLOG(error,"getScanData");
                publicErrorToROS(error);
                if(error==INCORRECT_ANSWER)
                {
                        numErrorResponses++;
                        if(numErrorResponses>=10)		
                        {
                                file.writeErrorInLOG(FRAME_OVERFLOW,"getScanData");
                                publicErrorToROS(error);

                        }
                }
            }
            else
            {
                setStateModule(n,STATE_ERROR);
                file.writeErrorInLOG(COMM_ERROR,"getScanData");
                publicErrorToROS(COMM_ERROR);
                disconnect();
                return 1;
            }
        }
  }
  
  ROS_INFO("Atica Laser2D Rear :: Modulo finalizado");
  return 0;
}

/*******************************************************************************
 *******************************************************************************
 *                              SUSCRIPTORES
 * *****************************************************************************
 * ****************************************************************************/

// No hay

/*******************************************************************************
 *******************************************************************************
 *                              FUNCIONES PROPIAS
 * *****************************************************************************
 * ****************************************************************************/

//Funciones propias

int connect()
{
    file.writeDataInLOG("Conectando con el Laser... ");
    ROS_INFO("Laser: Conectando con el laser");
    int response=miLaser.connect(configConection.portLaser,configConection.ipLaser);
    if(response!=NO_ERROR)
    {
            ROS_INFO("Laser: Error Conectando. Compruebe el fichero LOG");
            file.writeErrorInLOG(response,"Connect");
            publicErrorToROS(response);
            return response;
    }    
    return NO_ERROR;
}

bool disconnect(){
    miLaser.disconnect();
    return true;
}

int configure(){
 
    int error;
   
    //Configuracion del laser
    //Nuevo (Una vez configurado espero a que este preparado el laser)
    ROS_INFO("Laser: Esperando a laser READY");
    do
    {
            error=miLaser.queryStatus(&q);
            if(error!= NO_ERROR)
                    return error;
            sleep(1);

    }while(q.statusLMS!=7);	
    ROS_INFO("Laser: laser READY");
    //fin nuevo

    file.writeDataInLOG("Configurando data Output... ");

    ROS_INFO("Configurando datos de salida del laser");
    //Configuro parámetros secundarios del laser
    error=miLaser.configureDataOutput(configOutput,0xF4724744);//(0x01, 0x00,eightBits,0,1,false,false,false,false,1,0xF4724744);
    if(error!= NO_ERROR)
    {	
            ROS_INFO("Laser: Error configurando.Compruebe el fichero LOG");
            file.writeErrorInLOG(error,"configureDataOutput");
            return error;
    }
    
    file.writeDataInLOG("Configurado data Output... ");
    file.writeDataInLOG("Configurando Parametros del laser... ");

    //Configuro los parametros principales del laser
    ROS_INFO("Laser: Configurando parametros de configuracion del laser");
    error=miLaser.configureLaser(configParameters,0xF4724744);
    if(error!= NO_ERROR)
    {	
            ROS_INFO("Laser: Error configurando.Compruebe el fichero LOG");
            file.writeErrorInLOG(error,"configureLaser");

            return error;
    }
    file.writeDataInLOG("Configuracion de parámetros realizada... ");
    file.writeDataInLOG("Activando laser para recibir datos... ");

    //Nuevo (Una vez configurado espero a que este preparado el laser)
    ROS_INFO("Laser: Esperando a laser READY");
    do
    {
            error=miLaser.queryStatus(&q);
            if(error!= NO_ERROR)
                    return error;
            sleep(1);

    }while(q.statusLMS!=7);	
    ROS_INFO("Laser: laser READY");


    //Configuro el laser para que empiece a enviarme datos de forma continua
    ROS_INFO("Laser: Activo escaneo continuo");
    error=miLaser.setScanDataContinuous(true);
    if(error!= NO_ERROR)
    {	

            ROS_INFO("Laser: Error activando modo continuo.Compruebe el fichero LOG");
            file.writeErrorInLOG(error,"setScanDataContinuous");
            return error;
    }
    file.writeDataInLOG("laser activado para recibir datos... ");	 
    return NO_ERROR;
}



int getStateModule(ros::NodeHandle n)
{
    int state;
    n.getParam("state_module_laser2D_rear",state);
    return state;
}

void setStateModule(ros::NodeHandle n,int state)
{
    n.setParam("state_module_laser2D_rear",state);
}

void publicDataToROS(laserScan scanData)
{
    sensor_msgs::LaserScan rosLaser;
    int factorEscala=scanData.laserChannel16[0].scalingFactor;

    rosLaser.angle_increment=(configParameters.stopAngle-configParameters.startAngle)/configParameters.angleResolution+1; 
    rosLaser.angle_min=configParameters.startAngle;
    rosLaser.angle_max=configParameters.stopAngle;

    stringstream data;
    float angle;
    int j=0; //para limitar entre un angulo minimo y uno maximo
    

    if(scanData.numberChannels16Bit>0)
    {

            data << "Datos:" <<endl;
            for(unsigned int i=0;i<scanData.laserChannel16[0].numberData;i++)
            {
                    angle=scanData.laserChannel16[0].startingAngle+scanData.laserChannel16[0].angularStepWidth*i;
                    if((angle >= rosLaser.angle_min) && (angle <=  rosLaser.angle_max))
                    {

                            //Distancia en metros
                            rosLaser.ranges.push_back(factorEscala*scanData.laserChannel16[0].data.at(i)/1000.0); 
                            data << angle << " ";					
                            data << rosLaser.ranges[j] << " ";
                            j++;
                    }
            }
            data << endl;
            file.writeDataInLOG(data.str().c_str());
            pub_data.publish(rosLaser);
    }
    else
            ROS_INFO("Configure el laser para tener acceso a algun canal de datos");
}

void publicErrorToROS(int error)
{
        
    Common_files::msg_error errorLaser2D;
    errorLaser2D.id_subsystem=SUBS_REAR_LASER;
    errorLaser2D.type_error=TOE_UNDEFINED;
    errorLaser2D.id_error=error;
    pub_error.publish(errorLaser2D);
    
}

int recvData(laserScan* scanData,int timeout)
{
    int error;
    error=miLaser.getScanData(scanData,timeout);
    return error;
}

bool isAlive(int error)
{
    if(error==COMM_ERROR)
        return false;
    else
        return true;
}
