#include <Modulo_Laser2D_Frontal/laser2D_frontal.h>
#include <Modulo_Laser2D_Frontal/interaction.h>

using namespace std;


int main(int argc, char **argv)
{

  // Obtencion del modo de operacion y comprobacion de que es correcto
  int operationMode=getOperationMode(argc, argv);
  if (operationMode == 0) {
        return 1;
  }
  ROS_INFO("Operation mode: %d",operationMode);
  // Inicio de ROS
  ros::init(argc, argv, "laser2D_frontal");
  ros::NodeHandle n;
  
  int error;
  int numErrorResponses;
  int response;
  int timeout;
  int current_state;  
  switch (operationMode) 
  {
        case OPERATION_MODE_DEBUG:
            // Espera activa de inicio de modulo
             current_state=STATE_OFF;
             do
             {
                 current_state=getStateModule(n);
                 usleep(100000);
             }  while(current_state!=STATE_CONF);
             ROS_INFO("Atica Laser2D Frontal :: Iniciando configuracion...");

             // Generación de publicadores
             pub_data = n.advertise<sensor_msgs::LaserScan>("laser", 1000);
             pub_error = n.advertise<Common_files::msg_error>("error", 1000);

             //Gestion del ficheros 
             //Apertura        
             response=file.openFiles();
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
             ROS_INFO("Atica Laser2D Frontal :: Configurado y funcionando");
             file.writeDataInLOG("Atica Laser2D Frontal :: Configurado y funcionando");



             //Empiezo a recoger datos del laser
             numErrorResponses=0;	
             timeout=configOutput.outputInterval/configParameters.scanningFrecuency;
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
             break;
        case OPERATION_MODE_RELEASE:
            // Funcionamiento del modo release
            break;
        case OPERATION_MODE_SIMULATION:
            // Funcionamiento del modo simulacion
            break;
        default:
            break;
  
  }
 
  ROS_INFO("Atica Laser2D Frontal :: Modulo finalizado");
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

    }while(q.statusLMS!=measurement);	
    ROS_INFO("Laser: laser READY");
    //fin nuevo

    file.writeDataInLOG("Configurando data Output... ");

    ROS_INFO("Configurando datos de salida del laser");
    //Configuro parámetros secundarios del laser
    error=miLaser.configureDataOutput(configOutput,PASSWORD);//(0x01, 0x00,eightBits,0,1,false,false,false,false,1,PASSWORD;
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
    error=miLaser.configureLaser(configParameters,PASSWORD);
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

    }while(q.statusLMS!=measurement);	
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
    n.getParam("state_module_front_laser_1",state);
    return state;
}

void setStateModule(ros::NodeHandle n,int state)
{
    n.setParam("state_module_front_laser_1",state);
}

void publicDataToROS(laserScan scanData)
{
    sensor_msgs::LaserScan rosLaser;
    int factorEscala=scanData.laserChannel16[0].scalingFactor;
    rosLaser.header.stamp=ros::Time::now();

    rosLaser.angle_increment=configParameters.angleResolution;
    rosLaser.angle_min=configParameters.startAngle;
    rosLaser.angle_max=configParameters.stopAngle;
    rosLaser.time_increment=0.02;
    rosLaser.scan_time=0.1;
    rosLaser.range_max=100;
    rosLaser.range_min=0;

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
	    rosLaser.angle_min=rosLaser.angle_min*M_PI/180;
	    rosLaser.angle_max=rosLaser.angle_max*M_PI/180;
	    rosLaser.angle_increment=rosLaser.angle_increment*M_PI/180;
            pub_data.publish(rosLaser);
    }
    else
            ROS_INFO("Configure el laser para tener acceso a algun canal de datos");
}

void publicErrorToROS(int error)
{
        
    Common_files::msg_error errorLaser2D;
    errorLaser2D.id_subsystem=SUBS_FRONT_LASER_1;
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
