#include <string.h>

#include "Modulo_GPS/GPS_Management.h"

using namespace std;

// Constructor de la clase
GPS_Management::GPS_Management(){
  port_opened=false;
  if(port.openSerial("/dev/ttyUSB0")){
    port.configura(B115200);
    port_opened=true;
  }else{
    cerr << "No se ha podido abrir el puerto serie" << endl;
  }
}

// Getters de los atributos
Bestgpsvel GPS_Management::getGPSVel(){return this->bestgpsvel;}
Bestgpspos GPS_Management::getGPSPos(){return this->bestgpspos;}
Inspvas GPS_Management::getInspVas(){return this->inspvas;}
Inspva GPS_Management::getInspVa(){return this->inspva;}
Bestleverarm GPS_Management::getLeverarm(){return this->bestleverarm;}
Corrimudata GPS_Management::getCorrIMUData(){return this->corrimudata;}
Inspos GPS_Management::getInsPos(){return this->inspos;}
Heading GPS_Management::getHeading(){return this->heading;}

/*gps_conf_alignmentmode:Configura la forma de alinearse de la IMU
 *
 * Recibe:
 *  type:Indica el tipo de alineamiento
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_alignmentmode(string mode){

  // Formacion del mensaje

  string options[1];
  options[0]=mode;
  string msg = create_message("ALIGNMENTMODE",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_applyvehiclebodyrotation:Activa o desactiva la rotación del gps respecto al vehículo
 *
 * Recibe:
 *  state:Indica si se habilita o no la rotación
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_applyvehiclebodyrotation(string mode){
  // Formacion del mensaje

  string options[1];
  options[0]=mode;
  string msg = create_message("APPLYVEHICLEBODYROTATION",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);

  return r.ok;
}

/* gps_conf_canconfig: Configuracion del puerto CAN
 *
 * Recibe:
 *  port: puerto a gestionar
 *  switch: activa/desactiva la configuracion del puerto CAN
 *  bit_rate: velocidad de comunicacion por el puerto
 *  base: direccion base
 *  tx_mask: mascara de los datos transmitidos
 *  source: de donde vienen los datos
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_canconfig(string port, string mode, string bitrate, int base, int tx_max, string source){
  string options[6];
  options[0]=port;
  options[1]=mode;
  options[2]=bitrate;
  options[3]=toString(base);
  options[4]=toString(tx_max);
  options[5]=source;
  string msg = create_message("CANCONFIG",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);

  return r.ok;
}

/* gps_conf_exthdgoffset:Configura el offset angular de la antena del gps
 *
 * Recibe:
 *  heading:Offset angular del azimuth
 *  headingSTD:Desviacion del heading
 *  pitch:Offset angular del pitch
 *  pitchSTD:Desviación del pitch
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_exthdgoffset(double heading,double headingSTD, double pitch, double pitchSTD){

  // Formacion del mensaje

  string options[4];
  options[0]=toString(heading);
  options[1]=toString(headingSTD);
  options[2]=toString(pitch);
  options[3]=toString(pitchSTD);
  string msg = create_message("EXTHDGOFFSET",4,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_freset: Resetea datos del GPS
 *
 * Recibe:
 *  target: dato a resetear
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_freset(string target){

  // Formacion del mensaje

  string options[1];
  options[0]=target;

  string msg = create_message("FRESET",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_inscommand:Activa o desactiva el posicionamiento INS
 *
 * Recibe:
 *  state:Indica si se habilita o no el posicionamiento INS
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_inscommand(string state){

  // Formacion del mensaje

  string options[1];
  options[0]=state;
  string msg = create_message("INSCOMMAND",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/*gps_conf_insphaseupdate:Configura el periodo de trabajo del INSGPS
 *
 * Recibe:
 *  mode: activa/deseactiva el control de fase
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_insphaseupdate(string mode){

  // Formacion del mensaje

  string options[1];
  options[0]=mode;
  string msg = create_message("INSPHASEUPDATE",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);

  return r.ok;

}

/* gps_conf_inszput:Actualiza la velocidad a 0
 *
 * Recibe:Nada
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_inszupt(){

  // Formacion del mensaje
  string msg = create_message("INZUPT",0,NULL);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_inszuptcontrol:Activa o desactiva la posibilidad de actualizar la velocidad a 0
 *
 * Recibe:
 *  state:Indica si se habilita o no esta posibilidad
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_inszuptcontrol(string state){

  // Formacion del mensaje

  string options[1];
  options[0]=state;
  string msg = create_message("INSZUPTCONTROL",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/*gps_conf_nmeatalker: Cambia el comportamiento del emisor nmea
 *
 * Recibe:
 *  id: tipo de comportamiento
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_nmeatalker(string id){

  // Formacion del mensaje

  string options[1];
  options[0]=id;
  string msg = create_message("NMEATALKER",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);

  return r.ok;

}

/* gps_conf_rvbcalibrate:Activa o desactiva la posibilidad de configurar el oofset angular de la antena
 *
 * Recibe:
 *  state:Indica si se habilita o no esta posibilidad
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_rvbcalibrate(string state){

  // Formacion del mensaje

  string options[1];
  options[0]=state;
  string msg = create_message("RVBCALIBRATE",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setimuorientation:Configura la orientacion del SPAN-CPT
 *
 * Recibe:
 *  orientation: Orientacion de los ejes del SPAN-CPT (que eje esta alineado con la gravedad)
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setimuorientation(string orientation){

  // Formacion del mensaje

  string options[1];
  options[0]=orientation;
  string msg = create_message("SETIMUORIENTATION",1,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setimutoantoffset:Configura el offset lineal de la antena
 *
 * Recibe:
 *  x:Offset en el eje X
 *  y:Offset en el eje Y
 *  z:Offset en el eje Z
 *  a:Incertidumbre del valor de x
 *  b:Incertidumbre del valor de y
 *  c:Incertidumbre del valor de z
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setimutoantoffset(double x,double y, double z, double a,double b, double c){

  // Formacion del mensaje

  string options[6];
  options[0]=toString(x);
  options[1]=toString(y);
  options[2]=toString(z);
  options[3]=toString(a);
  options[4]=toString(b);
  options[5]=toString(c);
  string msg = create_message("SETIMUTOANTOFFSET",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setimutoantoffset2:Configura el offset lineal de una segunda antena
 *
 * Recibe:
 *  x:Offset en el eje X
 *  y:Offset en el eje Y
 *  z:Offset en el eje Z
 *  a:Incertidumbre del valor de x
 *  b:Incertidumbre del valor de y
 *  c:Incertidumbre del valor de z
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setimutoantoffset2(double x,double y, double z, double a,double b, double c){

  // Formacion del mensaje

  string options[6];
  options[0]=toString(x);
  options[1]=toString(y);
  options[2]=toString(z);
  options[3]=toString(a);
  options[4]=toString(b);
  options[5]=toString(c);
  string msg = create_message("SETIMUTOANTOFFSET2",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setinitattitude:Inicializa la orientación del SPAN-CPT
 *
 * Recibe:
 *  pitch:valor del pitch inicial
 *  roll:valor del roll inicial
 *  azimuth:valor del azimuth inicial
 *  pitchSTD:desviación del pitch inicial
 *  rollSTD:desviación del roll inicial
 *  azSTD:desviación del azimuth inicial
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setinitattitude(double pitch,double roll, double azimuth, double pitchSTD,double rollSTD, double azSTD){

  // Formacion del mensaje

  string options[6];
  options[0]=toString(roll);
  options[1]=toString(pitch);
  options[2]=toString(azimuth);
  options[3]=toString(rollSTD);
  options[4]=toString(pitchSTD);
  options[5]=toString(azSTD);
  string msg = create_message("SETINITATTITUDE",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setinitazimuth:Inicializa solo el azimuth del SPAN-CPT
 *
 * Recibe:
 *  azimuth:valor del azimuth inicial
 *  azSTD:desviación del azimuth inicial
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setinitazimuth(double azimuth,double azSTD){

  // Formacion del mensaje

  string options[2];
  options[0]=toString(azimuth);
  options[1]=toString(azSTD);
  string msg = create_message("SETINITAZIMUTH",2,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setinsoffset:Especifica el offset de la posición del SPAN-CPT
 *
 * Recibe:
 *  x:valor inicial en el eje x
 *  y:valor inicial en el eje y
 *  z:valor inicial en el eje z
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setinsoffset(double x,double y,double z){

  // Formacion del mensaje

  string options[3];
  options[0]=toString(x);
  options[1]=toString(y);
  options[2]=toString(z);
  string msg = create_message("SETINSOFFSET",3,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setmark1offset: Especifica el offset al Mark1 del evento
 *
 * Recibe:
 *  x:valor offset en el eje x
 *  y:valor offset en el eje y
 *  z:valor offset en el eje z
 *  alphaoffset:offset en el roll
 *  betaOffset:offset en el pitch
 *  gammaOffset:offset en el azimuth
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setmark1offset(double x,double y,double z, double alphaOffset, double betaOffset, double gammaOffset){

  // Formacion del mensaje

  string options[6];
  options[0]=toString(x);
  options[1]=toString(y);
  options[2]=toString(z);
  options[3]=toString(alphaOffset);
  options[4]=toString(betaOffset);
  options[5]=toString(gammaOffset);
  string msg = create_message("SETMARK1OFFSET",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_setwheelparameters:Configura los parámetros de la rueda en caso de disponer de un sensor para esta
 *
 * Recibe:
 *  ticks:número de ticks por revolución
 *  circ:circunferencia de la rueda
 *  spacing:espacio entre ticks o resolución del sensor (en metros)
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_setwheelparameters(unsigned short ticks,double circ,double spacing){

  // Formacion del mensaje

  string options[3];
  options[0]=toString(ticks);
  options[1]=toString(circ);
  options[2]=toString(spacing);
  string msg = create_message("SETWHEELPARAMETERS",3,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_conf_vehiclebodyrotation: Configura el offset angular entre el vehículo y el SPAN-CPT
 *
 * Recibe:
 *  xAngle:ángulo en el eje x
 *  yAngle:ángulo en el eje y
 *  zAngle:ángulo en el eje z
 *  a:Incertidumbre del valor de xAngle
 *  b:Incertidumbre del valor de yAngle
 *  c:Incertidumbre del valor de zAngle
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_conf_vehiclebodyrotation(double xAngle,double yAngle, double zAngle, double a,double b, double c){

  // Formacion del mensaje

  string options[6];
  options[0]=toString(xAngle);
  options[1]=toString(yAngle);
  options[2]=toString(zAngle);
  options[3]=toString(a);
  options[4]=toString(b);
  options[5]=toString(c);
  string msg = create_message("VEHICLEBODYROTATION",6,options);

  // Envio del mensaje
  this->port.send((char *)msg.c_str(),msg.length());

  // Recepcion de la respuesta

  Response r = reception_management(true,false);
  return r.ok;

}

/* gps_log_general: Genera y envia un comando de tipo log
 *
 * Recibe:
 *  command: comando
 *  type: tipo de respuesta deseada (onchange, onnew, etc.)
 * Devuelve: Un booleano indicando si la petición se realizo correctamente
 *
 */
bool GPS_Management::gps_log_general(string command, string type){
  string options[2];
  options[0] = command;
  options[1] = type;
  string msg = create_message("log", 2, options);
  // Envio del mensaje

  this->port.send((char *) msg.c_str(), msg.length());

  // Recepcion de la respuesta
  Response r = reception_management(true, false);

  return r.ok;
}

// Funciones de adquisición de datos
bool GPS_Management::gps_adq_bestgpsvel(Response res){
  // Rellenar atributo bestgpsvel con los datos obtenidos de Response
  string *datos = getData(8,res.data);
  this->bestgpsvel.sol_status=datos[0];
  this->bestgpsvel.vel_type=datos[1];
  this->bestgpsvel.latency=stringToFloat(datos[2]);
  this->bestgpsvel.age=stringToFloat(datos[3]);
  this->bestgpsvel.hor_spd=stringToDouble(datos[4]);
  this->bestgpsvel.trk_gnd=stringToDouble(datos[5]);
  this->bestgpsvel.ver_spd=stringToDouble(datos[6]);
  this->bestgpsvel.reserved=stringToFloat(datos[7]);
  return true;
}

bool GPS_Management::gps_adq_bestgpspos(Response res){
  // Rellenar atributo bestgpspos con los datos obtenidos de Response
  string *datos = getData(21,res.data);
  this->bestgpspos.sol_status=datos[0];
  this->bestgpspos.state=getStateOfGPS(datos[0]);
  this->bestgpspos.pos_type=datos[1];
  this->bestgpspos.lat=stringToDouble(datos[2]);
  this->bestgpspos.lon=stringToDouble(datos[3]);
  this->bestgpspos.hgt=stringToDouble(datos[4]);
  this->bestgpspos.undulation=stringToFloat(datos[5]);
  this->bestgpspos.datum_id=datos[6];
  this->bestgpspos.lat_dev=stringToFloat(datos[7]);
  this->bestgpspos.lon_dev=stringToFloat(datos[8]);
  this->bestgpspos.hgt_dev=stringToFloat(datos[9]);
  this->bestgpspos.stn_id=datos[10];
  this->bestgpspos.diff_age=stringToFloat(datos[11]);
  this->bestgpspos.sol_age=stringToFloat(datos[12]);
  this->bestgpspos.obs=datos[13][0];
  this->bestgpspos.gpsL1=datos[14][0];
  this->bestgpspos.l1=datos[15][0];
  this->bestgpspos.l2=datos[16][0];
  this->bestgpspos.res1=datos[17][0];
  this->bestgpspos.res2=datos[18][0];
  this->bestgpspos.res3=datos[19][0];
  this->bestgpspos.res4=datos[20][0];
  return true;
}

bool GPS_Management::gps_adq_inspvas(Response res){
  // Rellenar atributo bestgpspos con los datos obtenidos de Response
  string *datos = getData(12,res.data);

  this->inspvas.week=stringToLong(datos[0]);
  this->inspvas.seconds=stringToDouble(datos[1]);
  this->inspvas.lat=stringToDouble(datos[2]);
  this->inspvas.lon=stringToDouble(datos[3]);
  this->inspvas.hgt=stringToDouble(datos[4]);
  this->inspvas.north_velocity=stringToDouble(datos[5]);
  this->inspvas.east_velocity=stringToDouble(datos[6]);
  this->inspvas.up_velocity=stringToDouble(datos[7]);
  this->inspvas.roll=stringToDouble(datos[8]);
  this->inspvas.pitch=stringToDouble(datos[9]);
  this->inspvas.azimuth=stringToDouble(datos[10]);
  this->inspvas.status=datos[11];

  return true;
}

bool GPS_Management::gps_adq_inspva(Response res){
  // Rellenar atributo bestgpspos con los datos obtenidos de Response
  string *datos = getData(12,res.data);
  this->inspva.state=getStateOfIMU(datos[11]);
  this->inspva.week=stringToLong(datos[0]);
  this->inspva.seconds=stringToDouble(datos[1]);
  this->inspva.lat=stringToDouble(datos[2]);
  this->inspva.lon=stringToDouble(datos[3]);
  this->inspva.hgt=stringToDouble(datos[4]);
  this->inspva.north_velocity=stringToDouble(datos[5]);
  this->inspva.east_velocity=stringToDouble(datos[6]);
  this->inspva.up_velocity=stringToDouble(datos[7]);
  this->inspva.roll=stringToDouble(datos[8]);
  this->inspva.pitch=stringToDouble(datos[9]);
  this->inspva.azimuth=stringToDouble(datos[10]);
  this->inspva.status=datos[11];

  return true;
}

bool GPS_Management::gps_adq_bestleverarm(Response res){
  string *datos = getData(7,res.data);
  this->bestleverarm.x_offset=stringToDouble(datos[0]);
  this->bestleverarm.y_offset=stringToDouble(datos[1]);
  this->bestleverarm.z_offset=stringToDouble(datos[2]);
  this->bestleverarm.x_uncertainty=stringToDouble(datos[3]);
  this->bestleverarm.y_uncertainty=stringToDouble(datos[4]);
  this->bestleverarm.z_uncertainty=stringToDouble(datos[5]);
  this->bestleverarm.imapping=stringToInt(datos[6]);
  return true;
}

bool GPS_Management::gps_adq_corrimudata(Response res){
  string *datos = getData(8,res.data);
  this->corrimudata.week=stringToLong(datos[0]);
  this->corrimudata.seconds=stringToDouble(datos[1]);
  this->corrimudata.roll_rate=stringToDouble(datos[2]);
  this->corrimudata.pitch_rate=stringToDouble(datos[3]);
  this->corrimudata.yaw_rate=stringToDouble(datos[4]);
  this->corrimudata.lateral_acc=stringToDouble(datos[5]);
  this->corrimudata.longitudinal_acc=stringToDouble(datos[6]);
  this->corrimudata.vertical_acc=stringToDouble(datos[7]);
  return true;
}


bool GPS_Management::gps_adq_inspos(Response res){
  string *datos = getData(6,res.data);
  this->inspos.week=stringToLong(datos[0]);
  this->inspos.seconds=stringToDouble(datos[1]);
  this->inspos.lat=stringToDouble(datos[2]);
  this->inspos.lon=stringToDouble(datos[3]);
  this->inspos.hgt=stringToDouble(datos[4]);
  this->inspos.status=datos[5];
  return true;
}

bool GPS_Management::gps_adq_heading(Response res) {
    string *datos = getData(17, res.data);
    this->heading.sol_stat=datos[0];
    this->heading.pos_type=datos[1];
    this->heading.length=stringToFloat(datos[2]);
    this->heading.heading=stringToFloat(datos[3]);
    this->heading.pitch=stringToFloat(datos[4]);
    this->heading.res=stringToFloat(datos[5]);
    this->heading.hdg_std_dev=stringToFloat(datos[6]);
    this->heading.ptch_std_dev=stringToFloat(datos[7]);
    this->heading.stn_id=datos[8];
    this->heading.observations=datos[9][0];
    this->heading.num_satellites=datos[10][0];
    this->heading.obs=datos[11][0];
    this->heading.multi=datos[12][0];
    this->heading.res2=datos[13][0];
    this->heading.ext_sol_stat=datos[14][0];
    this->heading.res3=datos[15][0];
    this->heading.sig_mask=datos[16][0];
    return true;
}

string GPS_Management::create_message(string command, int num_param, string options[]){
  string message=command;
  for(int i=0;i<num_param;i++){
    message+= " ";
    message+=options[i];
  }
  message+="\r";
  return message;
}

Response GPS_Management::reception_management(bool confirmation, bool data){
  Response resp;
  bool startConfirmation = false, endConfirmation = false;
  bool startData = false, endData =false;

  if (confirmation)
  {
    // Se lee OK o Error
    char c;
    string conf = "";
    // Se busca el comienzo de confirmacion
    while (!startConfirmation)
    {
      if (this->port.recv(&c, 1, 1) == SERIAL_OK){
        if (c == '<')
          startConfirmation = true;
      }else{
        resp.error = SERIAL_ERROR;
        resp.ok = false;
        return resp;
      }

    }
    // Se busca la confirmacion
    while (!endConfirmation){
      if (this->port.recv(&c, 1, 1) == SERIAL_OK)
      {
        conf += c;
        if (c == '\r')
          endConfirmation = true;
      }else{
        resp.error = FRAME_ERROR;
        resp.ok = false;
        return resp;
      }
    }
    // Se busca si es OK o ERROR dentro del mensaje de confirmacion
    if (conf.find("OK") != string::npos)
      resp.ok = true;
    else
      resp.ok = false;

    // No se esperan datos
    resp.data = "";
  }

  if(data){
    // Se lee los datos
    char c;
    string dataFrame = "";

    // Se busca el comienzo de los datos
    while (!startData){
      if (this->port.recv(&c, 1, 10) == SERIAL_OK){
        if (c == '$' || c=='#' || c=='%')
          startData = true;
      }else{
        resp.error = SERIAL_ERROR;
        resp.ok = false;
        return resp;
      }
    }

    // Se busca el final de los datos
    while (!endData){
      if (this->port.recv(&c, 1, 1) == SERIAL_OK){
        dataFrame += c;
        if (c == '\r')
          endData = true;
      }else{
        resp.error = FRAME_ERROR;
        resp.ok = false;
        return resp;
      }
    }

    analizeFrame(dataFrame, &resp);

    // Comprobacion de checksum
    string buf = resp.header + resp.headerdata + resp.data;
    unsigned long crcCalculated = CalculateBlockCRC32(buf.length(), (unsigned char*) buf.c_str());
    unsigned long checksumFrame;
    stringstream ss;
    ss << std::hex << resp.checksum;
    ss >> checksumFrame;
    if(crcCalculated==checksumFrame){
      resp.ok = true;
    }else{
      cerr << "Checksum ERROR" << endl;
    } 
  }

  return resp;
}

string* GPS_Management::getData(int numData, string frameData){
  int pos=0;
  string* data=new string[numData];
  char *s;
  s=strtok((char*)frameData.c_str(),",;");
  while(s!=NULL)
  {
    data[pos++]=s;
    s=strtok(NULL,",");

  }
  return data;
}

bool GPS_Management::isPortOpened(){

  return port_opened;
}

void GPS_Management::analizeFrame(string frame,Response* resp){
  unsigned int pos=0;

  while(frame[pos]!=',')
  {
    resp->header+=frame[pos++];
  }
  while(frame[pos]!=';')
  {
    resp->headerdata+=frame[pos++];
  }
  
  while(frame[pos]!='*')
  {
    resp->data+=frame[pos++];
  }
  
  pos++;
  while(frame[pos]!='\r'){
    resp->checksum+=frame[pos++];
  }

    
}

// Funcion que recibe datos independientemente cual sea
int GPS_Management::rcvData(){
  int tt=TT_ERROR;
  Response res = reception_management(false,true);
  if(res.ok){
    if(res.header=="BESTGPSVELA"){
      gps_adq_bestgpsvel(res);
      tt=TT_GPSVELA;
    }else if(res.header=="BESTGPSPOSA"){
      gps_adq_bestgpspos(res);
      tt=TT_BESTGPSPOSA;
    }else if(res.header=="INSPVASA"){
      gps_adq_inspvas(res);
      tt=TT_INSPVASA;
    }else if(res.header=="BESTLEVERARMA"){
      gps_adq_bestleverarm(res);
      tt=TT_BESTLEVERARMA;
    }else if(res.header=="CORRIMUDATASA"){
      gps_adq_corrimudata(res);
      tt=TT_CORRIMUDATASA;
    }else if(res.header=="INSPOSA"){
      gps_adq_inspos(res);
      tt=TT_INSPOSA;
    }else if(res.header=="HEADINGA"){
      gps_adq_heading(res);
      tt=TT_HEADINGA;
    }else if(res.header=="INSPVAA"){
      gps_adq_inspva(res);
      tt=TT_INSPVAA;
    }
  }else{
    tt=TT_ERROR;
  }
  
  return tt;
}

short GPS_Management::getStateOfGPS(string s){
    if(strcmp(s.c_str(),"INSUFFICIENT_OBS")==0)
        return INSUFFICIENT_OBS;
    else if(strcmp(s.c_str(),"NO_CONVERGENCE")==0)
        return NO_CONVERGENCE;
    else if(strcmp(s.c_str(),"SINGULARITY")==0)
        return SINGULARITY;
    else if(strcmp(s.c_str(),"COV_TRACE")==0)
        return COV_TRACE;
    else if(strcmp(s.c_str(),"TEST_DIST")==0)
        return TEST_DIST;
    else if(strcmp(s.c_str(),"COLD_START")==0)
        return COLD_START;
    else if(strcmp(s.c_str(),"V_H_LIMIT")==0)
        return V_H_LIMIT;
    else if(strcmp(s.c_str(),"VARIANCE")==0)
        return VARIANCE;
    else if(strcmp(s.c_str(),"RESIDUALS")==0)
        return RESIDUALS;
    else if(strcmp(s.c_str(),"DELTA_POS")==0)
        return DELTA_POS;
    else if(strcmp(s.c_str(),"NEGATIVE_VAR")==0)
        return NEGATIVE_VAR;
    else if(strcmp(s.c_str(),"INTEGRITY_WARNING")==0)
        return INTEGRITY_WARNING;
    else if(strcmp(s.c_str(),"IMU_UNPLUGGED")==0)
        return IMU_UNPLUGGED;
    else if(strcmp(s.c_str(),"PENDING")==0)
        return PENDING;
    else if(strcmp(s.c_str(),"INVALID_FIX")==0)
        return INVALID_FIX;
    else if(strcmp(s.c_str(),"UNAUTHORIZED_STATE")==0)
        return UNAUTHORIZED_STATE;
    else
        return GPS_GLOBAL_ERROR;
}

short GPS_Management::getStateOfIMU(string s){
    if(strcmp(s.c_str(),"NS_INACTIVE")==0)
        return INS_INACTIVE;
    else if(strcmp(s.c_str(),"INS_ALIGNING")==0)
        return INS_ALIGNING;
    else if(strcmp(s.c_str(),"INS_SOLUTION_NOT_GOOD")==0)
        return INS_SOLUTION_NOT_GOOD;
    else if(strcmp(s.c_str(),"INS_BAD_GPS_AGREEMENT")==0)
        return INS_BAD_GPS_AGREEMENT;
    else if(strcmp(s.c_str(),"INSUFFICIENT_OBS")==0)
        return INSUFFICIENT_OBS;
    else if(strcmp(s.c_str(),"INS_ALIGNMENT_COMPLETE")==0)
        return GPS_GLOBAL_ERROR;
    else
        return GPS_GLOBAL_ERROR;
}


void GPS_Management::setCom2ToRcvCorrections() {

    cout << "Configurando COM2 para obtencion de correcciones..." << endl;
    cout << "Enviando INTERFACE MODE...";
    string options[4];
    options[0] = "COM2";
    options[1] = "RTCMV3";
    options[2] = "NONE";
    options[3] = "OFF";
    string msg = create_message("INTERFACEMODE", 4, options);
    this->port.send((char *) msg.c_str(), msg.length());

    // Recepcion de la respuesta
    Response r = reception_management(true, false);

    if (r.ok) cout << "OK" << endl;
    else {
        cout << "ERROR" << endl;
        cout << "Fallo la configuracion RTK" << endl;
        return;
    }

    cout << "Enviando configuracion COM...";

    string optionsCom[7];
    optionsCom[0] = "COM2";
    optionsCom[1] = "57600";
    optionsCom[2] = "N";
    optionsCom[3] = "8";
    optionsCom[4] = "1";
    optionsCom[5] = "N";
    optionsCom[6] = "OFF";
    msg = create_message("COM", 7, optionsCom);
    this->port.send((char *) msg.c_str(), msg.length());

    // Recepcion de la respuesta
    r = reception_management(true, false);

    if (r.ok) cout << "OK" << endl;
    else {
        cout << "ERROR" << endl;
        cout << "Fallo la configuracion RTK" << endl;
        return;
    }
}
