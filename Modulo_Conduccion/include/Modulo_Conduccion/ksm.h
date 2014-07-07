//ELECTRONIC TRANSMISSION CONTROLLER1
typedef struct elecTransController1 {
  char * opcion;
  double revoluciones;
  double porcentaje;
  double revoluciones2;  
}etc1;


//ELECTRONIC ENGINE CONTROLLER1
typedef struct elecEngineController1 {
  double revoluciones;
  int porcentaje;
}eec1;


//ELECTRONIC TRANSMISSION CONTROLLER2
typedef struct elecTransController2 {
  int marcha;
  double marcha2;
  int marcha3;
}etc2;


//ELECTRONIC BRAKE CONTROLLER1
typedef struct elecBrakeController1 {
  char * opcion;
  double porcentaje;
}ebc1;


//CRUISE CONTROL VEHICULE SPEED
typedef struct cruiseControlVehiSpeed {
  char * opcion;
  double km_h;
  char * opcion2;
  char * opcion3;
  char * opcion4;
  char * pto_state;  
}ccvs;


//AUXILIARY STATE
typedef struct auxiliaryState {
  char * opcion;
  char * opcion2;
  char * opcion3;
}aux;


//ELECTRONIC ENGINE CONTROLLER2
typedef struct elecEngineController2 {
  char * opcion;
  char * opcion2;
  double porcentaje;
  double porcentaje2;
}eec2;


//ENGINE TEMPERATURE
typedef struct engineTemperature {
  int grados;
  int grados2;
  double grados3;
}et;


//ENGINE FLUID
typedef struct engineFluid {
  double bares;
}ef;


//SUPPLY PRESSURE
typedef struct supplyPressure {
  double bares;
  double bares2;
  double bares3;
  double bares4;
  double bares5;
  double bares6;
}sp;


//AMBIENT CONDITIONS
typedef struct ambientConditions {
  double bares;
  double grados;
}ac;


//VEHICULE DISTANCE HIGH RESOLUTION
typedef struct vehiculeDistance {
  double kilometros;
}vdhr;


//VEHICULE WEIGHT
typedef struct vehiculeWeight {
  char * axle;
  double kilogramos;
}vheacs;


//ENGINE HOURS
typedef struct engineHours {
  double horas;
}eh;


//TACOGRAPH
typedef struct tacograph {
  char * driverWorking1;
  char * driverWorking2;
  char * opcion;
  char * driverTime;
  char * opcion2;
  char * opcion3;
  char * driverTime2;
  char * opcion4;
  char * opcion5;
  char * opcion6;
  char * opcion7;
  double km_h;
}t;


//ELECTRONIC RETARDER CONTROLLER EXHAUST
typedef struct elecRetarder {
  int porcentaje;
}erc;


//AUX STAT
typedef struct auxStat {
  char * opcion;
  char * opcion2;
}auxstat;


//FUEL ECONOMY
typedef struct fuelEconomy {
  double litros;
  double km_l;  
}fe;


//ELECTRONIC TRANSMISSION CONTROLLER3
typedef struct elecTransController3 {
  char * opcion;
  char * opcion2;
  char * pto1state;
  char * pto2state;
  char * nmvstate;  
}etc3;




typedef struct ksmData{
  char signal1;
  etc1 elecTransController1;  
  
  char signal2;
  eec1 elecEngineController1;  
  
  char signal3;
  etc2 elecTransController2;  
  
  char signal4;
  ebc1 elecBrakeController1;  
  
  char signal5;
  ccvs cruiseControlVehiSpeed;
  
  char signal6;
  aux auxiliaryState;
  
  char signal7;
  eec2 elecEngineController2;
  
  char signal8;
  et engineTemperature;
  
  char signal9;
  ef engineFluid;
  
  char signal10;
  sp supplyPressure;
  
  char signal11;
  ac ambientConditions;
  
  char signal12;
  vdhr vehiculeDistance;
  
  char signal13;
  vheacs vehiculeWeight;
  
  char signal14;
  eh engineHours;
  
  char signal15;
  t tacograph;
  
  char signal16;
  erc elecRetarder;
  
  char signal17;
  auxstat auxStat;
  
  char signal18;
  fe fuelEconomy;

  char signal19;
  etc3 elecTransController3;

}ksm;
  
  



