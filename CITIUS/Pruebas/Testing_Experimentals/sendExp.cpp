#include "sendExp.h"

/*******************************************************************************
 EXP 1. SET USV REMOTE CONTROL
 ******************************************************************************/

void send_msg_exp1(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVRemote1Message msgExp = setUSVRemote1MessageCreate();
    msgExp->rpm_order = 2000;
    msgExp->rudder_angle = -30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setUSVRemote1MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setUSVRemote1MessageDestroy(msgExp);
}

void send_msg_exp1_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVRemote1Message msgExp = setUSVRemote1MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->rpm_order = 2000;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setUSVRemote1MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    //msgExp = setUSVRemote1MessageCreate();
    msgExp->presenceVector = 0x02;
    msgExp->rudder_angle = -30;
    // Envio el mensaje JAUS
    jMsg = setUSVRemote1MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setUSVRemote1MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 2. REPORT USV REMOTE CONTROL
 ******************************************************************************/

void send_msg_exp2(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportUSVRemoteControl2Message msgExp = reportUSVRemoteControl2MessageCreate();
    msgExp->applied_direction = 2.14;
    msgExp->requested_rpm = -3000;
    msgExp->requested_rudder_angle = -65;
    msgExp->applied_rpm_m1 = -3200;
    msgExp->applied_rpm_m2 = -1900;
    msgExp->applied_rudder_angle = -50;
    msgExp->velocity_limitations = 50;
    msgExp->direction_limitations = 20;
    msgExp->mode_switching_status = 6;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportUSVRemoteControl2MessageDestroy(msgExp);
}

void send_msg_exp2_pv(OjCmpt comp, JausAddress jAdd){
     //Mensaje JAUS a enviar
    ReportUSVRemoteControl2Message msgExp = reportUSVRemoteControl2MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x0001;
    msgExp->applied_direction = 2.14;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x0002;
    msgExp->requested_rpm = -3000;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);

    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->requested_rudder_angle = -65;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);

    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->applied_rpm_m1 = -3200;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->applied_rpm_m2 = -1900;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    // Sexto parametro
    msgExp->presenceVector = 0x0020;
    msgExp->applied_rudder_angle = -50;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    // Septimo parametro
    msgExp->presenceVector = 0x0040;
    msgExp->velocity_limitations = 50;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    // Octavo parametro
    msgExp->presenceVector = 0x0080;
    msgExp->direction_limitations = 20;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    // Octavo parametro
    msgExp->presenceVector = 0x0100;
    msgExp->mode_switching_status = 6;
    // Envio el mensaje JAUS
    jMsg = reportUSVRemoteControl2MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    
    
}

/*******************************************************************************
 EXP 3. USV INFO
 ******************************************************************************/ 
void send_msg_exp3(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    USVInfo3Message msgExp = usvInfo3MessageCreate();
    msgExp->active_rudder_angle = -15;
    msgExp->active_rpm_m1 = -3000;
    msgExp->active_rpm_m2 = -3500;
    msgExp->fuel_level = 52;
    msgExp->pressure_m1 = 70;
    msgExp->pressure_m2 = 60;
    msgExp->temperature_m1 = 250;
    msgExp->temperature_m2 = 20;
    msgExp->voltage_m1 = 6;
    msgExp->voltage_m2 = 15;
    msgExp->alarms = 1;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = usvInfo3MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    usvInfo3MessageDestroy(msgExp);
}

void send_msg_exp3_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    USVInfo3Message msgExp = usvInfo3MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x0001;
    msgExp->active_rudder_angle = -15;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x0002;
    msgExp->active_rpm_m1 = -3000;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->active_rpm_m2 = -3500;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->fuel_level = 52;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->pressure_m1 = 70;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Sexto parametro
    msgExp->presenceVector = 0x0020;
    msgExp->pressure_m2 = 60;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);  
    
    // Septimo parametro
    msgExp->presenceVector = 0x0040;
    msgExp->temperature_m1 = 250;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Octavo parametro
    msgExp->presenceVector = 0x0080;
    msgExp->temperature_m2 = 20;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Noveno parametro
    msgExp->presenceVector = 0x0100;
    msgExp->voltage_m1 = 6;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Decimo parametro
    msgExp->presenceVector = 0x0200;
    msgExp->voltage_m2 = 23;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Decimo primer parametro
    msgExp->presenceVector = 0x0400;
    msgExp->alarms = 1;
    // Envio el mensaje JAUS
    jMsg = usvInfo3MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    usvInfo3MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 4. ADDITIONAL GPS/INS INFO
 ******************************************************************************/ 
void send_msg_exp4(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    AditionalGPSINSInfo4Message msgExp = aditionalGPSINSInfo4MessageCreate();
    msgExp->longitudinal_acc = 25100;
    msgExp->lateral_acc = -25633;
    msgExp->vertical_acc = -12;
    msgExp->gpsins_status = -1;
    msgExp->measure_quality[0]=5.34;
    msgExp->measure_quality[1]=5.35;
    msgExp->measure_quality[2]=5.33;
    msgExp->st_lat_deviation = 500;
    msgExp->st_lon_deviation = 750;
    msgExp->st_alt_deviation = 0;
    msgExp->dgps_corrections = JAUS_TRUE;
    msgExp->gpsins_availability = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    aditionalGPSINSInfo4MessageDestroy(msgExp);
}

void send_msg_exp4_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    AditionalGPSINSInfo4Message msgExp = aditionalGPSINSInfo4MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x0001;
    msgExp->longitudinal_acc = 25100;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x0002;
    msgExp->lateral_acc = -25633;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->vertical_acc = -12;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->gpsins_status = -1;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->measure_quality[0]=5.34;
    msgExp->measure_quality[1]=5.35;
    msgExp->measure_quality[2]=5.33;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Sexto parametro
    msgExp->presenceVector = 0x0020;
    msgExp->st_lat_deviation = 500;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);  
    
    // Septimo parametro
    msgExp->presenceVector = 0x0040;
    msgExp->st_lon_deviation = 750;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Octavo parametro
    msgExp->presenceVector = 0x0080;
    msgExp->st_alt_deviation = 0;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Noveno parametro
    msgExp->presenceVector = 0x0100;
    msgExp->dgps_corrections = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Decimo parametro
    msgExp->presenceVector = 0x0200;
    msgExp->gpsins_availability = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    aditionalGPSINSInfo4MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 5. ANEMOMETER INFO
 ******************************************************************************/ 
void send_msg_exp5(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    AnemometerInfo5Message msgExp = anemometerInfo5MessageCreate();
    msgExp->anemometer_availability = JAUS_TRUE;
    msgExp->wind_velocity = 120;
    msgExp->wind_direction = -2.15;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = anemometerInfo5MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    anemometerInfo5MessageDestroy(msgExp);
}

void send_msg_exp5_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    AnemometerInfo5Message msgExp = anemometerInfo5MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->anemometer_availability = JAUS_TRUE;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = anemometerInfo5MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->wind_velocity = 120;
    // Envio el mensaje JAUS
    jMsg = anemometerInfo5MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->wind_direction = -2.15;
    // Envio el mensaje JAUS
    jMsg = anemometerInfo5MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
        
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    anemometerInfo5MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 6. SCPM INFO
 ******************************************************************************/
void send_msg_exp6(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SCPMInfo6Message msgExp = scpmInfo6MessageCreate();
    msgExp->wave_altitude = -45;
    msgExp->wave_frequency = 400;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = scpmInfo6MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    scpmInfo6MessageDestroy(msgExp);
}

void send_msg_exp6_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SCPMInfo6Message msgExp = scpmInfo6MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->wave_altitude = -45;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = scpmInfo6MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->wave_frequency = 400;
    // Envio el mensaje JAUS
    jMsg = scpmInfo6MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    scpmInfo6MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 7. SET USV OBSERVATIONS CONFIG
 ******************************************************************************/ 
void send_msg_exp7(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVObservationsConfig7Message msgExp = setUSVObservationsConfig7MessageCreate();
    msgExp->pan = -45;
    msgExp->tilt = 90;
    msgExp->day_night_function = JAUS_FALSE;
    msgExp->infrared_filter = JAUS_FALSE;
    msgExp->tracking_function = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setUSVObservationsConfig7MessageDestroy(msgExp);
}

void send_msg_exp7_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetUSVObservationsConfig7Message msgExp = setUSVObservationsConfig7MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->pan = -45;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->tilt = 90;
    // Envio el mensaje JAUS
    jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->day_night_function = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->infrared_filter = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->tracking_function = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setUSVObservationsConfig7MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setUSVObservationsConfig7MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 8. REPORT USV OBSERVATIONS CONFIG
 ******************************************************************************/ 

void send_msg_exp8(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportUSVObservationsConfig8Message msgExp = reportUSVObservationsConfig8MessageCreate();
    msgExp->active_pan = -45;
    msgExp->active_tilt = 90;
    msgExp->active_day_night_function = JAUS_FALSE;
    msgExp->active_infrared_filter = JAUS_FALSE;
    msgExp->active_tracking_function = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportUSVObservationsConfig8MessageDestroy(msgExp);
}

void send_msg_exp8_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportUSVObservationsConfig8Message msgExp = reportUSVObservationsConfig8MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->active_pan = -45;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->active_tilt = 90;
    // Envio el mensaje JAUS
    jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->active_day_night_function = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->active_infrared_filter = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->active_tracking_function = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportUSVObservationsConfig8MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportUSVObservationsConfig8MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 10. TELEMETER INFO
 ******************************************************************************/ 
void send_msg_exp10(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    TelemeterInfo10Message msgExp = telemeterInfo10MessageCreate();
    msgExp->shoot = JAUS_TRUE;
    for(int ind=0;ind<5;ind++) msgExp->echoes[ind]=ind*20;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = telemeterInfo10MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    telemeterInfo10MessageDestroy(msgExp);
}

void send_msg_exp10_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    TelemeterInfo10Message msgExp = telemeterInfo10MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->shoot = JAUS_TRUE;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = telemeterInfo10MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    for(int ind=0;ind<5;ind++) msgExp->echoes[ind]=ind*20;
    // Envio el mensaje JAUS
    jMsg = telemeterInfo10MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    telemeterInfo10MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 11. SET SCIENTIFICS OPERATIONS
 ******************************************************************************/ 
void send_msg_exp11(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetScientificsOperations11Message msgExp = setScientificsOperations11MessageCreate();
    msgExp->measure_panel= 13;
    msgExp->nof_measure_order = 49;
    msgExp->measure_step = 199;
    msgExp->sonar_config_slant_range = 5000;
    msgExp->sonar_config_bearing = 175;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setScientificsOperations11MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setScientificsOperations11MessageDestroy(msgExp);
}

void send_msg_exp11_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetScientificsOperations11Message msgExp = setScientificsOperations11MessageCreate();
    
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->measure_panel= 13;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setScientificsOperations11MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->nof_measure_order = 49;
    // Envio el mensaje JAUS
    jMsg = setScientificsOperations11MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
   msgExp->measure_step = 199;
    // Envio el mensaje JAUS
    jMsg = setScientificsOperations11MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->sonar_config_slant_range = 5000;
    // Envio el mensaje JAUS
    jMsg = setScientificsOperations11MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->sonar_config_bearing = 175;
    // Envio el mensaje JAUS
    jMsg = setScientificsOperations11MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setScientificsOperations11MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 12. UGV INFO
 ******************************************************************************/ 
void send_msg_exp12(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    UGVInfo12Message msgExp = ugvInfo12MessageCreate();
    msgExp->battery_level= 13;
    msgExp->battery_voltage = 23;
    msgExp->battery_current = 199;
    msgExp->battery_temperature = 350;
    msgExp->motor_temperature = 175;
    msgExp->motor_rpm = -3000;
    msgExp->alarms = 175;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = ugvInfo12MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    ugvInfo12MessageDestroy(msgExp);
}

void send_msg_exp12_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    UGVInfo12Message msgExp = ugvInfo12MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->battery_level= 13;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->battery_voltage = 23;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->battery_current = 199;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->battery_temperature = 350;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->motor_temperature = 175;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Sexto parametro
    msgExp->presenceVector = 0x20;
    msgExp->motor_rpm = -3000;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);  
    
    // Septimo parametro
    msgExp->presenceVector = 0x40;
    msgExp->alarms = 175;
    // Envio el mensaje JAUS
    jMsg = ugvInfo12MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    ugvInfo12MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 13. REPORT SCIENTIFICS OPERATIONS
 ******************************************************************************/ 
void send_msg_exp13(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportScientificOperations13Message msgExp = reportScientificOperations13MessageCreate();
    msgExp->sound_speed = 2500;
    msgExp->flow_value = 240;
    msgExp->flow_depth = 5000;
    msgExp->flow_direction = 250.23569;
    msgExp->water_sound_speed = 5236.3256;
    msgExp->water_sound_depth = 2563.236;
    msgExp->contact_id = 12;
    msgExp->contact_delay = -30.52;
    msgExp->contact_distance = 9325.236;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportScientificOperations13MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportScientificOperations13MessageDestroy(msgExp);
}

void send_msg_exp13_pv(OjCmpt comp, JausAddress jAdd){
//Mensaje JAUS a enviar
    ReportScientificOperations13Message msgExp = reportScientificOperations13MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x0001;
    msgExp->sound_speed = 2500;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x0002;
    msgExp->flow_value = 240;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x0004;
    msgExp->flow_depth = 5000;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x0008;
    msgExp->flow_direction = 250.23569;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x0010;
    msgExp->water_sound_speed = 5236.3256;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Sexto parametro
    msgExp->presenceVector = 0x0020;
    msgExp->water_sound_depth = 2563.236;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);  
    
    // Septimo parametro
    msgExp->presenceVector = 0x0040;
    msgExp->contact_id = 12;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Octavo parametro
    msgExp->presenceVector = 0x0080;
    msgExp->contact_delay = -30.52;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Noveno parametro
    msgExp->presenceVector = 0x0100;
    msgExp->contact_distance = 9325.236;
    // Envio el mensaje JAUS
    jMsg = reportScientificOperations13MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportScientificOperations13MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 14. SET LIST OF WAYPOINTS
 ******************************************************************************/ 
void send_msg_exp14(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetListOfWaypoints14Message msgExp = setListOfWaypoints14MessageCreate();
    msgExp->waypoints_list_id = 5;
    msgExp->nof_waypoints = 5;
    // Reserva de memoria para vectores
    msgExp -> waypoints_ids_list = (unsigned char *) malloc((msgExp->nof_waypoints) * JAUS_BYTE_SIZE_BYTES);
    msgExp -> latitudes_list = (JausDouble *) malloc((msgExp->nof_waypoints) * JAUS_DOUBLE_SIZE_BYTES);
    msgExp -> longitudes_list = (JausDouble *) malloc((msgExp->nof_waypoints) * JAUS_DOUBLE_SIZE_BYTES);
    msgExp -> velocities_list = (JausDouble *) malloc((msgExp->nof_waypoints) * JAUS_DOUBLE_SIZE_BYTES);
    
    for(int ind=0;ind<msgExp->nof_waypoints;ind++){
        msgExp->waypoints_ids_list[ind]=ind+1;
        msgExp->latitudes_list[ind]=(ind+1)*20.63;
        msgExp->longitudes_list[ind]=(ind+1)*30.63;
        msgExp->velocities_list[ind]=(ind+1)*40.63;
    }
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setListOfWaypoints14MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setListOfWaypoints14MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 15. REPORT LIST OF WAYPOINTS
 ******************************************************************************/ 
void send_msg_exp15(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportListOfWaypoints15Message msgExp = reportListOfWaypoints15MessageCreate();
    msgExp->objetive_list_id = 2;
    msgExp->objetive_waypoint_id = 50;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportListOfWaypoints15MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportListOfWaypoints15MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 16. HEARTBEAT - CHANNEL STATE MESSAGE OPERATIONS
 ******************************************************************************/ 
void send_msg_exp16(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    HeartbeatChannelState16Message msgExp = heartbeatChannelState16MessageCreate();
    msgExp->node_id = 2;
    msgExp->primary_channel_status = 3;
    msgExp->backup_channel_status = 2;
    msgExp->primary_channel_snr = 65;
    msgExp->backup_channel_snr = 62;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    heartbeatChannelState16MessageDestroy(msgExp);
}

void send_msg_exp16_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    HeartbeatChannelState16Message msgExp = heartbeatChannelState16MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->node_id = 2;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->primary_channel_status = 3;
    // Envio el mensaje JAUS
    jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->backup_channel_status = 2;
    // Envio el mensaje JAUS
    jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->primary_channel_snr = 65;
    // Envio el mensaje JAUS
    jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->backup_channel_snr = 62;
    // Envio el mensaje JAUS
    jMsg = heartbeatChannelState16MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    heartbeatChannelState16MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 17. HEARTBEAT - POSITION INFO
 ******************************************************************************/ 
void send_msg_exp17(OjCmpt comp, JausAddress jAdd){
        //Mensaje JAUS a enviar
    HeartbeatPositionInfo17Message msgExp = heartbeatPositionInfo17MessageCreate();
    msgExp->latitude = -25;
    msgExp->longitude = 30;
    msgExp->altitude = -2050;
    msgExp->heading = -2.3;
    msgExp->speed = 620;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    heartbeatPositionInfo17MessageDestroy(msgExp);
}

void send_msg_exp17_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    HeartbeatPositionInfo17Message msgExp = heartbeatPositionInfo17MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->latitude = -25;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->longitude = 30;
    // Envio el mensaje JAUS
    jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->altitude = -2050;
    // Envio el mensaje JAUS
    jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->heading = -2.3;
    // Envio el mensaje JAUS
    jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->speed = 620;
    // Envio el mensaje JAUS
    jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->speed = 620;
    // Envio el mensaje JAUS
    jMsg = heartbeatPositionInfo17MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    heartbeatPositionInfo17MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 18. SET SIGNALING ELEMENTS
 ******************************************************************************/ 
void send_msg_exp18(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetSignalingElements18Message msgExp = setSignalingElements18MessageCreate();
    msgExp->dipss = JAUS_FALSE;
    msgExp->dipsp = JAUS_TRUE;
    msgExp->dipsr = JAUS_FALSE;
    msgExp->blinker_left = JAUS_TRUE;
    msgExp->blinker_right = JAUS_FALSE;
    msgExp->klaxon = JAUS_TRUE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setSignalingElements18MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setSignalingElements18MessageDestroy(msgExp);
}

void send_msg_exp18_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetSignalingElements18Message msgExp = setSignalingElements18MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->dipss = JAUS_FALSE;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->dipsp = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->dipsr = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->blinker_left = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->blinker_right = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Quinto parametro
    msgExp->presenceVector = 0x20;
    msgExp->klaxon = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = setSignalingElements18MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);    
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setSignalingElements18MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 19. SET POSITIONER
 ******************************************************************************/ 
void send_msg_exp19(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetPositioner19Message msgExp = setPositioner19MessageCreate();
    msgExp->pan = 6200;
    msgExp->tilt = -1200;
    msgExp->spin_velocity = 20;
    msgExp->elevation_velocity = -30;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setPositioner19MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setPositioner19MessageDestroy(msgExp);
}

void send_msg_exp19_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetPositioner19Message msgExp = setPositioner19MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->pan = 6200;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setPositioner19MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->tilt = -1200;
    // Envio el mensaje JAUS
    jMsg = setPositioner19MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->spin_velocity = 20;
    // Envio el mensaje JAUS
    jMsg = setPositioner19MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->elevation_velocity = -30;
    // Envio el mensaje JAUS
    jMsg = setPositioner19MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setPositioner19MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 20. REPORT POSITIONER
 ******************************************************************************/ 
void send_msg_exp20(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportPositioner20Message msgExp = reportPositioner20MessageCreate();
    msgExp->active_pan = 6200;
    msgExp->active_tilt = -1200;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportPositioner20MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportPositioner20MessageDestroy(msgExp);
}

void send_msg_exp20_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportPositioner20Message msgExp = reportPositioner20MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->active_pan = 6200;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportPositioner20MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->active_tilt = -1200;
    // Envio el mensaje JAUS
    jMsg = reportPositioner20MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportPositioner20MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 21. SET DAY-TIME CAMERA
 ******************************************************************************/ 
void send_msg_exp21(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetDayTimeCamera21Message msgExp = setDayTimeCamera21MessageCreate();
    msgExp->direct_zoom = 30;
    msgExp->continuous_zoom = 1;
    msgExp->focus = 50;
    msgExp->autofocus = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setDayTimeCamera21MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setDayTimeCamera21MessageDestroy(msgExp);
}

void send_msg_exp21_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetDayTimeCamera21Message msgExp = setDayTimeCamera21MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->direct_zoom = 30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setDayTimeCamera21MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->continuous_zoom = 1;
    // Envio el mensaje JAUS
    jMsg = setDayTimeCamera21MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->focus = 50;
    // Envio el mensaje JAUS
    jMsg = setDayTimeCamera21MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->autofocus = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = setDayTimeCamera21MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setDayTimeCamera21MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 22. REPORT DAY-TIME CAMERA
 ******************************************************************************/ 
void send_msg_exp22(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportDayTimeCamera22Message msgExp = reportDayTimeCamera22MessageCreate();
    msgExp->active_zoom = 30;
    msgExp->active_focus = 10;
    msgExp->active_autofocus = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportDayTimeCamera22MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportDayTimeCamera22MessageDestroy(msgExp);
}

void send_msg_exp22_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportDayTimeCamera22Message msgExp = reportDayTimeCamera22MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->active_zoom = 30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportDayTimeCamera22MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->active_focus = 10;
    // Envio el mensaje JAUS
    jMsg = reportDayTimeCamera22MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->active_autofocus = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportDayTimeCamera22MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);

    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportDayTimeCamera22MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 23. SET NIGHT-TIME CAMERA
 ******************************************************************************/
void send_msg_exp23(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetNightTimeCamera23Message msgExp = setNightTimeCamera23MessageCreate();
    msgExp->zoom = 30;
    msgExp->polarity = JAUS_TRUE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setNightTimeCamera23MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setNightTimeCamera23MessageDestroy(msgExp);
}

void send_msg_exp23_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    SetNightTimeCamera23Message msgExp = setNightTimeCamera23MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->zoom = 30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = setNightTimeCamera23MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->polarity = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = setNightTimeCamera23MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    setNightTimeCamera23MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 24. REPORT NIGHT-TIME CAMERA
 ******************************************************************************/ 
void send_msg_exp24(OjCmpt comp, JausAddress jAdd){
//Mensaje JAUS a enviar
    ReportNightTimeCamera24Message msgExp = reportNightTimeCamera24MessageCreate();
    msgExp->active_zoom = 30;
    msgExp->active_polarity = JAUS_TRUE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportNightTimeCamera24MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportNightTimeCamera24MessageDestroy(msgExp);
}

void send_msg_exp24_pv(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportNightTimeCamera24Message msgExp = reportNightTimeCamera24MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->active_zoom = 30;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportNightTimeCamera24MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->active_polarity = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = reportNightTimeCamera24MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportNightTimeCamera24MessageDestroy(msgExp);
}

/*******************************************************************************
 EXP 25. REPORT SIGNALING ELEMENTS
 ******************************************************************************/ 
void send_msg_exp25(OjCmpt comp, JausAddress jAdd){
    //Mensaje JAUS a enviar
    ReportSignalingElements25Message msgExp = reportSignalingElements25MessageCreate();
    msgExp->dipss = JAUS_TRUE;
    msgExp->dipsp = JAUS_FALSE;
    msgExp->dipsr = JAUS_TRUE;
    msgExp->blinker_left = JAUS_FALSE;
    msgExp->blinker_right = JAUS_TRUE;
    msgExp->klaxon = JAUS_FALSE;
    
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportSignalingElements25MessageToJausMessage(msgExp);
    ojCmptSendMessage(comp, jMsg);
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportSignalingElements25MessageDestroy(msgExp);
}

void send_msg_exp25_pv(OjCmpt comp, JausAddress jAdd){
        //Mensaje JAUS a enviar
    ReportSignalingElements25Message msgExp = reportSignalingElements25MessageCreate();
        
    // Primer parametro
    msgExp->presenceVector = 0x01;
    msgExp->dipss = JAUS_TRUE;
    //Copio la dirección al mensaje
    jausAddressCopy(msgExp->destination, jAdd);
    // Envio el mensaje JAUS
    JausMessage jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Segundo parametro
    msgExp->presenceVector = 0x02;
    msgExp->dipsp = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Tercer parametro
    msgExp->presenceVector = 0x04;
    msgExp->dipsr = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Cuarto parametro
    msgExp->presenceVector = 0x08;
    msgExp->blinker_left = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Quinto parametro
    msgExp->presenceVector = 0x10;
    msgExp->blinker_right = JAUS_TRUE;
    // Envio el mensaje JAUS
    jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Sexto parametro
    msgExp->presenceVector = 0x20;
    msgExp->klaxon = JAUS_FALSE;
    // Envio el mensaje JAUS
    jMsg = reportSignalingElements25MessageToJausMessage(msgExp);    
    ojCmptSendMessage(comp, jMsg);
    
    // Liberación de memoria
    jausMessageDestroy(jMsg);
    reportSignalingElements25MessageDestroy(msgExp);
}
