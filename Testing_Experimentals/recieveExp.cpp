#include "recieveExp.h"

/*******************************************************************************
 EXP 1. SET USV REMOTE CONTROL
 ******************************************************************************/
void fcn_receive_exp1(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET USV REMOTE CONTROL (EXP #1)\n");
    SetUSVRemote1Message msgExp = setUSVRemote1MessageFromJausMessage(msg);
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_1_PV_RPM_ORDER_BIT)){
        printf("Recibido parámetro: ORDER_RPM con valor: %f\n",msgExp->rpm_order);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_1_PV_RUDDER_ANGLE_BIT)){
        printf("Recibido parámetro: RUDDER_ANGLE con valor: %f\n",msgExp->rudder_angle);
    }
    printf("*****************************\n");
}

/*******************************************************************************
 EXP 2. REPORT USV REMOTE CONTROL
 ******************************************************************************/ 
void fcn_receive_exp2(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT USV REMOTE CONTROL (EXP #2)\n");
    ReportUSVRemoteControl2Message msgExp = reportUSVRemoteControl2MessageFromJausMessage(msg);
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_APPLIED_DIRECTION_BIT)){
        printf("Recibido parámetro: RUMBO APLICADO con valor: %f\n",msgExp->applied_direction);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_REQUESTED_RPM_BIT)){
        printf("Recibido parámetro: RPM REQUERIDAS con valor: %f\n",msgExp->requested_rpm);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_REQUESTED_RUDDER_ANGLE_BIT)){
        printf("Recibido parámetro: ANGULO DE TIMON REQUERIDO con valor: %f\n",msgExp->requested_rudder_angle);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_APPLIED_RPM_M1_BIT)){
        printf("Recibido parámetro: RPM M1 APLICADAS con valor: %f\n",msgExp->applied_rpm_m1);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_APPLIED_RPM_M2_BIT)){
        printf("Recibido parámetro: RPM M2 APLICADAS con valor: %f\n",msgExp->applied_rpm_m2);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_APPLIED_RUDDER_ANGLE_BIT)){
        printf("Recibido parámetro: ANGULO DE TIMON ACTIVO con valor: %f\n",msgExp->applied_rudder_angle);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_VELOCITY_LIMITATIONS_BIT)){
        printf("Recibido parámetro: LIMITACIONES DE VELOCIDAD con valor: %d\n",msgExp->velocity_limitations);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_DIRECTION_LIMITATIONS_BIT)){
        printf("Recibido parámetro: LIMITACIONES DE RUMBO con valor: %d\n",msgExp->direction_limitations);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_2_PV_MODE_SWITCHING_STATUS_BIT)){
        printf("Recibido parámetro: ESTADO DEL CAMBIO DE MODO con valor: %d\n",msgExp->mode_switching_status);
    }
    printf("*****************************\n");
}

/*******************************************************************************
 EXP 3. USV INFO
 ******************************************************************************/ 
void fcn_receive_exp3(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: USV INFO(EXP #3)\n");
    USVInfo3Message msgExp = usvInfo3MessageFromJausMessage(msg);
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_ACTIVE_RUDDER_ANGLE_BIT)){
        printf("Recibido parámetro: ANGULO DE TIMON ACTIVO con valor: %f\n",msgExp->active_rudder_angle);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_ACTIVE_RPM_M1_BIT)){
        printf("Recibido parámetro: RPM M1 ACTIVO con valor: %f\n",msgExp->active_rpm_m1);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_ACTIVE_RPM_M2_BIT)){
        printf("Recibido parámetro: RPM M2 ACTIVO con valor: %f\n",msgExp->active_rpm_m2);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_FUEL_LEVEL_BIT)){
        printf("Recibido parámetro: NIVEL DE COMBUSTIBLE con valor: %f\n",msgExp->fuel_level);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_PRESSURE_M1_BIT)){
        printf("Recibido parámetro: PRESION M1 con valor: %f\n",msgExp->pressure_m1);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_PRESSURE_M2_BIT)){
        printf("Recibido parámetro: PRESION M2 con valor: %f\n",msgExp->pressure_m2);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_TEMPERATURE_M1_BIT)){
        printf("Recibido parámetro: TEMPERATURA M1 con valor: %f\n",msgExp->temperature_m1);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_TEMPERATURE_M2_BIT)){
        printf("Recibido parámetro: TEMPERATURA M2 con valor: %f\n",msgExp->temperature_m2);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_VOLTAGE_M1_BIT)){
        printf("Recibido parámetro: TENSION BAT M1 con valor: %f\n",msgExp->voltage_m1);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_VOLTAGE_M2_BIT)){
        printf("Recibido parámetro: TENSION BAT M2 con valor: %f\n",msgExp->voltage_m2);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_3_PV_ALARMS_BIT)){
        printf("Recibido parámetro: ALARMAS M2 con valor: %f\n",msgExp->alarms);
    }
    
    printf("*****************************\n");
}

/*******************************************************************************
 EXP 4. ADDITIONAL GPS/INS INFO
 ******************************************************************************/ 
void fcn_receive_exp4(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: ADDITIONAL GPS/INS INFO(EXP #4)\n");
    AditionalGPSINSInfo4Message msgExp = aditionalGPSINSInfo4MessageFromJausMessage(msg);
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_LONGITUDINAL_ACC_BIT)){
        printf("Recibido parámetro: ACC LONGITUDINAL con valor: %f\n",msgExp->longitudinal_acc);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_LATERAL_ACC_BIT)){
        printf("Recibido parámetro: ACC LATERAL con valor: %f\n",msgExp->longitudinal_acc);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_VERTICAL_ACC_BIT)){
        printf("Recibido parámetro: ACC VERTICAL con valor: %f\n",msgExp->vertical_acc);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_GPSINS_STATUS_BIT)){
        printf("Recibido parámetro: ESTADO INS/GPS con valor: %d\n",msgExp->gpsins_status);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_MEASURE_QUALITY_BIT)){
        printf("Recibido parámetro: CALIDAD DE LA MEDIDA con valor:\n ");
        for(int ind=0;ind<3;ind++) printf("P%d: %f\n",ind+1,msgExp->measure_quality[0]);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_ST_LAT_DEVIATION_BIT)){
        printf("Recibido parámetro: ST DESV LATITUD con valor: %f\n",msgExp->st_lat_deviation);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_ST_LON_DEVIATION_BIT)){
        printf("Recibido parámetro: ST DESV LONGITUD con valor: %f\n",msgExp->st_lon_deviation);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_ST_ALT_DEVIATION_BIT)){
        printf("Recibido parámetro: ST DESV ALTITUD con valor: %f\n",msgExp->st_alt_deviation);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_DGPS_CORRECTIONS_BIT)){
        if(msgExp->dgps_corrections) printf("Recibido parámetro: CORRECCIONES DGPS ON\n");
        else printf("Recibido parámetro: CORRECCIONES DGPS OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_4_PV_GPSINS_AVAILABILITY_BIT)){
        if(msgExp->gpsins_availability) printf("Recibido parámetro: GPSINS DISPONIBLE ON\n");
        else printf("Recibido parámetro: GPSINS DISPONIBLE OFF\n");
    }
    
}

/*******************************************************************************
 EXP 5. ANEMOMETER INFO
 ******************************************************************************/ 
void fcn_receive_exp5(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: ANEMOMETER INFO(EXP #5)\n");
    AnemometerInfo5Message msgExp = anemometerInfo5MessageFromJausMessage(msg);
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_5_PV_ANEMOMETER_AVAILABILITY_BIT)){
        if(msgExp->anemometer_availability) printf("Recibido parámetro: DISPONIBILIDAD DEL ANEMOMETRO ON\n");
        else printf("Recibido parámetro: DISPONIBILIDAD DEL ANEMOMETRO OFF\n");
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_5_PV_WIND_VELOCITY_BIT)){
        printf("Recibido parámetro: VELOCIDAD DEL VIENTO con valor: %f\n",msgExp->wind_velocity);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_5_PV_WIND_DIRECTION_BIT)){
        printf("Recibido parámetro: DIRECCION DEL VIENTO con valor: %f\n",msgExp->wind_direction);
    }
}

/*******************************************************************************
 EXP 6. SCPM INFO
 ******************************************************************************/ 
void fcn_receive_exp6(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SCPM INFO(EXP #6)\n");
    SCPMInfo6Message msgExp = scpmInfo6MessageFromJausMessage(msg);
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_6_PV_WAVE_ALTITUDE_BIT)){
        printf("Recibido parámetro: EST ALTURA DE LA OLA con valor: %f\n",msgExp->wave_altitude);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_6_PV_WAVE_FREQUENCY_BIT)){
        printf("Recibido parámetro: EST FRECUENCIA DE LA OLA con valor: %f\n",msgExp->wave_frequency);
    }
}

/*******************************************************************************
 EXP 7. SET USV OBSERVATIONS CONFIG
 ******************************************************************************/ 
void fcn_receive_exp7(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET USV OBSERVARIONS CONFIG(EXP #7)\n");
    SetUSVObservationsConfig7Message msgExp = setUSVObservationsConfig7MessageFromJausMessage(msg);
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_7_PV_PAN_BIT)){
        printf("Recibido parámetro: PAN con valor: %f\n",msgExp->pan);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_7_PV_TILT_BIT)){
        printf("Recibido parámetro: TILT con valor: %f\n",msgExp->tilt);
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_7_PV_DAY_NIGHT_FUNCTION_BIT)){
        if(msgExp->day_night_function) printf("Recibido parámetro: FUNCION DIA/NOCHE ON\n");
        else printf("Recibido parámetro: FUNCION DIA/NOCHE OFF\n");
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_7_PV_INFRARED_FILTER_BIT)){
        if(msgExp->infrared_filter) printf("Recibido parámetro: FILTRO INFRARROJO ON\n");
        else printf("Recibido parámetro: FILTRO INFRARROJO OFF\n");
    }
    if(jausByteIsBitSet(msgExp->presenceVector, JAUS_7_PV_TRACKING_FUNCTION_BIT)){
        if(msgExp->tracking_function) printf("Recibido parámetro: FUNCION TRACKING ON\n");
        else printf("Recibido parámetro: FUNCION TRACKING OFF\n");
    }
    
}

/*******************************************************************************
 EXP 8. REPORT USV OBSERVATIONS CONFIG
 ******************************************************************************/
void fcn_receive_exp8(OjCmpt comp, JausMessage msg) {
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT USV OBSERVARIONS CONFIG(EXP #8)\n");
    ReportUSVObservationsConfig8Message msgExp = reportUSVObservationsConfig8MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_8_PV_PAN_BIT)) {
        printf("Recibido parámetro: PAN ACTUAL con valor: %f\n", msgExp->active_pan);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_8_PV_TILT_BIT)) {
        printf("Recibido parámetro: TILT ACTUAL con valor: %f\n", msgExp->active_tilt);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_8_PV_DAY_NIGHT_FUNCTION_BIT)) {
        if (msgExp->active_day_night_function) printf("Recibido parámetro: FUNCION DIA/NOCHE ACTUAL ON\n");
        else printf("Recibido parámetro: FUNCION DIA/NOCHE OFF\n");
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_8_PV_INFRARED_FILTER_BIT)) {
        if (msgExp->active_infrared_filter) printf("Recibido parámetro: FILTRO INFRARROJO ACTUALON\n");
        else printf("Recibido parámetro: FILTRO INFRARROJO ACTUAL OFF\n");
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_8_PV_TRACKING_FUNCTION_BIT)) {
        if (msgExp->active_tracking_function) printf("Recibido parámetro: FUNCION TRACKING ACTUAL ON\n");
        else printf("Recibido parámetro: FUNCION TRACKING ACTUAL OFF\n");
    }
}

/*******************************************************************************
 EXP 10. TELEMETER INFO
 ******************************************************************************/ 
void fcn_receive_exp10(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: TELEMETER INFO(EXP #10)\n");
    TelemeterInfo10Message msgExp = telemeterInfo10MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_10_PV_SHOOT_BIT)) {
        if (msgExp->shoot) printf("Recibido parámetro: DISPARO ON\n");
        else printf("Recibido parámetro: DISPARO OFF\n");
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_10_PV_ECHOES_BIT)) {
        printf("Recibido parámetro: ECOS:\n");
        for(int ind=0;ind<5;ind++) printf("E%d: %f\n",ind+1,msgExp->echoes[ind]);
    }
}

/*******************************************************************************
 EXP 11. SET SCIENTIFICS OPERATIONS
 ******************************************************************************/ 
void fcn_receive_exp11(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET SCIENTIFICS OPERATIONS(EXP #11)\n");
    SetScientificsOperations11Message msgExp = setScientificsOperations11MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_11_PV_MEASURE_PANEL_BIT)) {
        printf("Recibido parámetro: PANEL DE MEDIDAS con valor: %d\n", msgExp->measure_panel);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_11_PV_NOF_MEASURE_ORDER_BIT)) {
        printf("Recibido parámetro: ORDEN NUMERO MEDIDAS con valor: %d\n", msgExp->nof_measure_order);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_11_PV_MEASURE_STEP_BIT)) {
        printf("Recibido parámetro: ESCALON ENTRE MEDIDAS con valor: %d\n", msgExp->measure_step);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_11_PV_SONAR_CONFIG_SLANT_RANGE_BIT)) {
        printf("Recibido parámetro: CONFIG SONAR (SLANT RANGE) con valor: %f\n", msgExp->sonar_config_slant_range);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_11_PV_SONAR_CONFIG_BEARING_BIT)) {
        printf("Recibido parámetro: CONFIG SONAR (BEARING) con valor: %f\n", msgExp->sonar_config_bearing);
    }
}

/*******************************************************************************
 EXP 12. UGV INFO
 ******************************************************************************/ 
void fcn_receive_exp12(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: UGV INFO(EXP #12)\n");
    UGVInfo12Message msgExp = ugvInfo12MessageFromJausMessage(msg);

    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_BATTERY_LEVEL_BIT)) {
        printf("Recibido parámetro: NIVEL DE BATERIA con valor: %f\n", msgExp->battery_level);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_BATTERY_VOLTAGE_BIT)) {
        printf("Recibido parámetro: TENSION DE BATERIA con valor: %f\n", msgExp->battery_voltage);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_BATTERY_CURRENT_BIT)) {
        printf("Recibido parámetro: INTENSIDAD DE BATERIA con valor: %f\n", msgExp->battery_current);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_BATTERY_TEMPERATURE_BIT)) {
        printf("Recibido parámetro: TEMPERATURA DE BATERIA con valor: %f\n", msgExp->battery_temperature);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_MOTOR_TEMPERATURE_BIT)) {
        printf("Recibido parámetro: TEMPERATURA DE MOTOR con valor: %f\n", msgExp->motor_temperature);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_MOTOR_RPM_BIT)) {
        printf("Recibido parámetro: RPM DE BATERIA con valor: %f\n", msgExp->motor_rpm);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_12_PV_ALARMS_BIT)) {
        printf("Recibido parámetro: ALARMAS con valor: %f\n", msgExp->alarms);
    }
}

/*******************************************************************************
 EXP 13. REPORT SCIENTIFICS OPERATIONS
 ******************************************************************************/ 
void fcn_receive_exp13(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT SCIENTIFICS OPERATIONS(EXP #13)\n");
    ReportScientificOperations13Message msgExp = reportScientificOperations13MessageFromJausMessage(msg);

    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_SOUND_SPEED_BIT)) {
        printf("Recibido parámetro: VELOCIDAD VIENTO con valor: %f\n", msgExp->sound_speed);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_FLOW_VALUE_BIT)) {
        printf("Recibido parámetro: VALOR DE CORRIENTE con valor: %f\n", msgExp->flow_value);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_FLOW_DEPTH_BIT)) {
        printf("Recibido parámetro: PROFUNDIDAD MEDIDA CORRIENTE con valor: %f\n", msgExp->flow_depth);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_FLOW_DIRECTION_BIT)) {
        printf("Recibido parámetro: DIRECCION VIENTO con valor: %f\n", msgExp->flow_direction);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_WATER_SOUND_SPEED_BIT)) {
        printf("Recibido parámetro: VELOCIDAD SONIDO AGUA con valor: %f\n", msgExp->water_sound_speed);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_WATER_SOUND_DEPTH_BIT)) {
        printf("Recibido parámetro: PROFUNDIDAD MEDIDA  SONIDO AGUA con valor: %f\n", msgExp->water_sound_depth);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_CONTACT_ID_BIT)) {
        printf("Recibido parámetro: ID CONTACTO con valor: %d\n", msgExp->contact_id);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_CONTACT_DELAY_BIT)) {
        printf("Recibido parámetro: DEMORA CONTACTO con valor: %f\n", msgExp->contact_delay);
    }
    if (jausShortIsBitSet(msgExp->presenceVector, JAUS_13_PV_CONTACT_DISTANCE_BIT)) {
        printf("Recibido parámetro: DISTANCIA CONTACTO con valor: %f\n", msgExp->contact_distance);
    }
   
}

/*******************************************************************************
 EXP 14. SET LIST OF WAYPOINTS
 ******************************************************************************/ 
void fcn_receive_exp14(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET LIST OF WAYPOINTS(EXP #14)\n");
    SetListOfWaypoints14Message msgExp = setListOfWaypoints14MessageFromJausMessage(msg);
    // Identificador de lista
    printf("Recibido parámetro: ID DE LISTA con valor: %d\n", msgExp->waypoints_list_id);
    // Identificador de lista
    printf("Recibido parámetro: NUMERO DE WPs con valor: %d\n", msgExp->nof_waypoints);
    // Listas
    printf("Recibido parámetro: LISTA DE IDENTIFICADORES DE WPs\n");
    for(int ind=0;ind<msgExp->nof_waypoints;ind++){
        printf("ID WP %d: %d LAT %d: %f LON %d: %f VEL %d: %f\n",ind+1, msgExp->waypoints_ids_list[ind],ind+1, msgExp->latitudes_list[ind],ind+1, msgExp->longitudes_list[ind],ind+1, msgExp->velocities_list[ind]);
    }
}

/*******************************************************************************
 EXP 15. REPORT LIST OF WAYPOINTS
 ******************************************************************************/ 
void fcn_receive_exp15(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT LIST OF WAYPOINTS(EXP #15)\n");
    ReportListOfWaypoints15Message msgExp = reportListOfWaypoints15MessageFromJausMessage(msg);
    // Identificador de lista
    printf("Recibido parámetro: ID DE LISTA con valor: %d\n", msgExp->objetive_list_id);
    // Identificador de WP
    printf("Recibido parámetro: ID DE WP con valor: %d\n", msgExp->objetive_waypoint_id);

}

/*******************************************************************************
 EXP 16. HEARTBEAT - CHANNEL STATE MESSAGE OPERATIONS
 ******************************************************************************/ 
void fcn_receive_exp16(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: HEARTBEAT - CHANNEL STATE MESSAGE OPERATIONS(EXP #16)\n");
    HeartbeatChannelState16Message msgExp = heartbeatChannelState16MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_16_PV_NODE_ID_BIT)) {
        printf("Recibido parámetro: ID DE NODO con valor: %d\n", msgExp->node_id);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_16_PV_PRIMARY_CHANNEL_STATUS_BIT)) {
       printf("Recibido parámetro: ESTADO CANAL PRIMARIO con valor: %d\n", msgExp->primary_channel_status);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_16_PV_BACKUP_CHANNEL_STATUS_BIT)) {
        printf("Recibido parámetro: ESTADO CANAL BACKUP con valor: %d\n", msgExp->backup_channel_status);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_16_PV_PRIMARY_CHANNEL_SNR_BIT)) {
        printf("Recibido parámetro: SNR CANAL PRIMARIO con valor: %f\n", msgExp->primary_channel_snr);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_16_PV_BACKUP_CHANNEL_SNR_BIT)) {
        printf("Recibido parámetro: SNR CANAL BACKUP con valor: %f\n", msgExp->backup_channel_snr);
    }
   
}

/*******************************************************************************
 EXP 17. HEARTBEAT - POSITION INFO
 ******************************************************************************/ 
void fcn_receive_exp17(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: HEARTBEAT - POSITION INFO(EXP #17)\n");
    HeartbeatPositionInfo17Message msgExp = heartbeatPositionInfo17MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_17_PV_LATITUDE_BIT)) {
        printf("Recibido parámetro: LATITUD con valor: %f\n", msgExp->latitude);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_17_PV_LONGITUDE_BIT)) {
       printf("Recibido parámetro: LONGITUD con valor: %f\n", msgExp->longitude);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_17_PV_ALTITUDE_BIT)) {
        printf("Recibido parámetro: ALTITUD con valor: %f\n", msgExp->altitude);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_17_PV_HEADING_BIT)) {
        printf("Recibido parámetro: HEADING con valor: %f\n", msgExp->heading);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_17_PV_SPEED_BIT)) {
        printf("Recibido parámetro: VELOCIDAD con valor: %f\n", msgExp->speed);
    }
}

/*******************************************************************************
 EXP 18. SET SIGNALING ELEMENTS
 ******************************************************************************/ 
void fcn_receive_exp18(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET SIGNALING ELEMENTS(EXP #18)\n");
    SetSignalingElements18Message msgExp = setSignalingElements18MessageFromJausMessage(msg);
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_DIPSS_BIT)){
        if(msgExp->dipss) printf("Recibido parámetro: DIPSS ON\n");
        else printf("Recibido parámetro: DIPSS OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_DIPSP_BIT)){
        if(msgExp->dipsp) printf("Recibido parámetro: DIPSP ON\n");
        else printf("Recibido parámetro: DIPSP OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_DIPSR_BIT)){
        if(msgExp->dipsr) printf("Recibido parámetro: DIPSR ON\n");
        else printf("Recibido parámetro: DIPSR OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_BKINKER_LEFT_BIT)){
        if(msgExp->blinker_left) printf("Recibido parámetro: INTERMITENTE IZQUIERDA ON\n");
        else printf("Recibido parámetro: INTERMITENTE IZQUIERDA OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_BLINKER_RIGHT_BIT)){
        if(msgExp->blinker_right) printf("Recibido parámetro: NTERMITENTE DERECHA ON\n");
        else printf("Recibido parámetro: INTERMITENTE DERECHA OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_18_PV_KLAXON_BIT)){
        if(msgExp->klaxon) printf("Recibido parámetro: CLAXON ON\n");
        else printf("Recibido parámetro: CLAXON OFF\n");
    }
}

/*******************************************************************************
 EXP 19. SET POSITIONER
 ******************************************************************************/ 
void fcn_receive_exp19(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET POSITIONER(EXP #19)\n");
    SetPositioner19Message msgExp = setPositioner19MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_19_PV_PAN_BIT)) {
        printf("Recibido parámetro: PAN con valor: %f\n", msgExp->pan);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_19_PV_TILT_BIT)) {
       printf("Recibido parámetro: TILT con valor: %f\n", msgExp->tilt);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_19_PV_SPIN_VELOCITY_BIT)) {
        printf("Recibido parámetro: VELOCIDAD GIRO con valor: %f\n", msgExp->spin_velocity);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_19_PV_ELEVATION_VELOCITY_BIT)) {
        printf("Recibido parámetro: VELOCIDAD ELEVACION con valor: %f\n", msgExp->elevation_velocity);
    }

}

/*******************************************************************************
 EXP 20. REPORT POSITIONER
 ******************************************************************************/ 
void fcn_receive_exp20(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT POSITIONER(EXP #20)\n");
    ReportPositioner20Message msgExp = reportPositioner20MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_20_PV_PAN_BIT)) {
        printf("Recibido parámetro: PAN ACTIVO con valor: %f\n", msgExp->active_pan);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_20_PV_TILT_BIT)) {
       printf("Recibido parámetro: TILT ACTIVO con valor: %f\n", msgExp->active_tilt);
    }
}

/*******************************************************************************
 EXP 21. SET DAY-TIME CAMERA
 ******************************************************************************/ 
void fcn_receive_exp21(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET DAY-TIME CAMERA(EXP #21)\n");
    SetDayTimeCamera21Message msgExp = setDayTimeCamera21MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_21_PV_DIRECT_ZOOM_BIT)) {
        printf("Recibido parámetro: ZOOM DIRECTO con valor: %f\n", msgExp->direct_zoom);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_21_PV_CONTINUOUS_ZOOM_BIT)) {
       printf("Recibido parámetro: ZOOM CONTINUO con valor: %d\n", msgExp->continuous_zoom);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_21_PV_FOCUS_BIT)) {
       printf("Recibido parámetro: FOCO con valor: %f\n", msgExp->focus);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_21_PV_AUTOFOCUS_BIT)){
        if(msgExp->autofocus) printf("Recibido parámetro: AUTOFOCO ON\n");
        else printf("Recibido parámetro: AUTOFOCO OFF\n");
    }
}

/*******************************************************************************
 EXP 22. REPORT DAY-TIME CAMERA
 ******************************************************************************/ 
void fcn_receive_exp22(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT DAY-TIME CAMERA(EXP #22)\n");
    ReportDayTimeCamera22Message msgExp = reportDayTimeCamera22MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_22_PV_ZOOM_BIT)) {
        printf("Recibido parámetro: ZOOM ACTUAL con valor: %d\n", msgExp->active_zoom);
    }
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_22_PV_FOCUS_BIT)) {
       printf("Recibido parámetro: FOCO ACTUAL con valor: %f\n", msgExp->active_focus);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_22_PV_AUTOFOCUS_BIT)){
        if(msgExp->active_autofocus) printf("Recibido parámetro: AUTOFOCO ACTUAL ON\n");
        else printf("Recibido parámetro: AUTOFOCO ACTUAL OFF\n");
    }
}

/*******************************************************************************
 EXP 23. SET NIGHT-TIME CAMERA
 ******************************************************************************/
void fcn_receive_exp23(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: SET NIGHT-TIME CAMERA(EXP #23)\n");
    SetNightTimeCamera23Message msgExp = setNightTimeCamera23MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_23_PV_ZOOM_BIT)) {
        printf("Recibido parámetro: ZOOM con valor: %d\n", msgExp->zoom);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_23_PV_POLARITY_BIT)){
        if(msgExp->polarity) printf("Recibido parámetro: POLARIDAD ON\n");
        else printf("Recibido parámetro: POLARIDAD OFF\n");
    }
}

/*******************************************************************************
 EXP 24. REPORT NIGHT-TIME CAMERA
 ******************************************************************************/ 
void fcn_receive_exp24(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT NIGHT-TIME CAMERA(EXP #24)\n");
    ReportNightTimeCamera24Message msgExp = reportNightTimeCamera24MessageFromJausMessage(msg);
    if (jausByteIsBitSet(msgExp->presenceVector, JAUS_24_PV_ZOOM_BIT)) {
        printf("Recibido parámetro: ZOOM ACTIVO con valor: %d\n", msgExp->active_zoom);
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_24_PV_POLARITY_BIT)){
        if(msgExp->active_polarity) printf("Recibido parámetro: POLARIDAD ON\n");
        else printf("Recibido parámetro: POLARIDAD ACTUAL OFF\n");
    }
}

/*******************************************************************************
 EXP 25. REPORT SIGNALING ELEMENTS
 ******************************************************************************/
void fcn_receive_exp25(OjCmpt comp, JausMessage msg){
    printf("*****************************\n");
    printf("Recibido mensaje: REPORT SIGNALING ELEMENTS(EXP #25)\n");
    ReportSignalingElements25Message msgExp = reportSignalingElements25MessageFromJausMessage(msg);
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_DIPSS_BIT)){
        if(msgExp->dipss) printf("Recibido parámetro: DIPSS ACTUAL ON\n");
        else printf("Recibido parámetro: DIPSS ACTUAL OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_DIPSP_BIT)){
        if(msgExp->dipsp) printf("Recibido parámetro: DIPSP ACTUAL ON\n");
        else printf("Recibido parámetro: DIPSP ACTUAL OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_DIPSR_BIT)){
        if(msgExp->dipsr) printf("Recibido parámetro: DIPSR ACTUAL ON\n");
        else printf("Recibido parámetro: DIPSR ACTUAL OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_BKINKER_LEFT_BIT)){
        if(msgExp->blinker_left) printf("Recibido parámetro: INTERMITENTE IZQUIERDA ACTUAL ON\n");
        else printf("Recibido parámetro: INTERMITENTE IZQUIERDA ACTUAL OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_BLINKER_RIGHT_BIT)){
        if(msgExp->blinker_right) printf("Recibido parámetro: NTERMITENTE DERECHA ACTUAL ON\n");
        else printf("Recibido parámetro: INTERMITENTE DERECHA ACTUAL OFF\n");
    }
    if(jausShortIsBitSet(msgExp->presenceVector, JAUS_25_PV_KLAXON_BIT)){
        if(msgExp->klaxon) printf("Recibido parámetro: CLAXON ACTUAL ON\n");
        else printf("Recibido parámetro: CLAXON ACTUAL OFF\n");
    }
}
