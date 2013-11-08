/* 
 * File:   main.cpp
 * Author: carlosamores
 *
 * Created on 21 de marzo de 2013, 10:19
 */

#include <cstdlib>
#include "../include/GPS_Management/GPS_Management.h"
#include <ros/ros.h>
using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps");
  ROS_INFO("Esperando ROS_MASTER_URI....");
  while(!ros::master::check())	
	sleep(1);
  ROS_INFO("ROS_MASTER_URI conectado....");
  Bestgpsvel gpsvel;
  GPS_Management *gps = new GPS_Management();

  if(gps->isPortOpened()){

        //gps->gps_log_general("bestgpsposa", "");
        //gps->rcvData();

        cout << "Configurando (Modo de alineamiento inicial)..." << endl;
        while (!gps->gps_conf_alignmentmode(ALIGNMENTMODE_UNAIDED));
        cout << "Establecido modo de alineamiento - KINEMATIC" << endl;

        cout << "Configuracion (Azimuth inicial)..." << endl;
        while (!gps->gps_conf_setinitazimuth(0.0, 5));
        cout << "Azimuth OK" << endl;
        /*
        cout << "Configurando offset de la antena..." << endl;
        while (!gps->gps_conf_setimutoantoffset(-35,15,62,0,0,0));
        cout << "Offset antena OK" << endl;
        */
        gps->gps_log_general("bestgpsposa", "ontime 1");
        while (gps->getGPSPos().sol_status != "SOL_COMPUTED") {
            gps->rcvData();
            cout << "Esperando Alineamiento de GPS: " << gps->getGPSPos().sol_status << endl;
        }
        //gps->gps_log_general("bestgpsposa", "");
        //gps->rcvData();
        
        //cout << "Comenzar movimiento 4km/h para alinear IMU" << endl;

        gps->gps_log_general("inspvasa", "ontime 1");
        while (gps->getInspVas().status != "INS_SOLUTION_GOOD") {
            gps->rcvData();
            cout << "Esperando Alineamiento de IMU: " << gps->getInspVas().status << endl;
        }
        gps->gps_log_general("corrimudatasa", "ontime 1");

        int i = 0;
        double vel_mod = 0.0;

        // Fichero a guardar
        FILE *fichero;
        char nombre[12] = "datos2.txt";

        fichero = fopen(nombre, "w");
        printf("Fichero: %s (para escritura) -> ", nombre);
        if (fichero)
            printf("Fichero abierto correctamente\n");
        else {
            printf("No se ha podido abrir el fichero\n");
            return 1;
        }


        int flagInspvas = false, flagCorrimudatasa = false, flagBestGPSPosa = false, flagHeadinga = false;
        int typeFrame;
        while (ros::ok()) {
            typeFrame = gps->rcvData();
            if (typeFrame == TT_INSPVASA)
                flagInspvas = true;
            else if (typeFrame == TT_CORRIMUDATASA)
                flagCorrimudatasa = true;
            else if (typeFrame == TT_BESTGPSPOSA)
                    flagBestGPSPosa = true;
            else if (typeFrame == TT_HEADINGA)
                    flagHeadinga = true;
            else if (typeFrame == TT_ERROR) {
                cerr << "Tiempo expirado: No se reciben datos del GPS" << endl;
                return 1;
            }

            int insStatus = 0, solStatus = 0;
            if (flagInspvas && flagCorrimudatasa && flagBestGPSPosa && flagHeadinga) {
               
                // Volcado de datos
                fprintf(fichero, "%.8f ", gps->getInspVas().seconds);

                fprintf(fichero, "%.8f ", gps->getGPSPos().lat);
                fprintf(fichero, "%.8f ", gps->getGPSPos().lon);
                fprintf(fichero, "%.8f ", gps->getGPSPos().hgt);
                fprintf(fichero, "%.8f ", gps->getGPSPos().lat_dev);
                fprintf(fichero, "%.8f ", gps->getGPSPos().lon_dev);
                fprintf(fichero, "%.8f ", gps->getGPSPos().hgt_dev);
                fprintf(fichero, "%c ", gps->getGPSPos().l1);
                fprintf(fichero, "%c ", gps->getGPSPos().l2);

                fprintf(fichero, "%.8f ", gps->getInspVas().roll);
                fprintf(fichero, "%.8f ", gps->getInspVas().pitch);
                fprintf(fichero, "%.8f ", gps->getInspVas().azimuth);

                fprintf(fichero, "%.8f ", gps->getInspVas().north_velocity);
                fprintf(fichero, "%.8f ", gps->getInspVas().east_velocity);
                vel_mod = sqrt(pow(gps->getInspVas().north_velocity, 2) + pow(gps->getInspVas().east_velocity, 2));
                fprintf(fichero, "%.8f ", vel_mod);

                fprintf(fichero, "%.8f ", gps->getCorrIMUData().roll_rate);
                fprintf(fichero, "%.8f ", gps->getCorrIMUData().pitch_rate);
                fprintf(fichero, "%.8f ", gps->getCorrIMUData().yaw_rate);
                fprintf(fichero, "%.8f ", gps->getCorrIMUData().lateral_acc);
                fprintf(fichero, "%.8f ", gps->getCorrIMUData().longitudinal_acc);
                fprintf(fichero, "%.8f ", gps->getCorrIMUData().vertical_acc);

                if (gps->getInspVas().status == "INS_INACTIVE")
                    insStatus = 0;
                else if (gps->getInspVas().status == "INS_ALIGNING")
                    insStatus = 1;
                else if (gps->getInspVas().status == "INS_SOLUTION_NOT_GOOD")
                    insStatus = 2;
                else if (gps->getInspVas().status == "INS_SOLUTION_GOOD")
                    insStatus = 3;
                else if (gps->getInspVas().status == "INS_BAD_GPS_AGREEMENT")
                    insStatus = 6;
                else if (gps->getInspVas().status == "INS_ALIGNMENT_COMPLETE")
                    insStatus = 7;
                
                if (gps->getGPSPos().sol_status == "SOL_COMPUTED")
                    solStatus=0;
                else if (gps->getGPSPos().sol_status == "INSUFFICIENT_OBS")
                    solStatus=1;
                else if (gps->getGPSPos().sol_status == "NO_CONVERGENCE")
                    solStatus=2;
                else if (gps->getGPSPos().sol_status == "SINGULARITY")
                    solStatus=3;
                else if (gps->getGPSPos().sol_status == "COV_TRACE")
                    solStatus=4;
                else if (gps->getGPSPos().sol_status == "TEST_DIST")
                    solStatus=5;
                else if (gps->getGPSPos().sol_status == "COLD_START")
                    solStatus=6;
                else if (gps->getGPSPos().sol_status == "V_H_LIMIT")
                    solStatus=7;
                else if (gps->getGPSPos().sol_status == "VARIANCE")
                    solStatus=8;
                else if (gps->getGPSPos().sol_status == "RESIDUALS")
                    solStatus=9;
                else if (gps->getGPSPos().sol_status == "DELTA_POS")
                    solStatus=10;
                else if (gps->getGPSPos().sol_status == "NEGATIVE_VAR")
                    solStatus=11;
                else if (gps->getGPSPos().sol_status == "INTEGRITY_WARNING")
                    solStatus=13;
                else if (gps->getGPSPos().sol_status == "IMU_UNPLUGGED")
                    solStatus=17;
                else if (gps->getGPSPos().sol_status == "PENDING")
                    solStatus=18;
                else if (gps->getGPSPos().sol_status == "INVALID_FIX")
                    solStatus=19;


                fprintf(fichero, "%d ", insStatus);
                fprintf(fichero, "%c ", gps->getHeading().num_satellites);
                fprintf(fichero, "%d\n", solStatus);

                //fprintf(fichero,"%s\n",gps->getInspVas().status.c_str());
                fflush(fichero);
                //printf("Escribiendo en el fichero latitud %f\n", latitud);
                i++;
                flagCorrimudatasa = false;
                flagInspvas = false;
                flagBestGPSPosa =false;
                flagHeadinga = false;

                // Muestreo por pantalla
                cout << "==========================================================" << endl;
                cout << "DATOS BESTGPSPOS - Estado: " << gps->getGPSPos().sol_status << endl;
                cout << "Latitud: " << gps->getGPSPos().lat << "; Longitud: " << gps->getGPSPos().lon << "; Altitud: " << gps->getGPSPos().hgt << endl;
                cout << "Desviacion latitud: " << gps->getGPSPos().lat_dev << "; Desviacion longitud: " << gps->getGPSPos().lon_dev << "; Desviacion altitud: " << gps->getGPSPos().hgt_dev << endl;
                cout << "L1: " << gps->getGPSPos().l1 << "; L2: " << gps->getGPSPos().l2 << endl;
                cout << "~~~~~~~~~~~~~~~~~~~~" << endl;
                cout << "DATOS INSPVAS - Estado: " << gps->getInspVas().status << endl;
                cout << "Roll: " << gps->getInspVas().roll << "; Pitch: " << gps->getInspVas().pitch << "; Azimuth: " << gps->getInspVas().azimuth << endl;
                cout << "Velocidad: " << vel_mod << "; Vel. Norte: " << gps->getInspVas().north_velocity << "; Vel. Este: " << gps->getInspVas().east_velocity << endl;
                cout << "~~~~~~~~~~~~~~~~~~~~" << endl;
                cout << "DATOS CORRIMUDATA" << endl;
                cout << "Roll rate: " << gps->getCorrIMUData().roll_rate << "; Pitch rate: " << gps->getCorrIMUData().pitch_rate << "; Yaw rate: " << gps->getCorrIMUData().yaw_rate << endl;
                cout << "Lateral acc: " << gps->getCorrIMUData().lateral_acc << "; Longitudinal acc: " << gps->getCorrIMUData().longitudinal_acc << "; Vertical acc: " << gps->getCorrIMUData().vertical_acc << endl;
                cout << "~~~~~~~~~~~~~~~~~~~~" << endl;
                cout << "DATOS HEADING - Num Satelites: " << gps->getHeading().num_satellites << endl;
            }
        }
      
    
  }
  else
	cout <<"El puerto no se ha abierto"<<endl;

  return 0;
}

