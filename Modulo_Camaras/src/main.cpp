/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 14 de mayo de 2014, 14:07
 */

#include <ros/ros.h>
#include <cstdlib>
#include <Modulo_Camaras/ProtPelcoD.h>

#define NAME_SERIAL "/dev/ttyUSB0"
#define VELOCITY B9600
using namespace std;

/*
 * 
 */
int main(int argc, char** argv) 
{
    ProtPelcoD camera(1);
    int option;
    bool exit=false;
    if(!camera.connect(NAME_SERIAL,VELOCITY))
    {
        ROS_INFO("Imposible conectar con la cámara");
        return 1;        
    }
    else
    {
        while(!exit)
        {
            do
            {
                ROS_INFO("Introduzca un comando");
                ROS_INFO("0- EXIT");
                ROS_INFO("1- PAN RIGHT");
                ROS_INFO("2- PAN LEFT");
                ROS_INFO("3- PAN STOP");
                ROS_INFO("4- TILT UP");        
                ROS_INFO("5- TILT DOWN");  
                ROS_INFO("6- TILT STOP");          
                ROS_INFO("7- ZOOM IN");   
                ROS_INFO("8- ZOOM OUT");        
                cin >> option;
                if(option <0 || option >8)
                    ROS_INFO("Option unavailable");
            }while(option <0 || option >8);
            switch(option)
            {
                case 1:
                    camera.commandPAN(CAMERA_PAN_RIGHT);
                    break;
                case 2:
                    camera.commandPAN(CAMERA_PAN_LEFT);
                    break;
                case 3:
                    camera.commandPAN(CAMERA_PAN_STOP);
                    break;
                case 4:
                    camera.commandTILT(CAMERA_TILT_UP);
                    break;
                case 5:
                    camera.commandTILT(CAMERA_TILT_DOWN);
                    break;
                case 6:
                    camera.commandTILT(CAMERA_TILT_STOP);
                    break;
                case 7:
                    camera.commandZOOM(CAMERA_ZOOM_IN);
                    break;
                case 8:
                    camera.commandZOOM(CAMERA_ZOOM_OUT);
                    break;
                default:
                    exit=true;
                    break;

            }

        }
    }
    return 0;
}

