/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 2 de marzo de 2015, 11:09
 */

#include <cstdlib>
#include "RosNode_Joystick.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) {
  ros::init(argc,argv,"RcvJoystick");
  RosNode_Joystick *joy = new RosNode_Joystick();
  joy->initROS();
  ros::spin();
  return 0;
}

