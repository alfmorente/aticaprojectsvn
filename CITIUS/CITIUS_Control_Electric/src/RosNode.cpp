/* 
 * File:   RosNode.cpp
 * Author: atica
 *
 * Created on 15 de octubre de 2014, 12:03
 */

#include <cstdlib>
#include "RosNode.h"

using namespace std;

void RosNode::initROS(){}

short RosNode::getNodeStatus(){
  return nodeStatus;
}

void RosNode::setNodeStatus(short newNodeStatus){
  nodeStatus = newNodeStatus;
}





