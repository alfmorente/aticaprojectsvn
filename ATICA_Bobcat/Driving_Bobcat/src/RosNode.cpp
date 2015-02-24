/** 
 * @file  RosNode.cpp
 * @brief Implementacion de la clase "RosNode"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode.h"

/**
 * Método público que inicializa los artefactos ROS atributos de la clase. 
 * Este método se redefine por las  subclases que heredan de RosNode
 */
void RosNode::initROS() {
}

/**
 * Método público consultor del atributo "nodeStatus" de la clase que 
 * proporciona el estado actual de la máquina de estados del nodo
 * @return Atributo "nodeStatus" de la clase
 */
NodeStatus RosNode::getNodeStatus() {
  return nodeStatus;
}

/**
 * Método público modificador del atributo "nodeStatus" de la clase para 
 * realizar una transición en la máquina de estados del nodo
 * @param[in] newNodeStatus Nuevo estado al que realizar la transición
 */
void RosNode::setNodeStatus(NodeStatus newNodeStatus) {
  nodeStatus = newNodeStatus;
}





