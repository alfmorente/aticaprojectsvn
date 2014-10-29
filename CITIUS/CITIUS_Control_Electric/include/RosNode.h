
/** 
 * @file  RosNode.h
 * @brief Declara el tipo de la clase "RosNode"
 * - La clase se presenta como la superclase de la cual hereda cada nodo ROS 
 * específico para la abstracción de las funciones comunes
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup CommonRosNode
 * @{
 */

#ifndef ROSNODE_H
#define	ROSNODE_H

#include "constant.h"

using namespace std;

/**
 * \class RosNode
 * \brief Superclase con métodos comunes a todos los nodos ROS
*/
class RosNode{
protected:
  NodeStatus nodeStatus;
public:
  virtual void initROS();
  void setNodeStatus(NodeStatus newNodeStatus);
  NodeStatus getNodeStatus();
};

#endif	/* ROSNODE_H */

/**
 * @}
 */