
/** 
 * @file  RosNode.h
 * @brief Declara el tipo de la clase "RosNode"
 * - La clase se presenta como la superclase de la cual hereda cada nodo ROS 
 * específico para la abstracción de las funciones comunes
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef ROSNODE_H
#define	ROSNODE_H

using namespace std;

/**
 * /class RosNode
 * /brief Superclase con métodos comunes a todos los nodos ROS
*/
class RosNode{
protected:
  short nodeStatus;
public:
  virtual void initROS();
  void setNodeStatus(short newNodeStatus);
  short getNodeStatus();
};

#endif	/* ROSNODE_H */

/**
 * @}
 */