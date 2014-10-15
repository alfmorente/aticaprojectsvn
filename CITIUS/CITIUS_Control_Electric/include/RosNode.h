/* 
 * File:   RosNode.h
 * Author: atica
 *
 * Created on 15 de octubre de 2014, 12:02
 */

#ifndef ROSNODE_H
#define	ROSNODE_H

class RosNode{
protected:
  short nodeStatus;
public:
  virtual void initROS();
  void setNodeStatus(short newNodeStatus);
  short getNodeStatus();
};

#endif	/* ROSNODE_H */