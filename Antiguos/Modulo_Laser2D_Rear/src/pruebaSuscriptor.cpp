#include "ros/ros.h"
#include "softwareLaser/ScanData.h"
#include <iostream>
#include "std_msgs/String.h"
#include "math.h"
using namespace std;
int i=0;
void chatterCallback(const softwareLaser::ScanData::ConstPtr& msg)
{
	
	//cout <<"Contado: " << i << "   Angle: " << msg->pHor[0] <<"  Distancia: " <<msg->pDist[0] <<endl;
	int j=0;
	for(int i=0;i<msg->nPoints;i++)
	{
		if(msg->pHor[i]==0 || msg->pHor[i]==-90 || msg->pHor[i]==90||msg->pHor[i]==135||msg->pHor[i]==-135||msg->pHor[i]==180||msg->pHor[i]==225)
		printf("Angle: %f  Distancia: %f\n",msg->pHor[i],msg->pDist[i]);
	}
	i++;
}

void chatterCallback2(const std_msgs::String::ConstPtr& msg)
{

	cout << msg->data<<endl;
	exit(1);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("SICK", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("sicklmsError", 1000, chatterCallback2);
  ros::spin();


  return 0;
}

