#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "Eigen/Dense"
#include <std_msgs/Bool.h>

bool flag=false;
Eigen::Matrix3d getR(double x,double y,double z,double w){
    Eigen::Matrix3d R;
    R(0,0)=1-2*y*y-2*z*z;
    R(0,1)=2*(x*y-z*w);
    R(0,2)=2*(x*z+y*w);
    R(1,0)=2*(x*y+z*w);
    R(1,1)=1-2*x*x-2*z*z;
    R(1,2)=2*(y*z-x*w);
    R(2,0)=2*(x*z-y*w);
    R(2,1)=2*(y*z+x*w);
    R(2,2)=1-2*x*x-2*y*y;
    return R;
  
}

std::vector<double> quaternion2RPY(double x,double y,double z,double w){
    std::vector<double> RPY(3,0);
    RPY[0]=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    RPY[1]=asin(2*(w*y-z*x));
    RPY[2]=atan2(2*(w*z+x*y),1-2*(z*z+y*y));
    // for(int i=0;i<3;i++) std::cout<<RPY[i]<<",";
    std::cout<<std::endl;
    return RPY;
}

void flagCallback(const std_msgs::BoolConstPtr msg){
	if(msg->data==true) flag=true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_get2point_node");

  ros::NodeHandle node;
	ros::Subscriber sub=node.subscribe("flag",1,flagCallback);
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base", "tool0",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Matrix3x3 R(transform.getRotation());

 
    double roll,pitch,yaw;
    R.getRPY(roll,pitch,yaw);
	if(flag){
		std::cout<<"the position is: "<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl;
		std::cout<<"the RPY is:"<<roll<<","<<pitch<<","<<yaw<<std::endl;
	}
	flag=false;
    rate.sleep();
	ros::spinOnce();
  }
  return 0;
};