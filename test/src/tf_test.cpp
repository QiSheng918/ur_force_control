#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "Eigen/Dense"

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

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base_link", "tool0",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Matrix3x3 R(transform.getRotation());
    
// transform.getRotation().
    double x=transform.getRotation().getX();
    double y=transform.getRotation().getY();
    double z=transform.getRotation().getZ();
    double w=transform.getRotation().getW();
    std::cout<<"the orientation is:"<<","<<x<<","<<y<<","<<z<<","<<w<<std::endl;
    // ROS_INFO_STREAM("the orientation is:"<<transform.getRotation().getW()<<","<<transform.getRotation().getX()<<","<<transform.getRotation().getY()<<","<<transform.getRotation().getZ());
    std::cout<<"the rotation matrix is:\n"<<getR(x,y,z,w)<<std::endl;
    // tf
    double roll,pitch,yaw;
    R.getRPY(roll,pitch,yaw);
    std::cout<<"the RPY is:"<<roll<<","<<pitch<<","<<yaw<<std::endl;
    rate.sleep();
  }
  return 0;
};