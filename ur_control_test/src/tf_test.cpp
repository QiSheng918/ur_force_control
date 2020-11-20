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

std::vector<double> quaternion2RPY(double x,double y,double z,double w){
    std::vector<double> RPY(3,0);
    RPY[0]=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
    RPY[1]=asin(2*(w*y-z*x));
    RPY[2]=atan2(2*(w*z+x*y),1-2*(z*z+y*y));
    // for(int i=0;i<3;i++) std::cout<<RPY[i]<<",";
    std::cout<<std::endl;
    return RPY;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(50.0);
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
    
// transform.getRotation()
    std::cout<<"the position is: "<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl;
    double x=transform.getRotation().getX();
    double y=transform.getRotation().getY();
    double z=transform.getRotation().getZ();
    double w=transform.getRotation().getW();
    std::cout<<"the orientation is:"<<","<<x<<","<<y<<","<<z<<","<<w<<std::endl;
    // ROS_INFO_STREAM("the orientation is:"<<transform.getRotation().getW()<<","<<transform.getRotation().getX()<<","<<transform.getRotation().getY()<<","<<transform.getRotation().getZ());
    // std::cout<<"the rotation matrix is:\n"<<getR(x,y,z,w)<<std::endl;
    // tf
    Eigen::Matrix3d rotation_matrix=getR(x,y,z,w);
    // std::cout<<rotation_matrix<<std::endl;
    double theta=acos((rotation_matrix(0,0)+rotation_matrix(1,1)+rotation_matrix(2,2)-1)/2);
    double rx=1/(2*sin(theta))*(rotation_matrix(2,1)-rotation_matrix(1,2));
    double ry=1/(2*sin(theta))*(rotation_matrix(0,2)-rotation_matrix(2,0));
    double rz=1/(2*sin(theta))*(rotation_matrix(1,0)-rotation_matrix(0,1));
    // std::cout<<rx*theta<<","<<ry*theta<<","<<rz*theta<<std::endl;
    double roll,pitch,yaw;
    R.getRPY(roll,pitch,yaw);
    // std::cout<<"the RPY is:"<<roll<<","<<pitch<<","<<yaw<<std::endl;
    // std::cout<<roll<<","<<pitch<<","<<yaw<<std::endl;
    quaternion2RPY(x,y,z,w);
    rate.sleep();
  }
  return 0;
};