#include "ros/ros.h"
#include <cmath>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>

  
std::string double2string(double input);
std::string combinemsg(std::vector<double> velocity, double acc = 1);

ros::Publisher  ur_script_pub;
Eigen::Matrix3d rotation_matrix;
std_msgs::Float64MultiArray pub_msg;

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
    return R.transpose();
  
}

void VelocityToolsubCallback(const std_msgs::Float64MultiArray &msg)
{
    
    std::vector<double> tool_vel(6,0);
    for(int i=0;i<6;i++) tool_vel[i]=msg.data[i];
 
    Eigen::Matrix<double,3,1> vel{tool_vel[0],tool_vel[1],tool_vel[2]};
    Eigen::Matrix<double,3,1> base_vel=rotation_matrix*vel;
    for(int i=0;i<3;i++) tool_vel[i]=base_vel[i];
    tool_vel[2]=0;
    for(int i=0;i<6;i++) pub_msg.data[i]=tool_vel[i];
    ur_script_pub.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "space2tool_velocity_node");
    ros::NodeHandle nh;
    pub_msg.data.resize(6);
    ur_script_pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_tool_cmd", 1000);
    ros::Subscriber tool_sub = nh.subscribe("/space2tool_velocity_cmd",1000, VelocityToolsubCallback);

    tf::TransformListener listener;
    rotation_matrix=Eigen::Matrix3d::Identity();
    ros::Duration(1.0).sleep();

    ros::Rate rate(250.0);
    while (ros::ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("base", "tool0",ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        double x=transform.getRotation().getX();
        double y=transform.getRotation().getY();
        double z=transform.getRotation().getZ();
        double w=transform.getRotation().getW();
    
        rotation_matrix=getR(x,y,z,w);
        ros::spinOnce();
        rate.sleep();
        
    }
    return 0;
}
