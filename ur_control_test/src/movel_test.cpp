#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Core>


std::vector<double> command_pose;
std::vector<double> initial_pose;


std::string double2string(double input);
std::string combinemsg(std::vector<double> velocity, double acc = 1);

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

const double radius=0.05;
const double angular_velocity=0.5;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "movel_test");
    ros::NodeHandle nh;
    ros::Publisher ur_script_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1000);
    
    tf::TransformListener listener;
    command_pose.resize(6);
    initial_pose.resize(6);
    for(int i=0;i<6;i++){
        initial_pose[i]=0;
        command_pose[i]=0;
    }

    ros::Duration(4).sleep(); 
    ros::Rate loop_rate(25); 
        std_msgs::String ur_script_msgs;
    for(int i=0;i<25;i++){
        tf::StampedTransform transform;
        try{
        listener.lookupTransform("base", "tool0",  
                                ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }

        initial_pose[0]=transform.getOrigin().getX();
        initial_pose[1]=transform.getOrigin().getY();
        initial_pose[2]=transform.getOrigin().getZ();
        double x=transform.getRotation().getX();
        double y=transform.getRotation().getY();
        double z=transform.getRotation().getZ();
        double w=transform.getRotation().getW();
        

        Eigen::Matrix3d rotation_matrix=getR(x,y,z,w);
        double theta=acos((rotation_matrix(0,0)+rotation_matrix(1,1)+rotation_matrix(2,2)-1)/2);
        double rx=1/(2*sin(theta))*(rotation_matrix(2,1)-rotation_matrix(1,2));
        double ry=1/(2*sin(theta))*(rotation_matrix(0,2)-rotation_matrix(2,0));
        double rz=1/(2*sin(theta))*(rotation_matrix(1,0)-rotation_matrix(0,1));
        
        initial_pose[3]=rx*theta;
        initial_pose[4]=ry*theta;
        initial_pose[5]=rz*theta;

        for(int i=0;i<6;i++) command_pose[i]=initial_pose[i];
        command_pose[2]=initial_pose[2];
        command_pose[2]+=0.01;
        for(int i=0;i<6;i++) std::cout<<command_pose[i]<<","; 
        std::cout<<std::endl;
    

        ur_script_msgs.data = combinemsg(command_pose,0.5);
        std::cout<<ur_script_msgs.data<<std::endl;
        ur_script_pub.publish(ur_script_msgs);
        loop_rate.sleep();
    }
    ros::Duration(1).sleep();
    std::string move_msg="stopl(1)\n";
    ur_script_msgs.data = move_msg;
    ur_script_pub.publish(ur_script_msgs);
    ROS_INFO("finished");
    return 0;
}

std::string double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}

std::string combinemsg(std::vector<double> command_pose, double acc)
{
    double time2move = 1;
    std::string move_msg;
    move_msg = "movel(p[";
    move_msg = move_msg + double2string(command_pose[0]) + ",";
    move_msg = move_msg + double2string(command_pose[1]) + ",";
    move_msg = move_msg + double2string(command_pose[2]) + ",";
    move_msg = move_msg + double2string(command_pose[3]) + ",";
    move_msg = move_msg + double2string(command_pose[4]) + ",";
    move_msg = move_msg + double2string(command_pose[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(1) + ",";
    move_msg = move_msg + double2string(0.4) + ",";
    move_msg = move_msg + double2string(1) + ")";

    move_msg = move_msg + "\n";
   
    return move_msg; 
}