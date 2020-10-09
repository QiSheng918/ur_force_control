// use hybird position/force to let ur5 to move a circie; x,y direction is controlled by velocity
// z direction is controlled by pd control


#include "ros/ros.h"
#include "cmath"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>


double fz;
const double desire_fz=-10;



void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    // ROS_INFO("hello");
    fz=msg.wrench.force.z;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hybrid_position_force");
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench_tool", 1000, WrenchsubCallback);

    ros::Publisher ur_script_pub = nh.advertise<std_msgs::Float64MultiArray>("/velocity_tool", 1000);
    ros::Duration(1.0).sleep();
    std_msgs::Float64MultiArray msg;
    msg.data.resize(6);
    for(int i=0;i<6;i++) msg.data[i]=0;
    
    ros::Rate loop_rate(50);
    double last_error;
  
    while(ros::ok()){
        double error=fz-desire_fz;
       
       
        
        double vel=0.001*error+0.0005*(error-last_error);
        last_error=error;
        // vel=0;
        
        if(vel>0.1) vel=0.1;
        else if(vel<-0.1) vel=-0.1;

        msg.data[2]=vel;
       
        ur_script_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
   
    return 0;
}

