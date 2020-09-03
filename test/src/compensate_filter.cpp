#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"

std::vector<double> wrench;
int flag=0;
ros::Publisher wrench_pub;
geometry_msgs::WrenchStamped pub_msg;
int frequency=5;
void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    
    wrench[0]+= msg.wrench.force.x;
    wrench[1]+= msg.wrench.force.y;
    wrench[2]+= msg.wrench.force.z;
    wrench[3]+= msg.wrench.torque.x;
    wrench[4]+= msg.wrench.torque.y;
    wrench[5]+= msg.wrench.torque.z;
    flag=(flag+1)%frequency;
    // std::cout<<flag<<std::endl;
    if(flag==(frequency-1)){
        for(int i=0;i<3;i++){
            if(abs(wrench[i])<1*frequency) wrench[i]=0;
        }
        for(int i=3;i<6;i++){
            if(abs(wrench[i])<0.1*frequency) wrench[i]=0;
        }
        pub_msg.wrench.force.x=wrench[0]/frequency;
        pub_msg.wrench.force.y=wrench[1]/frequency;
        pub_msg.wrench.force.z=wrench[2]/frequency;
        pub_msg.wrench.torque.x=wrench[3]/frequency;
        pub_msg.wrench.torque.y=wrench[4]/frequency;
        pub_msg.wrench.torque.z=wrench[5]/frequency;
        for(int i=0;i<6;i++) wrench[i]=0;
        pub_msg.header.frame_id="base_link";
        pub_msg.header.stamp=ros::Time::now();
        // pub_msg.header
        wrench_pub.publish(pub_msg);
        ROS_INFO("Hello");
    }
    
}
int main(int argc, char *argv[])
{
    wrench.resize(6);
    
    for(int i=0;i<6;i++) wrench[i]=0;
    ros::init(argc, argv, "compensate_filter");
    ros::NodeHandle nh;
    flag=0;
    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/compensate_filter", 1000);
    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench", 1000, WrenchsubCallback);
    ros::Duration(1.0).sleep();
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("1");
    ros::waitForShutdown();
    return 0;
}
