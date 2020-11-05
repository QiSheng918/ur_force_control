#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"

ros::Publisher wrench_pub;



std::vector<std::vector<double> > wrench;
std::vector<double> wrench_sum;
const double z=0.087;

void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    geometry_msgs::WrenchStamped temp=msg;
    geometry_msgs::WrenchStamped pub_msg;

    pub_msg.wrench.force.x=temp.wrench.force.x;
    pub_msg.wrench.force.y=temp.wrench.force.y;
    pub_msg.wrench.force.z=temp.wrench.force.z;
    pub_msg.wrench.torque.x=temp.wrench.torque.x+temp.wrench.force.y*z;
    pub_msg.wrench.torque.y=temp.wrench.torque.y-temp.wrench.force.x*z;
    pub_msg.wrench.torque.z=temp.wrench.torque.z;
    pub_msg.header.frame_id="tool0";
    pub_msg.header.stamp=ros::Time::now();
    wrench_pub.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wrench2tool");
    ros::NodeHandle nh;

   

    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/ethdaq_tool", 1000);
    ros::Subscriber wrench_sub = nh.subscribe("/ethdaq_data", 1000, WrenchsubCallback);

    ros::spin();
    ros::waitForShutdown();
    return 0;
}
