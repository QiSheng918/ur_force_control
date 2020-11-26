#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>


ros::Publisher wrench_pub;
const double z=0.089;

void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    geometry_msgs::WrenchStamped pub_msg;

    pub_msg.wrench.force.x=msg.wrench.force.x;
    pub_msg.wrench.force.y=msg.wrench.force.y;
    pub_msg.wrench.force.z=msg.wrench.force.z;
    pub_msg.wrench.torque.x=msg.wrench.torque.x+msg.wrench.force.y*z;
    pub_msg.wrench.torque.y=msg.wrench.torque.y-msg.wrench.force.x*z;
    pub_msg.wrench.torque.z=msg.wrench.torque.z;
    pub_msg.header.frame_id="tool0";
    pub_msg.header.stamp=ros::Time::now();
    wrench_pub.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "contact_frame_wrench");
    ros::NodeHandle nh;

    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/compensate_wrench_contact", 1000);
    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench_tool", 1000, WrenchsubCallback);

    ros::spin();
    return 0;
}
