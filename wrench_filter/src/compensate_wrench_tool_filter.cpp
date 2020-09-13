#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"

ros::Publisher wrench_pub;
geometry_msgs::WrenchStamped pub_msg;
const int frequency=3;
int flag=0;
std::vector<std::vector<double> > wrench;
std::vector<double> wrench_sum;


void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    wrench[0][flag]= msg.wrench.force.x;
    wrench[1][flag]= msg.wrench.force.y;
    wrench[2][flag]= msg.wrench.force.z;
    wrench[3][flag]= msg.wrench.torque.x;
    wrench[4][flag]= msg.wrench.torque.y;
    wrench[5][flag]= msg.wrench.torque.z;
    flag=(flag+1)%frequency;
    for(int i=0;i<6;i++){
        for(int j=0;j<frequency;j++){
            wrench_sum[i]+=wrench[i][j];
        }
    }
    
    for(int i=0;i<3;i++){
        if(abs(wrench_sum[i])<1.5*frequency) wrench_sum[i]=0;
    }
    for(int i=3;i<6;i++){
        if(abs(wrench_sum[i])<0.05*frequency) wrench_sum[i]=0;
    }

    pub_msg.wrench.force.x=wrench_sum[0]/frequency;
    pub_msg.wrench.force.y=wrench_sum[1]/frequency;
    pub_msg.wrench.force.z=wrench_sum[2]/frequency;
    pub_msg.wrench.torque.x=wrench_sum[3]/frequency;
    pub_msg.wrench.torque.y=wrench_sum[4]/frequency;
    pub_msg.wrench.torque.z=wrench_sum[5]/frequency;
    for(int i=0;i<6;i++) wrench_sum[i]=0;
    pub_msg.header.frame_id="tool0";
    pub_msg.header.stamp=ros::Time::now();
    wrench_pub.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "compensate_wrench_tool_filter");
    ros::NodeHandle nh;

    flag=0;
    std::vector<double> temp(frequency,0);
    for(int i=0;i<6;i++) wrench.push_back(temp);
    wrench_sum.resize(6);

    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/compensate_wrench_tool_filter", 1000);
    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench_tool", 1000, WrenchsubCallback);

    ros::Duration(1.0).sleep();
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
