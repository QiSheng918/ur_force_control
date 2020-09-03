#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "string.h"
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

static const std::string PLANNING_GROUP = "manipulator";
const int pos_num=6;
class GravityCompensate
{
public:
    GravityCompensate()
    {
        std::vector<double> temp;
        if(!nh.getParam("pos",temp)){
            ROS_FATAL_STREAM("pos Missing");
            return;
        }
        p<<temp[0],temp[1],temp[2];
        if(!nh.getParam("Gm",temp)){
            ROS_FATAL_STREAM("Gm Missing");
            return;
        }
        G0<<temp[0],temp[1],temp[2];
        if(!nh.getParam("F0",temp)){
            ROS_FATAL_STREAM("F0 Missing");
            return;
        }
        F0<<temp[0],temp[1],temp[2],temp[3],temp[4],temp[5];
        ROS_INFO_STREAM(F0);

        theta.resize(6);
        for(int i=0;i<6;i++){
            theta[i]=0;
        }
        twist<<0,0,1,0,0,0,
                0,-1,0,0.0892,0,0,
                0,-1,0,0.0892,0,0.425,
                0,-1,0,0.0892,0,0.8173,
                0,0,-1,0.1092,-0.8173,0,
                0,-1,0,-0.0055,0,0.8173;

        H<<1,0,0,-0.8173,
           0,0,-1,-0.1915,
           0,1,0,-0.0055,
           0,0,0,1;
       
        
        wrench_sub = nh.subscribe("/wrench", 1000, &GravityCompensate::WrenchsubCallback,this);
        joint_states_sub=nh.subscribe("/joint_states",1000,&GravityCompensate::JointStatesubCallback,this);
        wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("compensate_wrench",1000);
        ROS_INFO("init finished");
        ros::Duration(5).sleep();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub;
    ros::Subscriber joint_states_sub;
    ros::Publisher wrench_pub;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> G0;
    Eigen::Matrix<double,6,1> F0;



    std::vector<double> theta;
    Eigen::Matrix<double,6,6> twist;
    Eigen::Matrix<double,4,4> H;
    Eigen::Matrix<double,6,1> wrenchs;


    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void JointStatesubCallback(const sensor_msgs::JointState& msg);
    Eigen::Matrix<double,3,3> getSE3();

    Eigen::Matrix<double,3,3> getAntisymmetric(Eigen::Matrix<double,3,1> v);
};



void GravityCompensate::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg){
    Eigen::Matrix<double,6,1> wrenchb_temp;
    wrenchb_temp(0) = msg.wrench.force.x;
    wrenchb_temp(1) = msg.wrench.force.y;
    wrenchb_temp(2) = msg.wrench.force.z;
    wrenchb_temp(3) = msg.wrench.torque.x;
    wrenchb_temp(4) = msg.wrench.torque.y;
    wrenchb_temp(5) = msg.wrench.torque.z;
    // ROS_INFO("0");
    Eigen::Matrix<double,3,3> R=getSE3();
    // ROS_INFO("0.25");
    Eigen::Matrix<double,3,1> Gb=(R.transpose()*G0);
     
    Eigen::Matrix<double,6,1> GF;
    // ROS_INFO("1");
    GF<<Gb(0),Gb(1),Gb(2),
        Gb(2)*p(1)-Gb(1)*p(2),Gb(0)*p(2)-Gb(2)*p(0),Gb(1)*p(0)-Gb(0)*p(1);
    Eigen::Matrix<double,6,6> T;
    Eigen::Matrix<double,3,3> zeros33;
    zeros33<<0,0,0,0,0,0,0,0,0;
    // ROS_INFO("U are here");
    T.block(0,0,3,3)=R;
    T.block(3,3,3,3)=R;
    T.block(0,3,3,3)=zeros33;
    T.block(3,0,3,3)=zeros33;
    wrenchs=wrenchb_temp-GF-F0;
    // wrenchs=T*(wrenchb_temp-GF-F0);
    geometry_msgs::WrenchStamped pub_msg;
    pub_msg.header.stamp=ros::Time::now();
    pub_msg.header.frame_id="tool";
    pub_msg.wrench.force.x=wrenchs(0);
    pub_msg.wrench.force.y=wrenchs(1);
    pub_msg.wrench.force.z=wrenchs(2);
    pub_msg.wrench.torque.x=wrenchs(3);
    pub_msg.wrench.torque.y=wrenchs(4);
    pub_msg.wrench.torque.z=wrenchs(5);
    wrench_pub.publish(pub_msg);
 
}

void GravityCompensate::JointStatesubCallback(const sensor_msgs::JointState& msg)
{   
    // ROS_INFO("U are in jointstate callback");
    for(int i=0;i<6;i++) theta[i]=msg.position[i];
}

Eigen::Matrix<double,3,3> GravityCompensate::getSE3(){
    std::vector<double> theta_(theta);
    Eigen::Matrix<double,3,3> SO3;
    Eigen::Matrix<double,4,4> SE3[6];
    Eigen::Matrix<double,3,3> W;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix<double,3,3> I33;
    Eigen::Matrix<double,1,4> zeros_one;
    I33<<1,0,0,0,1,0,0,0,1;
    zeros_one<<0,0,0,1;
        // ROS_INFO("1.5");
    for(int i=0;i<6;i++){
        W<<0,-twist(i,2),twist(i,1),
            twist(i,2), 0,-twist(i,0),
            -twist(i,1),twist(i,0),0;
        v<<twist(i,3),twist(i,4),twist(i,5);
        SO3=I33+sin(theta_[i])*W+(1-cos(theta_[i]))*W*W;
        p=(theta_[i]*I33+(1-cos(theta_[i]))*W+(theta_[i]-sin(theta_[i]))*W*W)*v;
        SE3[i].block(0,0,3,3)=SO3;
        SE3[i].block(0,3,3,1)=p;
        SE3[i].block(3,0,1,4)=zeros_one;
    }
    // ROS_INFO("2");
    Eigen::Matrix<double,4,4> I44;
    I44<<1,0,0,0,
         0,1,0,0,
  	     0,0,1,0,
	     0,0,0,1;
    Eigen::Matrix<double,6,1> temp;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++) temp(j,0)=twist(i,j);
        I44=I44*SE3[i];
    }
    return (I44*H).block(0,0,3,3);
}

Eigen::Matrix<double,3,3> GravityCompensate::getAntisymmetric(Eigen::Matrix<double,3,1> v){
    Eigen::Matrix<double,3,3> temp;
    temp<<0,v(2,0),-v(1,0),
          -v(2,0),0,v(0,0),
          v(1,0),-v(0,0),0;
    return temp;
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GravityCompensate");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GravityCompensate ad;
    ros::waitForShutdown();
    return 0;
}



