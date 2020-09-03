#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "Eigen/Dense"
#include "string.h"
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

static const std::string PLANNING_GROUP = "manipulator";
const int pos_num=6;
class GravityCompensate
{
public:
    GravityCompensate():move_group(PLANNING_GROUP)
    {
        wrenchb_temp.resize(6);
        theta.resize(6);
        index=0;
        sensor_point=0;
        bool_flag=false;
        for(int i=0;i<6;i++){
            wrenchb_temp[i]=0;
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
        flag=false;
        wrench_sub = nh.subscribe("/wrench", 1000, &GravityCompensate::WrenchsubCallback,this);
        bool_flag_sub = nh.subscribe("/flag", 1000, &GravityCompensate::boolFlagsubCallback,this);
        joint_states_sub=nh.subscribe("/joint_states",1000,&GravityCompensate::JointStatesubCallback,this);
        ros::Duration(5).sleep();
        while(index<pos_num);
        // int exit_code=ur_move();
        // if(exit_code!=0){
        //     std::cout<<"move failed"<<std::endl;
        //     return;
        // }
        calculateP();
        calculateG();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber bool_flag_sub;
    moveit::planning_interface::MoveGroupInterface move_group;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    Eigen::Matrix<double,pos_num*3,1> F;
    Eigen::Matrix<double,pos_num*3,1> M;
    Eigen::Matrix<double,pos_num*3,3> R;
    std::vector<double> wrenchb_temp;
    std::vector<double> theta;
    Eigen::Matrix<double,6,6> twist;
    Eigen::Matrix<double,4,4> H;
    bool bool_flag;
    bool flag;
    int sensor_point; 
    int index;
    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void JointStatesubCallback(const sensor_msgs::JointState& msg);
    void boolFlagsubCallback(const std_msgs::Bool &msg);
    void getSE3();
    Eigen::Matrix<double,3,3> getAntisymmetric(Eigen::Matrix<double,3,1> v);
    void calculateP();
    void calculateG();
    int ur_move();
};



void GravityCompensate::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg){
    if(bool_flag){
        sensor_point++;
        wrenchb_temp[0] += msg.wrench.force.x;
        wrenchb_temp[1] += msg.wrench.force.y;
        wrenchb_temp[2] += msg.wrench.force.z;
        wrenchb_temp[3] += msg.wrench.torque.x;
        wrenchb_temp[4] +=msg.wrench.torque.y;
        wrenchb_temp[5] += msg.wrench.torque.z;
        if(sensor_point==100){
            // calculate 100 times force average date
            for(int i=0;i<100;i++){
                wrenchb_temp[i]/=100;
            }
            Eigen::Matrix<double,3,1> M_temp;
            Eigen::Matrix<double,3,1> F_temp;
            M_temp<<wrenchb_temp[3],wrenchb_temp[4],wrenchb_temp[5];
            F_temp<<wrenchb_temp[0],wrenchb_temp[1],wrenchb_temp[2];
            // F_temp<<0,wrenchb_temp[2],-wrenchb_temp[1],
            //         -wrenchb_temp[2],0,wrenchb_temp[0],
            //         wrenchb_temp[1],-wrenchb_temp[0],0;
            M.block(index*3,0,3,1)=M_temp;
            F.block(index*3,0,3,1)=F_temp;
            for(int i=0;i<100;i++){
                wrenchb_temp[i]=0;
            }
            getSE3();
            sensor_point=0;
            bool_flag=false;
            index++;     
        }
    }
}

void GravityCompensate::JointStatesubCallback(const sensor_msgs::JointState& msg)
{    
    for(int i=0;i<6;i++) theta[i]=msg.position[i];
}

void GravityCompensate::boolFlagsubCallback(const std_msgs::Bool& msg)
{    
    bool_flag=true;
}

void GravityCompensate::getSE3(){
    std::vector<double> theta_(theta);
    Eigen::Matrix<double,3,3> SO3;
    Eigen::Matrix<double,4,4> SE3[6];
    Eigen::Matrix<double,3,3> W;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix<double,3,3> I33;
    Eigen::Matrix<double,3,3> zeros_one;
    I33<<1,0,0,0,1,0,0,0,1;
    zeros_one<<0,0,0,1;
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
    R.block(3*index,0,3,3)=(I44*H).block(0,0,3,3).transpose();
}

Eigen::Matrix<double,3,3> GravityCompensate::getAntisymmetric(Eigen::Matrix<double,3,1> v){
    Eigen::Matrix<double,3,3> temp;
    temp<<0,v(2,0),-v(1,0),
          -v(2,0),0,v(0,0),
          v(1,0),-v(0,0),0;
    return temp;
}

void GravityCompensate::calculateP(){
    Eigen::Matrix<double,6,1> p;
    Eigen::Matrix<double,pos_num*3,6> E;
    Eigen::Matrix<double,3,3> I33;
    I33<<1,0,0,0,1,0,0,0,1;
    for(int i=0;i<pos_num;i++){
        E.block(i*3,0,3,3)=getAntisymmetric(F.block(index*3,0,3,1));
        E.block(i*3,3,3,3)=I33;
    }
    p=((E.transpose()*E).inverse())*E.transpose()*M;
    std::cout<<"the trans of gravity is"<<p<<std::endl;
}

void GravityCompensate::calculateG(){
    Eigen::Matrix<double,6,1> G;
    Eigen::Matrix<double,pos_num*3,6> E;
    Eigen::Matrix<double,3,3> I33;
    I33<<1,0,0,0,1,0,0,0,1;
    E.block(0,0,pos_num*3,3)=R;
    for(int i=0;i<6;i++) E.block(i*3,3,3,3)=I33;
    G=(E.transpose()*E).inverse()*E.transpose()*F;
    std::cout<<"the gravity and zero point of sensor is"<<G<<std::endl;
}

int GravityCompensate::ur_move()
{
    std::string pos="pos";
    for(int i=0;i<6;i++){
        move_group.setNamedTarget(pos+std::to_string(i));
        bool plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success) return -1;
        bool execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success) return -1;
        ros::Duration(5).sleep();
        flag=true;
        while(flag) continue;
        ros::Duration(5).sleep();
        index++;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GravityCompensate");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    GravityCompensate ad;
    // ros::waitForShutdown();
    return 0;
}



