#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TwistStamped.h>

// #define zeros_one (Eigen::Vector4d(0,0,0,1))


const Eigen::Matrix3d I33=Eigen::Matrix3d::Identity();
// const Eigen::Vector4d zeros_one(0,0,0,1);
const Eigen::Matrix3d Zeros33=Eigen::Matrix3d::Zero();
const double veloity_limit=1;


class admittance
{
public:
    admittance()
    {
        std::cout<<"hello world"<<std::endl;
        twist<<0,0,1,0,0,0,
                0,-1,0,0.0892,0,0,
                0,-1,0,0.0892,0,0.425,
                0,-1,0,0.0892,0,0.8173,
                0,0,-1,0.1092,-0.8173,0,
                0,-1,0,-0.0055,0,0.8173;

        H0<<1,0,0,-0.8173,
           0,0,-1,-0.1915,
           0,1,0,-0.0055,
           0,0,0,1;
        T<<1,0,0,0,
           0,1,0,0,
           0,0,1,0,
           0,0,0,1;
        zeros_one<<0,0,0,1;
        wrench_base.resize(6);
        command_vel.resize(6);
        actual_vel.resize(6);

        for(int i = 0; i < 6; i ++)
        {   
            command_vel[i]=0;
            wrench_base[i]=0;
            actual_vel[i]=0;       
        }
        //  std::cout<<"hello world"<<std::endl;
        std::cout<<std::endl;
        wrench_sub = nh.subscribe("/compensate_wrench_base_filter", 1000, &admittance::WrenchsubCallback,this);
        joint_states_sub=nh.subscribe("/joint_states",1000,&admittance::JointStatesubCallback,this);
        tool_velocity_sub=nh.subscribe("/tool_velocity",1000,&admittance::ToolVelocitysubCallback,this);
        ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);

        ros::Duration(1.0).sleep();
       
        ros::Rate loop_rate(25);
        ros::Time last_time=ros::Time::now();
        while (ros::ok())
        {
            
            for(int i=0;i<3;i++){
                double zdd=0.6*(this->wrench_base[i]-1*actual_vel[i]);
                // command_vel[i]+=0.04*zdd;
                 command_vel[i]=zdd*(ros::Time::now()-last_time).toSec();
                // command_vel[i]=0.0075*this->wrench_base[i];
            }
            // double zdd=50*(this->wrench_base[3]-0.1*actual_vel[3]);
            // command_vel[3]=zdd*(ros::Time::now()-last_time).toSec();
     
            for(int i=3;i<6;i++){
                double zdd=100*(this->wrench_base[i]-0.1*actual_vel[i]);
                // command_vel[i]=0.04*zdd;
                 command_vel[i]=zdd*(ros::Time::now()-last_time).toSec();
                // command_vel[i]=0.5*this->wrench_base[i];
            }
                   last_time=ros::Time::now();
            urMove();
            for(int i=0;i<6;i++) std::cout<<command_vel[i]<<"#   ";
            std::cout<<std::endl;
            for(int i=0;i<6;i++) std::cout<<wrench_base[i]<<"#   ";
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub;
    ros::Subscriber joint_states_sub;
    ros::Subscriber tool_velocity_sub;

    ros::Publisher ur_pub;
    ros::Publisher wrench_pub;
    std::vector<double> command_vel;
    std::vector<double> wrench_base;
    std::vector<double> actual_vel;
    
    Eigen::Matrix<double,6,6> twist;
    Eigen::Matrix<double,4,4> H0;
    Eigen::Matrix<double,4,4> T;
    Eigen::Matrix<double,6,6> J;

    Eigen::Matrix<double,1,4> zeros_one;

    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void JointStatesubCallback(const sensor_msgs::JointState& msg);
    void ToolVelocitysubCallback(const geometry_msgs::TwistStamped& msg);


    void limitVelocity(std::vector<double> &velocity);
    void urMove();

    std::string double2string(double input);

    Eigen::Matrix<double,6,6> getAdT(Eigen::Matrix4d T);
    void getSE3(std::vector<double> theta);
};

void admittance::ToolVelocitysubCallback(const geometry_msgs::TwistStamped& msg)
{
    actual_vel[0]=msg.twist.linear.x;
    actual_vel[1]=msg.twist.linear.y;
    actual_vel[2]=msg.twist.linear.z;
    actual_vel[3]=msg.twist.angular.x;
    actual_vel[4]=msg.twist.angular.y;
    actual_vel[5]=msg.twist.angular.z;
}



//计算工具坐标系相对于基座坐标系的齐次变换矩阵以及雅克比矩阵
void admittance::getSE3(std::vector<double> theta){

    Eigen::Matrix<double,3,3> SO3;
    Eigen::Matrix<double,4,4> SE3[6];
    Eigen::Matrix<double,3,3> W;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;

    for(int i=0;i<6;i++){
         
        W<<0,-twist(i,2),twist(i,1),
            twist(i,2), 0,-twist(i,0),
            -twist(i,1),twist(i,0),0;
        v<<twist(i,3),twist(i,4),twist(i,5);
       
        SO3=I33+sin(theta[i])*W+(1-cos(theta[i]))*W*W;
        p=(theta[i]*I33+(1-cos(theta[i]))*W+(theta[i]-sin(theta[i]))*W*W)*v;

        SE3[i].block(0,0,3,3)=SO3;
        SE3[i].block(0,3,3,1)=p;
        SE3[i].block(3,0,1,4)=zeros_one;
       
    }
    Eigen::Matrix<double,6,6> J_;
    Eigen::Matrix<double,4,4> I44=Eigen::Matrix4d::Identity();

    Eigen::Matrix<double,6,1> temp;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++) temp(j,0)=twist(i,j);
        J_.block(0,i,6,1)=getAdT(I44)*temp;
        I44=I44*SE3[i];
    }   
    this->T=I44*H0;
    this->J=J_;
}

//计算齐次变换矩阵矩阵对应的伴随矩阵
Eigen::Matrix<double,6,6> admittance::getAdT(Eigen::Matrix<double,4,4> T)
{

    Eigen::Matrix<double,3,3> p33,zeros33;
    p33<<0,-T(2,3),T(1,3),
       T(2,3), 0,-T(0,3),
       -T(1,3),T(0,3),0;
    Eigen::Matrix<double,3,3> R=T.block(0,0,3,3);
    Eigen::Matrix<double,6,6> AdT;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)  zeros33(i,j)=0;
    }
    AdT.block(0,0,3,3)=R;
    AdT.block(3,3,3,3)=R;
    AdT.block(0,3,3,3)=p33*R;
    AdT.block(3,0,3,3)=zeros33;
    return AdT;
}

//力矩传感器回调函数
void admittance::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
//    std::cout<<"hello world"<<std::endl;
    wrench_base[0] = msg.wrench.force.x;
    wrench_base[1] = msg.wrench.force.y;
    wrench_base[2] = msg.wrench.force.z;
    wrench_base[3] = msg.wrench.torque.x;
    wrench_base[4] = msg.wrench.torque.y;
    wrench_base[5] = msg.wrench.torque.z;
}


//机器日关节状态回调函数
void admittance::JointStatesubCallback(const sensor_msgs::JointState& msg)
{   
    
    std::vector<double> theta(6,0);
    for(int i=0;i<6;i++) theta[i]=msg.position[i];
   
    this->getSE3(theta);
}

//浮点数转string
std::string admittance::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}


//限制速度大小
void admittance::limitVelocity(std::vector<double> &velocity){
    for(int i=0;i<velocity.size();i++){
        if(fabs(velocity[i])<1e-3) velocity[i]=0;
        if(velocity[i]>veloity_limit) velocity[i]=veloity_limit;
        else if(velocity[i]<-veloity_limit) velocity[i]=-veloity_limit;
        else ;
    }
}


//UR机器人回调函数
void admittance::urMove()
{
    this->limitVelocity(command_vel);
    std_msgs::String ur_script_msgs;
    double time2move = 0.5;
    double acc=1;
    std::string move_msg;
    move_msg = "speedl([";
    move_msg = move_msg + double2string(command_vel[0]) + ",";
    move_msg = move_msg + double2string(command_vel[1]) + ",";
    move_msg = move_msg + double2string(command_vel[2]) + ",";
    move_msg = move_msg + double2string(command_vel[3]) + ",";
    move_msg = move_msg + double2string(command_vel[4]) + ",";
    move_msg = move_msg + double2string(command_vel[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(acc) + ",";
    move_msg = move_msg + double2string(time2move) + ")";
    move_msg = move_msg + "\n";
    ur_script_msgs.data=move_msg;
    ur_pub.publish(ur_script_msgs);
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "admittance_control_of_ur");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    admittance ad;
    ros::waitForShutdown();
    return 0;
}