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


class admittance
{
public:
    admittance()
    {
        zeros_one<<0,0,0,1;
        flag=true;
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
        T<<1,0,0,0,
           0,1,0,0,
           0,0,1,0,
           0,0,0,1;
        wrench_s<<0,0,0,0,0,0;
        vel.resize(6);

        for(int i = 0; i < 6; i ++)
        {   
            vel[i]=0;
            // std::cout<<vel[i]<<std::endl;
            p_gain(i,i) = 0.1;
        }
        std::cout<<std::endl;
        wrench_sub = nh.subscribe("/compensate_wrench", 1000, &admittance::WrenchsubCallback,this);
        joint_states_sub=nh.subscribe("/joint_states",1000,&admittance::JointStatesubCallback,this);
        ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);
        wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_s",1000);
        ros::Duration(1.0).sleep();
        while(flag);
        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            for(int i=0;i<3;i++){
                vel[i]=0.0075*this->wrench_s(i+3);
                // if(vel[i]<1e-3 && vel[i]>=-1e-3) vel[i]=0;
                if(vel[i]>1) vel[i]=1;
                else if(vel[i]<-1) vel[i]=-1;
                else;
            }
            for(int i=3;i<6;i++){
                vel[i]=0.5*this->wrench_s(i-3);
                // if(vel[i]<1e-3 && vel[i]>=-1e-3) vel[i]=0;
                if(vel[i]>0.75) vel[i]=0.75;
                else if(vel[i]<-0.75) vel[i]=-0.75;
                else;
            }
            // vel[1]=0.0075*this->wrench_s(4);
            // vel[2]=0.0075*this->wrench_s(5);
            // vel[0]=0.0075*this->wrench_s(3);
            // vel[1]=0.0075*this->wrench_s(4);
            // vel[2]=0.0075*this->wrench_s(5);
            // vel[3]=0.1*wrench_s(0);
            // vel[4]=0.1*wrench_s(1);
            // vel[5]=0.1*wrench_s(2);
            ur_move();
            for(int i=0;i<6;i++) std::cout<<vel[i]<<"###";
            // std::cout<<this->wrench_s<<std::endl;
            // std::cout<<this->wrench_s(2)<<std::endl;
    // std::cout<<"**"<<std::endl;
            std::cout<<std::endl;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub;
    ros::Subscriber joint_states_sub;
    ros::Publisher ur_pub;
    ros::Publisher wrench_pub;
    std::vector<double> vel;
    bool flag;
    
    Eigen::Matrix<double,6,1> wrench_s;
    Eigen::Matrix<double,6,6> p_gain;
    Eigen::Matrix<double,1,4> zeros_one;
    Eigen::Matrix<double,6,6> twist;
    Eigen::Matrix<double,4,4> H;
    Eigen::Matrix<double,4,4> T;
    Eigen::Matrix<double,6,6> J;

    // functions
    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void JointStatesubCallback(const sensor_msgs::JointState& msg);
    void ur_move();
    std::string double2string(double input);
    Eigen::Matrix<double,6,6> getAdT(Eigen::Matrix<double,4,4> T);
    void getSE3(std::vector<double> theta);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "admittance_control_of_ur");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    admittance ad;
    ros::waitForShutdown();
    return 0;
}

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



void admittance::getSE3(std::vector<double> theta){
    Eigen::Matrix<double,3,3> SO3;
    Eigen::Matrix<double,4,4> SE3[6];
    Eigen::Matrix<double,3,3> W;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix<double,3,3> I33;
    I33<<1,0,0,0,1,0,0,0,1;
    zeros_one<<0,0,0,1;
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
    Eigen::Matrix<double,4,4> I44;
    I44<<1,0,0,0,
         0,1,0,0,
  	     0,0,1,0,
	     0,0,0,1;
    Eigen::Matrix<double,6,1> temp;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++) temp(j,0)=twist(i,j);
        J.block(0,i,6,1)=getAdT(I44)*temp;
        I44=I44*SE3[i];
    }
    
    this->T=I44*H;
    this->J=J_;
}


void admittance::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    // ROS_INFO("hello");
    Eigen::Matrix<double,6,1> wrench;
    wrench(3) = msg.wrench.force.x;
    wrench(4) = msg.wrench.force.y;
    wrench(5) = msg.wrench.force.z;
    wrench(0) = msg.wrench.torque.x;
    wrench(1) = msg.wrench.torque.y;
    wrench(2) = msg.wrench.torque.z;
    Eigen::Matrix<double,3,3> R=T.block(0,0,3,3);
    Eigen::Matrix<double,6,6> AdT;
    Eigen::Matrix<double,3,3> zeros33;
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++)  zeros33(i,j)=0;
    }
    AdT.block(0,0,3,3)=R;
    AdT.block(3,3,3,3)=R;
    AdT.block(0,3,3,3)=zeros33;
    AdT.block(3,0,3,3)=zeros33;
    this->wrench_s=AdT*wrench;
    // return wrench_s;
    // wrench(0) = msg.wrench.force.x;
    // wrench(1) = msg.wrench.force.y;
    // wrench(2) = msg.wrench.force.z;
    // wrench(3) = msg.wrench.torque.x;
    // wrench(4) = msg.wrench.torque.y;
    // wrench(5) = msg.wrench.torque.z;
    // Eigen::Matrix<double,3,3> p33,zeros33;
    // p33<<0,-T(2,3),T(1,3),
    //    T(2,3), 0,-T(0,3),
    //    -T(1,3),T(0,3),0;
    // Eigen::Matrix<double,3,3> R=T.block(0,0,3,3);
    // Eigen::Matrix<double,6,6> AdT;
    // for(int i=0;i<3;i++){
    //     for(int j=0;j<3;j++)  zeros33(i,j)=0;
    // }
    // AdT.block(0,0,3,3)=R;
    // AdT.block(3,3,3,3)=R;
    // AdT.block(3,0,3,3)=p33*R;
    // AdT.block(0,3,3,3)=zeros33;
    
    // this->wrench_s=AdT*wrench;
    geometry_msgs::WrenchStamped pub_msg;
    pub_msg.wrench.force.x=this->wrench_s(0);
    pub_msg.wrench.force.y=this->wrench_s(1);
    pub_msg.wrench.force.z=this->wrench_s(2);
    pub_msg.wrench.torque.x=this->wrench_s(3);
    pub_msg.wrench.torque.y=this->wrench_s(4);
    pub_msg.wrench.torque.z=this->wrench_s(5);
    
    pub_msg.header.frame_id="base_link";
    pub_msg.header.stamp=ros::Time::now();
        // pub_msg.header
    wrench_pub.publish(pub_msg);
    // std::cout<<this->wrench_s<<std::endl;
    // std::cout<<"**"<<std::endl;
}

void admittance::JointStatesubCallback(const sensor_msgs::JointState& msg)
{
    
    std::vector<double> theta(6,0);
    for(int i=0;i<6;i++) theta[i]=msg.position[i];
    this->getSE3(theta);
    flag=false;
}


std::string admittance::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}

void admittance::ur_move()
{
    std_msgs::String ur_script_msgs;
    double time2move = 0.5;
    double acc=1;
    std::string move_msg;
    move_msg = "speedl([";
    move_msg = move_msg + double2string(vel[0]) + ",";
    move_msg = move_msg + double2string(vel[1]) + ",";
    move_msg = move_msg + double2string(vel[2]) + ",";
    move_msg = move_msg + double2string(vel[3]) + ",";
    move_msg = move_msg + double2string(vel[4]) + ",";
    move_msg = move_msg + double2string(vel[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(acc) + ",";
    move_msg = move_msg + double2string(time2move) + ")";
    move_msg = move_msg + "\n";
    ur_script_msgs.data=move_msg;
    ur_pub.publish(ur_script_msgs);

}

