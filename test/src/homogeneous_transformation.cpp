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
#include <tf/transform_listener.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>



class PoeTest
{
public:
    PoeTest()
    {
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

        H0<<1,0,0,-0.8173,
           0,0,-1,-0.1915,
           0,1,0,-0.0055,
           0,0,0,1;
       
        joint_states_sub=nh.subscribe("/joint_states",1000,&PoeTest::JointStatesubCallback,this);
        ros::Duration(5).sleep();
        ros::Rate loop_rate(20);
        tf::StampedTransform transform;
  
        while(ros::ok()){
            Eigen::Matrix4d T=getSE3();
            // std::cout<<getSE3()<<std::endl;
            try{
                listener.lookupTransform("base", "tool0",ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            double x=transform.getRotation().getX();
            double y=transform.getRotation().getY();
            double z=transform.getRotation().getZ();
            double w=transform.getRotation().getW();
            ROS_INFO_STREAM("THE ERROR OF TF AND POE IS: "<<T(0,3)-transform.getOrigin().getX());
            // std::cout<<quaternion2Rotation(x,y,z,w)<<std::endl;
            // std::cout<<transform.getOrigin().getX()<<","<<transform.getOrigin().getY()<<","<<transform.getOrigin().getZ()<<std::endl;
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber joint_states_sub;
    
    std::vector<double> theta;
    Eigen::Matrix<double,6,6> twist;
    Eigen::Matrix<double,4,4> H0;

    tf::TransformListener listener;

    void JointStatesubCallback(const sensor_msgs::JointState& msg);
    Eigen::Matrix4d getSE3();
    Eigen::Matrix<double,6,6> getAdT(Eigen::Matrix4d T);
    Eigen::Matrix3d getAntisymmetric(Eigen::Matrix<double,3,1> v);
    Eigen::Matrix3d quaternion2Rotation(double x,double y,double z,double w);
};

Eigen::Matrix<double,6,6> PoeTest::getAdT(Eigen::Matrix4d T)
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

void PoeTest::JointStatesubCallback(const sensor_msgs::JointState& msg)
{    
    for(int i=0;i<6;i++) theta[i]=msg.position[i];
}

Eigen::Matrix3d PoeTest::quaternion2Rotation(double x,double y,double z,double w){
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

Eigen::Matrix4d PoeTest::getSE3(){
    std::vector<double> theta_(theta);
    Eigen::Matrix3d SO3;
    Eigen::Matrix4d SE3[6];
    Eigen::Matrix3d W;
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> v;
    Eigen::Matrix3d I33=Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,1,4> zeros_one;
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
    Eigen::Matrix<double,4,4> I44=Eigen::Matrix<double,4,4>::Identity();
     Eigen::Matrix<double,6,6> J_;
    Eigen::Matrix<double,6,1> temp;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++) temp(j,0)=twist(i,j);
        J_.block(0,i,6,1)=getAdT(I44)*temp;
        I44=I44*SE3[i];
    }
    return I44*H0;  
}

Eigen::Matrix3d PoeTest::getAntisymmetric(Eigen::Matrix<double,3,1> v){
    Eigen::Matrix3d temp;
    temp<<0,v(2,0),-v(1,0),
          -v(2,0),0,v(0,0),
          v(1,0),-v(0,0),0;
    return temp;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "PoeTest");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    PoeTest ad;
    return 0;
}



