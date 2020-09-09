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


const double veloity_limit=1;
const double linear_damp=0.0075;
const double angular_damp=0.5;

class AdmittanceControlDrag
{
public:
    AdmittanceControlDrag()
    {
        wrench_base.resize(6);
        command_vel.resize(6);

        for(int i = 0; i < 6; i ++)
        {   
            command_vel[i]=0;
            wrench_base[i]=0;
        }
      
        wrench_sub = nh.subscribe("/compensate_wrench_base_filter", 1000, &AdmittanceControlDrag::WrenchsubCallback,this);
        ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);

        ros::Duration(1.0).sleep();
        ROS_INFO("robot starts to move");

        ros::Rate loop_rate(25);
        while (ros::ok())
        {
            //导纳控制仅添加阻尼项进行拖动控制
            for(int i=0;i<3;i++){
                command_vel[i]=linear_damp*this->wrench_base[i];
            }
            for(int i=3;i<6;i++){
                command_vel[i]=angular_damp*this->wrench_base[i];
            }
            urMove();
            for(int i=0;i<6;i++) std::cout<<command_vel[i]<<"#   ";
            std::cout<<std::endl;
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber wrench_sub; 
    ros::Publisher ur_pub;

    std::vector<double> command_vel;
    std::vector<double> wrench_base;
    



    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void limitVelocity(std::vector<double> &velocity);
    void urMove();
    std::string double2string(double input);
};



//基坐标系力矩信息回调函数
void AdmittanceControlDrag::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    wrench_base[0] = msg.wrench.force.x;
    wrench_base[1] = msg.wrench.force.y;
    wrench_base[2] = msg.wrench.force.z;
    wrench_base[3] = msg.wrench.torque.x;
    wrench_base[4] = msg.wrench.torque.y;
    wrench_base[5] = msg.wrench.torque.z;
}


//浮点数转string
std::string AdmittanceControlDrag::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}


//限制速度大小
void AdmittanceControlDrag::limitVelocity(std::vector<double> &velocity){
    for(int i=0;i<velocity.size();i++){
        if(fabs(velocity[i])<1e-3) velocity[i]=0;
        if(velocity[i]>veloity_limit) velocity[i]=veloity_limit;
        else if(velocity[i]<-veloity_limit) velocity[i]=-veloity_limit;
        else ;
    }
}


//UR机器人回调函数
void AdmittanceControlDrag::urMove()
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
    ros::init(argc, argv, "AdmittanceControlDragUR");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    AdmittanceControlDrag adg;
    ros::waitForShutdown();
    return 0;
}