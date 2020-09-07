#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "cmath"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

const double m=5;
const double b=5;
const double k=20;
const double desire_fz=10;

class FixedPointAdmittanceControl
{
public:
    FixedPointAdmittanceControl()
    {
        command_vel.resize(6);
        actual_vel.resize(6);
        wrench_base.resize(6);
        for(int i = 0; i < 6; i ++)
        {   
            command_vel[i]=0;
            actual_vel[i]=0;
            wrench_base[i]=0;
        }

        wrench_sub = nh.subscribe("/compensate_wrench_base", 1000, &FixedPointAdmittanceControl::WrenchsubCallback,this);
        tool_velocity_sub=nh.subscribe("/tool_velocity",1000,&FixedPointAdmittanceControl::ToolVelocitysubCallback,this);
        ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);
       
        ros::Duration(5.0).sleep();
        ros::Rate loop_rate(20);
        double delta_t=0.05;
        while (ros::ok())
        {
            double zdd=1/m*((this->wrench_base[2]-desire_fz)-b*(actual_vel[2]-0));
            command_vel[2]=zdd*delta_t;
            this->limitVelocity(command_vel);
            this->urMove();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber wrench_sub;
    ros::Subscriber tool_velocity_sub;
    ros::Publisher ur_pub;

    std::vector<double> command_vel;
    std::vector<double> actual_vel;
    std::vector<double> wrench_base;

    void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg);
    void ToolVelocitysubCallback(const geometry_msgs::TwistStamped& msg);

    void urMove();
    std::string double2string(double input);
    void limitVelocity(std::vector<double> &velocity);
};


// 限制发送速度指令大小
void FixedPointAdmittanceControl::limitVelocity(std::vector<double> &velocity){
    for(int i=0;i<velocity.size();i++){
        if(abs(velocity[i])<1e-4) velocity[i]=0;
        else if(velocity[i]>0.5) velocity[i]=0.5;
        else if(velocity[i]<-0.5) velocity[i]=-0.5;
        else ;
    }
}

//世界坐标系力矩传感器回调函数
void FixedPointAdmittanceControl::WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    wrench_base[0] = msg.wrench.force.x;
    wrench_base[1] = msg.wrench.force.y;
    wrench_base[2] = msg.wrench.force.z;
    wrench_base[3] = msg.wrench.torque.x;
    wrench_base[4] = msg.wrench.torque.y;
    wrench_base[5] = msg.wrench.torque.z;
}

//世界坐标系末端速度回调函数
void FixedPointAdmittanceControl::ToolVelocitysubCallback(const geometry_msgs::TwistStamped& msg)
{
    actual_vel[0]=msg.twist.linear.x;
    actual_vel[1]=msg.twist.linear.y;
    actual_vel[2]=msg.twist.linear.z;
    actual_vel[3]=msg.twist.angular.x;
    actual_vel[4]=msg.twist.angular.y;
    actual_vel[5]=msg.twist.angular.z;
}

//double转string
std::string FixedPointAdmittanceControl::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}


//UR机器人运动函数
void FixedPointAdmittanceControl::urMove()
{
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
    ros::init(argc, argv, "fixed_point_admittance_control");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    FixedPointAdmittanceControl ad;
    return 0;
}

