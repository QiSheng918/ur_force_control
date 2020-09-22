// use hybird position/force to let ur5 to move a circie; x,y direction is controlled by velocity
// z direction is controlled by pd control


#include "ros/ros.h"
#include "cmath"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <geometry_msgs/WrenchStamped.h>


double wz;
double fz;
const double desire_fz=8;
const double desire_wz=0;
const double radius=0.04;
const double angular_velocity=0.25;

std::string double2string(double input);
std::string combinemsg(std::vector<double> velocity, double acc = 1.2);

void WrenchsubCallback(const geometry_msgs::WrenchStamped& msg)
{
    // ROS_INFO("hello");
    fz=msg.wrench.force.z;
    wz=msg.wrench.torque.x;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hybrid_position_force");
    ros::NodeHandle nh;
    ros::Subscriber wrench_sub = nh.subscribe("/compensate_wrench_base", 1000, WrenchsubCallback);

    ros::Publisher ur_script_pub = nh.advertise<std_msgs::String>("ur_driver/URScript", 1000);
    ros::Duration(1.0).sleep();
    std_msgs::String ur_script_msgs;
    std::vector<double> joint_velocity = {0, 0, 0, 0, 0, 0};
    int flag=0;
    int loop_flag=0;
    int init_flag=0;
    ros::Rate loop_rate(50);
    double last_error;
    ros::Time start_time;
    while(ros::ok()){
        double wz_error=wz-desire_wz;
        
        double fz_error=fz-desire_fz;

        
        double f_vel=0.002*fz_error+0.000*(fz_error-last_error);
        double w_vel=0.2*wz_error+0.000*(fz_error-last_error);
        if(fabs(wz_error)<0.05 && fabs(fz_error)<0.05) flag++;
        ROS_INFO_STREAM("flag is: "<<flag);
        if(flag>100){
            ROS_INFO_ONCE("START X MOVE");
            if(init_flag==0){
                start_time=ros::Time::now();
                init_flag=1;
            }
            // double time_duration = (ros::Time::now() - start_time).toSec();
            // double x=-radius*angular_velocity*sin(angular_velocity*time_duration);
            // double y=radius*angular_velocity*cos(angular_velocity*time_duration);
            // joint_velocity[0]=x;
            // joint_velocity[1]=y;
            loop_flag=(loop_flag+1)%2000;
            // loop_flag++;
            if(loop_flag<1000) joint_velocity[1]=0.01;
            else joint_velocity[1]=-0.01;
            // else if(loop_flag>=200 && loop_flag<=400) joint_velocity[1]=0.02;
            // else joint_velocity[1]=0;
        }
        // joint_velocity[1]=0;
        last_error=fz_error;
        // vel=0;
        
        if(f_vel>0.5) f_vel=0.5;
        else if(f_vel<-0.5) f_vel=-0.5;

        if(w_vel>0.5) w_vel=0.5;
        else if(w_vel<-0.5) w_vel=-0.5;
        joint_velocity[3]=w_vel;
        joint_velocity[2]=f_vel;

        // std::cout<<vel<<"###"<<joint_velocity[0]<<"###"<<joint_velocity[1]<<std::endl;
        ur_script_msgs.data = combinemsg(joint_velocity);
        ur_script_pub.publish(ur_script_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::string move_msg="stopl(1)\n";
    ur_script_msgs.data = move_msg;
    ur_script_pub.publish(ur_script_msgs);
    return 0;
}

std::string double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}

std::string combinemsg(std::vector<double> velocity, double acc)
{
    double time2move = 0.5;
    std::string move_msg;
    move_msg = "speedl([";
    move_msg = move_msg + double2string(velocity[0]) + ",";
    move_msg = move_msg + double2string(velocity[1]) + ",";
    move_msg = move_msg + double2string(velocity[2]) + ",";
    move_msg = move_msg + double2string(velocity[3]) + ",";
    move_msg = move_msg + double2string(velocity[4]) + ",";
    move_msg = move_msg + double2string(velocity[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(0.5) + ",";
    move_msg = move_msg + double2string(time2move) + ")";
    move_msg = move_msg + "\n";
    return move_msg; 
}