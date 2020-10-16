#include "ros/ros.h"
#include "cmath"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/transform_listener.h>


std::vector<double> current_pos;

std::string double2string(double input);
std::string combinemsg(std::vector<double> velocity, double acc = 1);

const double radius=0.05;
const double angular_velocity=0.5;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "speedl_test");
    ros::NodeHandle nh;

 
    current_pos.resize(6);
    ros::Publisher ur_script_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1000);
    tf::TransformListener listener;
    ros::Duration(1.0).sleep();
    std_msgs::String ur_script_msgs;
    std::vector<double> cart_velocity = {0, 0, 0, 0, 0, 0};
    
    
    ur_script_msgs.data = combinemsg(cart_velocity,1.5);
    ur_script_pub.publish(ur_script_msgs);
    ros::Rate loop_rate(50);
    
    
    Eigen::AngleAxisd rotationVector(theta,n);

    Eigen::Vector3d eulerAngle=rotationVector.matrix().eulerAngles(0,1,2);
	// std::cout<<eulerAngle(0)<<std::endl;
	std::cout<<eulerAngle(0)<<","<<eulerAngle(1)<<","<<eulerAngle(2)<<std::endl;
	std::cout<<n<<std::endl;

    while(ros::ok()){
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("base", "tool0",ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
      ros::Duration(1.0).sleep();
        joint_velocity[1]=0.1*sin(time_duration);
        // if(time_duration>5) ros::shutdown();
        ur_script_msgs.data = combinemsg(joint_velocity);
        ur_script_pub.publish(ur_script_msgs);
        ros::spinOnce();
        loop_rate.sleep();
    }
 
    std::string move_msg="stopl(1)\n";
    ur_script_msgs.data = move_msg;
    ur_script_pub.publish(ur_script_msgs);
    ROS_INFO("finished");
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
    double time2move = 1;
    std::string move_msg;
    move_msg = "speedl([";
    move_msg = move_msg + double2string(velocity[0]) + ",";
    move_msg = move_msg + double2string(velocity[1]) + ",";
    move_msg = move_msg + double2string(velocity[2]) + ",";
    move_msg = move_msg + double2string(velocity[3]) + ",";
    move_msg = move_msg + double2string(velocity[4]) + ",";
    move_msg = move_msg + double2string(velocity[5]) + "]";
    move_msg = move_msg + ",";
    move_msg = move_msg + double2string(1.5) + ",";
    move_msg = move_msg + double2string(time2move) + ")";
    move_msg = move_msg + "\n";
    return move_msg; 
}