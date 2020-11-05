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
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char *argv[])
{
    Eigen::Matrix<double,3,1> p;
    Eigen::Matrix<double,3,1> Gm;
    Eigen::Matrix<double,6,1> F0;
    ros::init(argc, argv, "GravityCompensate");
    ros::NodeHandle nh;
    std::vector<double> temp;
        if(!nh.getParam("pos",temp)){
            ROS_FATAL_STREAM("pos Missing");
            return -1;
        }
        p<<temp[0],temp[1],temp[2];
        if(!nh.getParam("Gm",temp)){
            ROS_FATAL_STREAM("Gm Missing");
            return -1;
        }
        Gm<<temp[0],temp[1],temp[2];
        if(!nh.getParam("F0",temp)){
            ROS_FATAL_STREAM("F0 Missing");
            return -1;
        }
        F0<<temp[0],temp[1],temp[2],temp[3],temp[4],temp[5];
    std::cout<<p<<std::endl;
    std::cout<<Gm<<std::endl;
    std::cout<<F0<<std::endl;
    return 0;
}



