#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/String.h"
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "std_msgs/Float64.h"
#include "string.h"
#include "sensor_msgs/JointState.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <ros/package.h>
#include <std_msgs/Bool.h>

static const std::string PLANNING_GROUP = "manipulator";


class URMoveInit
{
public:
    URMoveInit():move_group(PLANNING_GROUP)
    {
        

        pub = nh.advertise<std_msgs::Bool>("/ethdaq_zero", 1000);
       
        ros::Duration(3).sleep();
        move_group.setNamedTarget("compensate_pose");
        bool plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success){
            ROS_ERROR("plan failed");
            return;
        }
        std::cout<<"plan success"<<std::endl;
        bool execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success){
            ROS_ERROR("execute failed");
            return;
        }
        ros::Duration(3).sleep();
        std::cout<<"execute success:"<<std::endl;
        std_msgs::Bool msg;
        msg.data=true;
        pub.publish(msg);

        ros::shutdown();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan; 
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ur_move_init_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    URMoveInit ur_move;
    return 0;
}
