#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/utilities/error.h>
#include <kdl/utilities/utility.h>
#include <kdl/trajectory_composite.hpp>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

std::string double2string(double input);
std::string combinemsg(std::vector<double> velocity, double acc = 1);

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "intercept_test_node");

    ros::NodeHandle nh;
    tf::TransformListener listener;
    
    int flag = 0;
    ros::Publisher ur_script_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1000);
    KDL::Trajectory *traject;
    try
    {

        KDL::Path_RoundedComposite *path = new KDL::Path_RoundedComposite(0.05, 0.01, new KDL::RotationalInterpolation_SingleAxis());
        if (flag == 0)
        {
                        path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.192521,-0.348544,0.396922,0.826986), KDL::Vector(-0.23073, -0.0927758, 0.647443)));
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.322089,-0.60504,0.31081,0.65847), KDL::Vector(-0.372824, -0.25565, 0.552965)));
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(0.41756,0.796722,-0.181505,-0.397408), KDL::Vector(-0.373234, -0.255303, 0.648539)));

            path->Add(KDL::Frame(KDL::Rotation::Quaternion(0.453703,0.882523,-0.0415021,-0.116549), KDL::Vector(-0.373234, -0.255303, 0.848539)));







        }
        else
        {
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.541875, -0.480129, 0.460675, 0.513446), KDL::Vector(-0.372824, -0.25565, 0.552965)));
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.541533, -0.480408, 0.46102, 0.513236), KDL::Vector(-0.373234, -0.255303, 0.648539)));
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.541533, -0.480408, 0.46102, 0.513236), KDL::Vector(-0.370093, -0.0927312, 0.647424)));
            path->Add(KDL::Frame(KDL::Rotation::Quaternion(-0.541875, -0.480129, 0.460675, 0.513446), KDL::Vector(-0.23073, -0.0927758, 0.647443)));
        }
        // always call Finish() at the end, otherwise the last segment will not be added.
        path->Finish();

        KDL::VelocityProfile *velpref = new KDL::VelocityProfile_Trap(0.3, 0.1);
        velpref->SetProfile(0, path->PathLength());
        traject = new KDL::Trajectory_Segment(path, velpref);
    }
    catch (KDL::Error &error)
    {
        std::cout << "I encountered this error : " << error.Description() << std::endl;
        std::cout << "with the following type " << error.GetType() << std::endl;
        return 0;
    }

    std::vector<double> command_vel(6, 0);

    std_msgs::String ur_script_msgs;

    ros::Rate rate(25.0);
    // ros::Time init_time=ros::Time::now();
    ros::Time init_time = ros::Time::now();

    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("base", "tool0", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            init_time = ros::Time::now();
        }

        double pos_x = transform.getOrigin().getX();
        double pos_y = transform.getOrigin().getY();
        double pos_z = transform.getOrigin().getZ();

        double x=transform.getRotation().getX();
        double y=transform.getRotation().getY();
        double z=transform.getRotation().getZ();
        double w=transform.getRotation().getW();

        double t = (ros::Time::now() - init_time).toSec();
        std::vector<double> current_pose{pos_x, pos_y, pos_z};

        KDL::Frame target_pose = traject->Pos(t);
        KDL::Twist target_vel = traject->Vel(t);
        double xd,yd,zd,wd;
        target_pose.M.GetQuaternion(xd,yd,zd,wd);

		double q_dot=x*xd+y*yd+z*zd+w*wd;
        
        if(q_dot<0){
            x*=-1;
            y*=-1;
            z*=-1;
            w*=-1;
        }

        Eigen::Matrix<double,3,1> epsilon, epsilon_d;                 
        epsilon << x,y,z;        
        epsilon_d <<xd,yd,zd;    
        Eigen::Matrix<double,3,3> skew_matrix;
        skew_matrix << 0,-epsilon_d(2),epsilon_d(1),epsilon_d(2),0,-epsilon_d(0),-epsilon_d(1),epsilon_d(0),0;    
        Eigen::Matrix<double,3,1> orient_error = wd * epsilon - w * epsilon_d + skew_matrix * epsilon;   
        for (int i = 0; i < 3; i++)
            // command_vel[i] = target_vel.vel.data[i] + 0.5 * (target_pose(i, 3) - current_pose[i]);
            command_vel[i+3] = target_vel.rot.data[i]-1*orient_error[i];

        ur_script_msgs.data = combinemsg(command_vel);
        std::cout << ur_script_msgs.data << std::endl;
        ur_script_pub.publish(ur_script_msgs);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

std::string double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream << input;
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