// 利用点云数据提取人体表面法向量并生成一条轨迹

#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <string>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>


const double velocity_limit=0.5;
const double desire_fz=-5;
static const std::string PLANNING_GROUP = "manipulator";


class SpeedCmdGenerator
{
public:
	SpeedCmdGenerator():move_group(PLANNING_GROUP)
	{
		flag=true;
        last_pos_flag=true;
		for(int i=0;i<6;i++){
			command_vel.push_back(0);
			wrench_base_now.push_back(0);
            wrench_tool_now.push_back(0);		
		}
		for(int i=0;i<7;i++){
            pos_now.push_back(0);
            last_pos.push_back(0);
        }



		ur_pub = nh.advertise<std_msgs::String>("ur_driver/URScript",1000);
		wrench_tool_sub = nh.subscribe("compensate_wrench_tool", 1000, &SpeedCmdGenerator::wrenchToolCallback,this);
        wrench_base_sub = nh.subscribe("compensate_wrench_base", 1000, &SpeedCmdGenerator::wrenchBaseCallback,this);

		pose_pub=nh.advertise<geometry_msgs::PoseArray>("pose_array",1000);
		ros::Duration(5).sleep();
		// while(flag) ;
		// ros::spin();
		this->posCmdGenerator();

		ros::Duration(5).sleep();
		std::string pos="vision_pose";
        move_group.setNamedTarget(pos);
        bool plan_success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!plan_success) return;
        std::cout<<"plan success"<<std::endl;
        bool execute_success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!execute_success) return;
        std::cout<<"execute success"<<std::endl;
	}


private:
	ros::NodeHandle nh;
	ros::Subscriber wrench_tool_sub,wrench_base_sub;
	ros::Publisher ur_pub,pose_pub;
	tf::TransformListener listener;

	std::vector<double> pos_now;
    std::vector<double> last_pos;
	Eigen::Matrix3d rotation_matrix;

	std::vector<double> wrench_base_now;
    std::vector<double> wrench_tool_now;


	std::vector<double> command_vel;
	bool flag,last_pos_flag;

	moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


	void wrenchToolCallback(const geometry_msgs::WrenchStampedConstPtr &msg);
    void wrenchBaseCallback(const geometry_msgs::WrenchStampedConstPtr &msg);
	void posCmdGenerator();
	void urMove();
	void limitVelocity(std::vector<double> &velocity);
	void getTransform();
    void display();
	std::string double2string(double input);
};


//力矩传感器TCP回调函数
void SpeedCmdGenerator::wrenchToolCallback(const geometry_msgs::WrenchStampedConstPtr& msg){
	wrench_tool_now[0] = msg->wrench.force.x;
    wrench_tool_now[1] = msg->wrench.force.y;
    wrench_tool_now[2] = msg->wrench.force.z;
    wrench_tool_now[3] = msg->wrench.torque.x;
    wrench_tool_now[4] = msg->wrench.torque.y;
    wrench_tool_now[5] = msg->wrench.torque.z;
};



//力矩传感器Base回调函数
void SpeedCmdGenerator::wrenchBaseCallback(const geometry_msgs::WrenchStampedConstPtr& msg){
	wrench_base_now[0] = msg->wrench.force.x;
    wrench_base_now[1] = msg->wrench.force.y;
    wrench_base_now[2] = msg->wrench.force.z;
    wrench_base_now[3] = msg->wrench.torque.x;
    wrench_base_now[4] = msg->wrench.torque.y;
    wrench_base_now[5] = msg->wrench.torque.z;
};

void SpeedCmdGenerator::display(){
    ros::Rate loop_rate(25);
    while(ros::ok()){
        this->getTransform();
        double delta_x[3];
        double temp_sum=0;
        for(int m=0;m<3;m++){
            delta_x[m]=pos_now[m]-last_pos[m];
            temp_sum+=delta_x[m]*delta_x[m];
        }
        temp_sum=sqrt(temp_sum);
        for(int m=0;m<3;m++){
            delta_x[m]/=temp_sum;
        }
        temp_sum=0;
        double normal[3];
        double temp=0;
        for(int m=0;m<3;m++) temp+=wrench_base_now[m]*delta_x[m];
        for(int m=0;m<3;m++){
            normal[m]=wrench_base_now[m]-temp*delta_x[m];
            // temp_sum+=normal[m];
        }

        Eigen::Vector3d norm1{1,0,0};
        
        Eigen::Vector3d norm2;
        // std::cout<<normal_vec[m][1]<<','<<normal_vec[m][2]<<","<<normal_vec[m][3]<<std::endl;
        norm2<<-normal[0],-normal[1],-normal[2];
        norm2.normalize();
        std::cout<<norm2<<std::endl;
        Eigen::Vector3d n=norm1.cross(norm2);
        // n.normalize();
        double theta=acos(norm1.dot(norm2));

        Eigen::AngleAxisd angle_axis(theta,n);
        Eigen::AngleAxisd angle_axis1(M_PI/2,Eigen::Vector3d(0,1,0));
        Eigen::Quaterniond q(angle_axis.toRotationMatrix()*angle_axis1.toRotationMatrix());
        double orientation_desire[4]={q.x(),q.y(),q.z(),q.w()};
        geometry_msgs::PoseArray pose_msg;
        pose_msg.header.stamp=ros::Time::now();
        pose_msg.header.frame_id="base";
        geometry_msgs::Pose temp_pose;
        temp_pose.position.x=pos_now[0];
        temp_pose.position.y=pos_now[1];
        temp_pose.position.z=pos_now[2];
        temp_pose.orientation.x=q.x();
        temp_pose.orientation.y=q.y();

        temp_pose.orientation.z=q.z();

        temp_pose.orientation.w=q.w();
        pose_msg.poses.push_back(temp_pose);
        pose_pub.publish(pose_msg);
        ros::spinOnce();
    }
}



void  SpeedCmdGenerator::posCmdGenerator()
{
    // display();
    ros::Rate loop_rate(25);
    for(int i=0;i<200;i++){
        this->getTransform();
        double delta_x[3];
        double temp_sum=0;
        for(int m=0;m<3;m++){
            delta_x[m]=pos_now[m]-last_pos[m];
            temp_sum+=delta_x[m]*delta_x[m];
        }
        temp_sum=sqrt(temp_sum);
        for(int m=0;m<3;m++){
            delta_x[m]/=temp_sum;
            std::cout<<delta_x[m]<<",";
        }
        std::cout<<std::endl;
        temp_sum=0;
        double normal[3];
        double temp=0;
        for(int m=0;m<3;m++) temp+=wrench_base_now[m]*delta_x[m];
        for(int m=0;m<3;m++){
            normal[m]=wrench_base_now[m]-temp*delta_x[m];
            // temp_sum+=normal[m];
        }

        Eigen::Vector3d norm1{1,0,0};
        
        Eigen::Vector3d norm2;
        // std::cout<<normal_vec[m][1]<<','<<normal_vec[m][2]<<","<<normal_vec[m][3]<<std::endl;
        norm2<<-normal[0],-normal[1],-normal[2];
        norm2.normalize();
        std::cout<<norm2<<std::endl;
        Eigen::Vector3d n=norm1.cross(norm2);
        // n.normalize();
        double theta=acos(norm1.dot(norm2));

        Eigen::AngleAxisd angle_axis(theta,n);
        Eigen::AngleAxisd angle_axis1(M_PI/2,Eigen::Vector3d(0,1,0));
        Eigen::Quaterniond q(angle_axis.toRotationMatrix()*angle_axis1.toRotationMatrix());
        double orientation_desire[4]={q.x(),q.y(),q.z(),q.w()};


        geometry_msgs::PoseArray pose_msg;
        pose_msg.header.stamp=ros::Time::now();
        pose_msg.header.frame_id="base";
        geometry_msgs::Pose temp_pose;
        temp_pose.position.x=pos_now[0];
        temp_pose.position.y=pos_now[1];
        temp_pose.position.z=pos_now[2];
        temp_pose.orientation.x=q.x();
        temp_pose.orientation.y=q.y();

        temp_pose.orientation.z=q.z();

        temp_pose.orientation.w=q.w();
        pose_msg.poses.push_back(temp_pose);
        pose_pub.publish(pose_msg);
        loop_rate.sleep();

        Eigen::Vector3d linear_speed,angular_speed;
        for(int m=0;m<3;m++) linear_speed(m)=0;
        linear_speed[0]=0.01;
        
        double q_dot=0;
        for(int m=3;m<7;m++) q_dot+=pos_now[m]*orientation_desire[m-3];
        if(q_dot<0){
            for(int m=3;m<7;m++) pos_now[m]=-pos_now[m];
        }
        Eigen::Matrix<double,3,1> epsilon, epsilon_d;                 
        epsilon << pos_now[3], pos_now[4], pos_now[5];          
        epsilon_d << orientation_desire[0], orientation_desire[1],orientation_desire[2];          
        Eigen::Matrix<double,3,3> skew_matrix;
        skew_matrix << 0,-epsilon_d(2),epsilon_d(1),epsilon_d(2),0,-epsilon_d(0),-epsilon_d(1),epsilon_d(0),0;    
        Eigen::Matrix<double,3,1> orient_error = orientation_desire[3] * epsilon - pos_now[6] * epsilon_d + skew_matrix * epsilon;                
        for(int m=0;m<3;m++) angular_speed(m)=  -0.5*orient_error(m); 

        std::cout<<wrench_tool_now[2]<<std::endl;

        linear_speed=rotation_matrix.transpose()*linear_speed;
        linear_speed(2)=-0.002*(desire_fz-wrench_tool_now[2]);
        linear_speed=rotation_matrix*linear_speed;
        for(int m=0;m<3;m++) command_vel[m]=linear_speed(m);
        for(int m=0;m<3;m++) command_vel[m+3]=angular_speed(m);
        for(int m=0;m<7;m++) last_pos[m]=pos_now[m];
        this->urMove();
        loop_rate.sleep();
        ros::spinOnce();
    }
}
   

//获取机器人当前状态信息
void SpeedCmdGenerator::getTransform(){
	tf::StampedTransform transform; 
	try{
		listener.lookupTransform("base", "contact_frame",ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	double _x=transform.getRotation().getX();
	double _y=transform.getRotation().getY();
	double _z=transform.getRotation().getZ();
	double _w=transform.getRotation().getW();
	Eigen::Quaterniond q(_w,_x,_y,_z);
	rotation_matrix=q.toRotationMatrix();
    
	pos_now[0]=transform.getOrigin().getX();
	pos_now[1]=transform.getOrigin().getY();
	pos_now[2]=transform.getOrigin().getZ();
    pos_now[3]=transform.getRotation().getX();
    pos_now[4]=transform.getRotation().getY();
    pos_now[5]=transform.getRotation().getZ();
    pos_now[6]=transform.getRotation().getW();
	for(int i=0;i<7;i++) std::cout<<pos_now[i]<<";";
	std::cout<<std::endl;
    if(last_pos_flag){
        for(int i=0;i<7;i++) last_pos[i]=pos_now[i];
        last_pos[0]-=0.01;
        last_pos_flag=false;
    }
}


//浮点数转string
std::string SpeedCmdGenerator::double2string(double input)
{
    std::string string_temp;
    std::stringstream stream;
    stream<<input;
    string_temp = stream.str();
    return string_temp;
}


//限制速度大小
void SpeedCmdGenerator::limitVelocity(std::vector<double> &velocity){
    for(int i=0;i<velocity.size();i++){
        if(fabs(velocity[i])<1e-4) velocity[i]=0;
        if(velocity[i]>velocity_limit) velocity[i]=velocity_limit;
        else if(velocity[i]<-velocity_limit) velocity[i]=-velocity_limit;
        else ;
    }
}


//UR机器人回调函数
void SpeedCmdGenerator::urMove()
{
    this->limitVelocity(command_vel);
    std_msgs::String ur_script_msgs;
    double time2move = 0.2;
    double acc=0.5;
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

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "speed_cmd_generator_node");
	ros::AsyncSpinner spinner(2);
    spinner.start();
	SpeedCmdGenerator speed_cmd_generator;
	return 0;
}
