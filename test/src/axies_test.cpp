#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "Eigen/Core"
#include "cmath"
#include "Eigen/Dense"






int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  double input[3];
  double normal=0;
  for(int i=0;i<3;i++){
    std::cin>>input[i];
    normal+=input[i]*input[i];
  }
  normal=sqrt(normal);
  Eigen::AngleAxisd rotation_vector(normal,Eigen::Vector3d(input[0]/normal,input[1]/normal,input[2]/normal));
  std::cout<<rotation_vector.toRotationMatrix()<<std::endl;
  Eigen::Matrix3d toy_in_base;
  toy_in_base<<-1,0,0,
                0,1,0,
                0,0,-1;
  Eigen::Matrix3d result=toy_in_base*rotation_vector.toRotationMatrix().inverse();
  Eigen::AngleAxisd final_result;
  final_result.fromRotationMatrix(result);
  Eigen::Matrix3d rotation_matrix=result;
  double theta=acos((rotation_matrix(0,0)+rotation_matrix(1,1)+rotation_matrix(2,2)-1)/2);
  double rx=1/(2*sin(theta))*(rotation_matrix(2,1)-rotation_matrix(1,2));
  double ry=1/(2*sin(theta))*(rotation_matrix(0,2)-rotation_matrix(2,0));
  double rz=1/(2*sin(theta))*(rotation_matrix(1,0)-rotation_matrix(0,1));
  Eigen::Vector3d zhou=final_result.axis();
  std::cout<<final_result.angle()<<std::endl;
  std::cout<<zhou.x()*final_result.angle()<<"    "<<zhou.y()*final_result.angle()<<"    "<<zhou.z()*final_result.angle()<<std::endl;
  // std::cout<<rx<<"    "<<ry<<"   "<<rz<<std::endl;
      std::cout<<rx*theta<<","<<ry*theta<<","<<rz*theta<<std::endl;
  return 0;
};