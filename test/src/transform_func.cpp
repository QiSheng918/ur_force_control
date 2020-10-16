#include<iostream>
#include<vector>
#include<Eigen/Dense>
#include<Eigen/Core>

void normToRotationMatrix(Eigen::Vector3d &norm1, Eigen::Vector3d &norm2,Eigen::Matrix3d &R)
{
    Eigen::Vector3d temp=norm1.cross(norm2);
    double theta=acos(norm1.dot(norm2));
    double cos_theta=cos(theta);
    double sin_theta=sin(theta);
    double rx=temp(0);
    double ry=temp(1);
    double rz=temp(2);
    R(0,0)=rx*rx*(1-cos_theta)+cos_theta;
    R(0,1)=rx*ry*(1-cos_theta)-rz*sin_theta;
    R(0,2)=rx*rz*(1-cos_theta)+ry*sin_theta;
    R(1,0)=rx*ry*(1-cos_theta)+rz*sin_theta;
    R(1,1)=ry*ry*(1-cos_theta)+cos_theta;
    R(1,2)=ry*rz*(1-cos_theta)-rx*sin_theta;
    R(2,0)=rx*rz*(1-cos_theta)-ry*sin_theta;
    R(2,1)=ry*rz*(1-cos_theta)+rx*sin_theta;
    R(2,2)=rz*rz*(1-cos_theta)+cos_theta;
}

void rotationMatrixToEular(Eigen::Matrix3d &R,Eigen::Vector3d &angles){
    R.eulerAngles(2,1,0);
}