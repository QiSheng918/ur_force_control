#include<ros/ros.h>
#include<yaml-cpp/yaml.h>
#include<iostream>
#include<fstream>
#include <ros/package.h>

using namespace std;
int main(){
    
    std::string dir_package, dir_package_file;
    dir_package = ros::package::getPath("test");
    dir_package_file = dir_package + "/config/param.yaml";
    std::ofstream fout(dir_package_file);
    std::cout<<dir_package_file<<std::endl;
    YAML::Emitter out(fout);
    out << YAML::BeginMap;
    
    out << YAML::Key << "int_param";

    out << YAML::BeginSeq;
    out << YAML::Value << 1;
    out << YAML::Value << 2;
    out << YAML::EndSeq;
    out << YAML::Key << "double_param";
    out << YAML::Value << 0.5;
    out << YAML::Key << "bool_param";
    out << YAML::Value << false;
    out << YAML::Comment("bool parameter");
    out << YAML::Key << "str_param";
    out << YAML::Value << "test";
    out << YAML::EndMap;
    fout.close();
    return 0;
}