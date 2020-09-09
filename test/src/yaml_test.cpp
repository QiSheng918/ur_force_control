#include<ros/ros.h>
#include<yaml-cpp/yaml.h>
#include<iostream>
#include<fstream>
using namespace std;
int main(){
     std::ofstream fout("/home/leon/param.yaml");

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
}