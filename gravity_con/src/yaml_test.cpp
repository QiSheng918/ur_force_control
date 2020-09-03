#include<ros/ros.h>
#include<yaml-cpp/yaml.h>
#include<iostream>
#include<fstream>
using namespace std;
int main(){
    // std::string fin="/home/leon/param.yaml";
    // // std::string fin = "/home/user/param/param.yaml";       //yaml文件所在的路径
    // YAML::Node yamlConfig = YAML::LoadFile(fin);
    // int int_param = yamlConfig["int_param"].as<int>();
    // std::cout << "  node size: " << yamlConfig.size() << std::endl;
    // std::cout << yamlConfig["bool_param"].as<bool>() << "\n";
    // yamlConfig["bool_param"] = !yamlConfig["bool_param"].as<bool>();
    // yamlConfig["str_param"] = "test";
    // std::ofstream file;
    // file.open(fin);
    // file.flush();
    // file << yamlConfig;
    // file.close();
     std::ofstream fout("/home/leon/param.yaml");

     YAML::Emitter out(fout);
    out << YAML::BeginMap;
    
    out << YAML::Key << "int_param";
    out << YAML::BeginSeq;
    // out << YAML::BeginMap;
    out << YAML::Value << 1;
    out << YAML::Value << 2;
    out << YAML::EndSeq;
    // out << YAML::EndMap;
    out << YAML::Key << "double_param";
    out << YAML::Value << 0.5;
    out << YAML::Key << "bool_param";
    out << YAML::Value << false;
    out << YAML::Comment("bool parameter");
    out << YAML::Key << "str_param";
    out << YAML::Value << "test";
    out << YAML::EndMap;

    // YAML::Node config = YAML::LoadFile("../config.yaml");

    // cout << "name:" << config["name"].as<string>() << endl;
    // cout << "sex:" << config["sex"].as<string>() << endl;
    // cout << "age:" << config["age"].as<int>() << endl;
    // return 0;
 
//   std::string str;
//   std::ifstream fin("/home/leon/param.yaml");
//   YAML::Node doc = YAML::Load(fin);
//   doc["resolution"] >> "1";
//   doc["mytest"] = "g";
//   doc["resolutiond"] >> "1";
// //   ROS_INFO("resolution:%f,%d",res,doc.size());
// //   ROS_INFO("mytest:%s",str.c_str());
//   std::ofstream fout("/home/leon/param.yaml");
//   fout << doc;

//     return 0;
}