#include <iostream>
#include <ros/ros.h>
#include <string>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <autoware_msgs/Waypoint.h>

// A constant distance for opening ndt/gnss switch   
int switch_distance = 5;

std::string hole_topic_data;
std::string file_prefix_in;
std::string file_prefix_out;
std::string hole_in_name ;
std::string hole_out_name ;
bool vaild_hole_changed = false;

typedef struct point{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
}choosed_point_t,current_pose_t;

choosed_point_t choosed_in_point,choosed_out_point;
current_pose_t current_pose;


void parseWaypointForVer2(const std::string& line, autoware_msgs::Waypoint* wp)
{
  std::vector<std::string> columns;
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ',')){
    while (1){
        auto res = std::find(column.begin(), column.end(), ' ');
        if (res == column.end()){
            break;
        }
        column.erase(res);
    }

    if (!column.empty()){
      columns.emplace_back(column);
    }
  }
  wp->pose.pose.position.x = std::stof(columns[0]);
  wp->pose.pose.position.y = std::stof(columns[1]);
  wp->pose.pose.position.z = std::stof(columns[2]);
}

void loadWaypointsForVer2(std::string& filename, std::vector<autoware_msgs::Waypoint>* csv_wps){
  std::ifstream ifs(filename);

	if(!ifs){
		std::cout << "Error File " << std::endl;
		return;
	}else{
    std::cout << filename << std::endl;
  }

  std::string line;
  std::getline(ifs, line);  // Remove first line
  
  while (std::getline(ifs, line)){
    autoware_msgs::Waypoint wp;
    parseWaypointForVer2(line, &wp);
    csv_wps->emplace_back(wp);
  }
}

void holeCallback(const std_msgs::String::ConstPtr& msg){
    std::string tmp = msg->data.c_str();
    if(hole_topic_data != tmp && !vaild_hole_changed){
        hole_topic_data = tmp;
        hole_in_name = "";
        hole_in_name.append(file_prefix_in);
        hole_in_name.append(tmp);
        hole_in_name.append("_in.csv");

        hole_out_name = "";
        hole_out_name.append(file_prefix_out);
        hole_out_name.append(tmp);        
        hole_out_name.append("_out.csv");
        vaild_hole_changed = true;
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose){
    current_pose.x = pose->pose.position.x;
    current_pose.y = pose->pose.position.y;
    current_pose.z = pose->pose.position.z;
}

bool isInRange(const point& choosed_point,const point& current_pose){
    float distance = std::sqrt( std::abs(choosed_point.x - current_pose.x)+
                                std::abs(choosed_point.y - current_pose.y)+
                                std::abs(choosed_point.z - current_pose.z));
    std::cout << distance << std::endl;
    return distance <  switch_distance;
}

void set_hole_point(){
    // Load data by using csv
    std::vector<autoware_msgs::Waypoint> waypoints_in;
    std::vector<autoware_msgs::Waypoint> waypoints_out;
    loadWaypointsForVer2(hole_in_name, &waypoints_in);
    loadWaypointsForVer2(hole_out_name, &waypoints_out);
    
    //Set choosed_in_point and choosed_out_point
    choosed_in_point.x = waypoints_in.at(0).pose.pose.position.x;
    choosed_in_point.y = waypoints_in.at(0).pose.pose.position.y;
    choosed_in_point.z = waypoints_in.at(0).pose.pose.position.z;
    int i = waypoints_out.size()-1;
    choosed_out_point.x = waypoints_out.at(i).pose.pose.position.x;
    choosed_out_point.y = waypoints_out.at(i).pose.pose.position.y;
    choosed_out_point.z = waypoints_out.at(i).pose.pose.position.z;
}

int main(int argc,char **argv){
    ros::init(argc, argv ,"course_driving_detect");
    
    ros::NodeHandle nh;

    ros::Subscriber sub_hole = nh.subscribe("/destination_hole", 1000, holeCallback);

    ros::Subscriber sub_current_pose = nh.subscribe("/current_pose", 1000, poseCallback);

    nh.param<std::string>("file_prefix_in",file_prefix_in,"$(find course_driving_detect)/waypoints_in/" );
    nh.param<std::string>("file_prefix_out",file_prefix_out,"$(find course_driving_detect)/waypoints_out/" );

    ros::Rate loop_rate=10;

    while (ros::ok())
    {
        if(vaild_hole_changed){
            set_hole_point();
            std::cout <<"IN_POSE: " <<  choosed_in_point.x << " , " << choosed_in_point.y << " , " << choosed_in_point.z << std::endl;
            std::cout <<"OUT_POSE: " << choosed_out_point.x << " , " << choosed_out_point.y << " , " << choosed_out_point.z  << std::endl;
            nh.setParam("/ndt_reliability_detect/gps_switch", true);
            vaild_hole_changed = false;
        }

        if(isInRange(choosed_in_point,current_pose)){
            std::cout << " current_pose is in choosed_in_point range  " << std::endl;
            nh.setParam("/msf_localizer/ball_mode", true);
        }

        if(isInRange(choosed_out_point,current_pose)){
            std::cout << " current_pose is in choosed_out_point range  " << std::endl;
            nh.setParam("/msf_localizer/ball_mode", false);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}