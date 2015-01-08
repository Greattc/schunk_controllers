#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <schunk_controllers/send_trajectoriesConfig.h>
#include <fstream>
#include <cmath>
#include <ros/package.h>

#define NUMBER_OF_JOINTS 7 //<-- SCHUNK 7 JOINTS!

using namespace std;

//static const string name_space = "SCHUNK";
static const string joints_controller_names[7] = {
    "joint1_position_controller","joint2_position_controller","joint3_position_controller",
		"joint4_position_controller","joint5_position_controller","joint6_position_controller",
		"joint7_position_controller", };
static const string comand = "command";
static const string package="schunk_rrt";

double freq = 50.0;
bool start_trajectory = false;
int trajectory_lines = 0;
//We define all the messages needed for all the joints
vector<std_msgs::Float64> desired_joints_values;

void publication_rate_cb(schunk_controllers::send_trajectoriesConfig& config, uint32_t level)
{
    freq = config.publish_frequency;
    start_trajectory = config.execute_trajectory;
    if(config.reset_joints)
    {
        for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i)
            desired_joints_values[i].data = 0.0;
    }
    if(config.reset_trajectory)
        trajectory_lines = 0;
    ROS_INFO("Requested new publication rate %f. This will be applied to all joints!", freq);
}

void copy_trj_to_joints(const string& line, vector<std_msgs::Float64>& desired_joint_values)
{
    float fs[7];
    std::istringstream(line) >> fs[0] >> fs[1] >> fs[2] >> fs[3] >> fs[4] >> fs[5] >> fs[6];

//    for(unsigned int i = 0; i < 29; ++i)
//        cout<<fs[i]<<"  ";
//    cout<<endl;

    desired_joint_values[0].data = fs[0];
    desired_joint_values[1].data = fs[1];
    desired_joint_values[2].data = fs[2];
    desired_joint_values[3].data = fs[3];
    desired_joint_values[4].data = fs[4];
    desired_joint_values[5].data = fs[5];
    desired_joint_values[6].data = fs[6];

}

int main(int argc, char **argv)
{
    ROS_INFO("NODE TO SEND TRAJECTORIES...START!");

    //**** INITIALIZATION OF ROS, MSGS AND PUBLISHERS ****
    ros::init(argc, argv, "send_trajectories");

    ros::NodeHandle n;


    for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i){
        std_msgs::Float64 desired_joint_value;
        desired_joint_value.data = 0.0;
        desired_joints_values.push_back(desired_joint_value);
		}


    //We need 7 publishers also!
    vector<ros::Publisher> desired_joints_values_publishers;
    for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i){
        //string topic = name_space + "/" + joints_controller_names[i] + "/" + comand;
     		string topic = joints_controller_names[i] + "/" + comand;   
				ros::Publisher desired_joints_values_publisher = n.advertise<std_msgs::Float64>(topic, 1);
        desired_joints_values_publishers.push_back(desired_joints_values_publisher);}

    //Dynamic reconfigure stuffs
    dynamic_reconfigure::Server<schunk_controllers::send_trajectoriesConfig> server;
    dynamic_reconfigure::Server<schunk_controllers::send_trajectoriesConfig>::CallbackType f;

    f = boost::bind(&publication_rate_cb, _1, _2);
    server.setCallback(f);

    ROS_INFO("Reading trajectory file");
	string filepath = ros::package::getPath(package);
    
    ifstream trj_file((filepath+"/data/smooth_joints.dat").c_str());
    ROS_INFO("Successful!");		
    vector<string> lines;
    while(!trj_file.eof())
    {
        string line;
        getline(trj_file,line);
        lines.push_back(line);
    }
    ROS_INFO("Trajectory has %ld lines!", lines.size()-1);

    ros::Rate loop_rate(freq);
/*
		int count = 1;
		std_msgs::Float64 joint1_position;
*/

    while(ros::ok())
    {
        loop_rate = freq;
				//std::cout<<start_trajectory<<std::endl;
        if(start_trajectory)
        {
            if(trajectory_lines < lines.size()-1){
                copy_trj_to_joints(lines[trajectory_lines], desired_joints_values);
                trajectory_lines++;
						}
            else{
                //trajectory_lines = 0;
                start_trajectory = false;
            }
        

        for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i)
            desired_joints_values_publishers[i].publish(desired_joints_values[i]);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
