/*
    sensor_state_convert.cpp
    created at 2018-05-15
    Xiao SHI && Huaxin LIU
    objective: read the external sensor informations and transform it to binary

    Subscribe : /mobile_base/sensors/core
    Publish   : /capteur_carrelage
*/

#include "ros/ros.h"
#include <kobuki_msgs/SensorState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>

using namespace std;

//Publishers
ros::Publisher publisher_capteur;

//sensor limites;
const int sensor_threshold = 2000;

std_msgs::Int8MultiArray etat_capteur;


//Callback de mesure
void conversionCallback(kobuki_msgs::SensorState msg_state)
{
	//ROS_INFO("data begin");
	etat_capteur.data.clear();

    etat_capteur.data.push_back(msg_state.bottom[2] > sensor_threshold ? 1 : 0);   //left sensor
    etat_capteur.data.push_back(msg_state.bottom[0] > sensor_threshold ? 1 : 0);  //right sensor

	//ROS_INFO("data done");
    publisher_capteur.publish(etat_capteur);   //publish all the sensor state at onece
}

int main(int argc, char **argv)
{
    //ROS Initialization
    ros::init(argc, argv, "sensor_state_convert");
    ROS_INFO("Node conversion_capteur_node connected to roscore");
    ros::NodeHandle nh;
    //Subscribing
    ROS_INFO("Subscribing to topics\n");
    ros::Subscriber subscriber_capteur = nh.subscribe<kobuki_msgs::SensorState>("/mobile_base/sensors/core" , 1, conversionCallback);
	ROS_INFO("subscribe done");
    //Publishing
    publisher_capteur = nh.advertise< std_msgs::Int8MultiArray >("/capteur_carrelage", 1);
	ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
