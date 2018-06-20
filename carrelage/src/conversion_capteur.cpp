/*
	conversion_capteur.cpp
	Créé le 2018-01-31
	BOUVIER-LAMBERT Hugo, PETESCH Chloé
	Convertit l'information donnée par les capteurs en une donnée booléenne
	Subscribe : /mobile_base/sensors/core
	Publish : /capteur_carrelage_R, capteur_carrelage_L

	Traite les données brutes issues des capteurs de carrelage (applique un seuil), et publie dans deux topics séparés
*/

#include "ros/ros.h"
#include <kobuki_msgs/SensorState.h>
#include <std_msgs/Bool.h>

using namespace std;

//Publishers
ros::Publisher publisher_capteur_R;
ros::Publisher publisher_capteur_L;

//sensor limites;
const int sensor_threshold = 2000;

//Callback de mesure
void conversionCallback(kobuki_msgs::SensorState msg_state)
{
	//On se contente d'appliquer un seuil
    std_msgs::Bool etat_capteur;
	//capture right sensor data
	
    etat_capteur.data = msg_state.bottom[0] > sensor_threshold;
	publisher_capteur_R.publish(etat_capteur);

	//capture left sensor data
    etat_capteur.data = msg_state.bottom[2] > sensor_threshold;
	publisher_capteur_L.publish(etat_capteur);	
}

int main(int argc, char **argv)
{
	//ROS Initialization
	ros::init(argc, argv, "conversion_capteur");
	ROS_INFO("Node conversion_capteur_node connected to roscore");
	ros::NodeHandle nh;

	//Subscribing
	ROS_INFO("Subscribing to topics\n");
	ros::Subscriber subscriber_capteur = nh.subscribe<kobuki_msgs::SensorState>("/mobile_base/sensors/core" , 1, conversionCallback);

	//Publishing  
	publisher_capteur_R = nh.advertise<std_msgs::Bool>("/capteur_carrelage_R", 1);
	publisher_capteur_L = nh.advertise<std_msgs::Bool>("/capteur_carrelage_L", 1);
	
	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}
