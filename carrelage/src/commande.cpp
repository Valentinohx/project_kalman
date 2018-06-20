/*
	commande.cpp
	Créé le 2018-01-31
	BOUVIER-LAMBERT Hugo, PETESCH Chloé
	Publie une consigne de commande en boucle ouverte (en vitesse) pour le turtlebot
	Subscribe : aucun
	Publish : /cmd_vel_mux/input/teleop

	Publie une commande prédéfinie pour réaliser des tests avec le Turtlebot
*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

using namespace std;

//Publisher
ros::Publisher publisher_command;
geometry_msgs::Twist command_robot;

int main(int argc, char **argv)
{
	//ROS Initialization
	ros::init(argc, argv, "commande");
	ROS_INFO("Node commande_node connected to roscore");
	ros::NodeHandle nh;


	//Publishing  
	publisher_command = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
	ros::Rate loop_rate(10);	
	double t0 = ros::Time::now().toSec();
	double wait = 10;

	while (ros::ok())
	{
		
		ros::spinOnce();
		
		//On définit ici le temps de la trajectoire ainsi que les paramètres de vitesse linéaire et angulaire du robot.
		//C'est un robot (2,0) terrestre donc seules les vitesses linéaire selon x et angulaire selon z sont prises en compte
		//La trajectoire définie ici dure 70s, les 10 premières secondes le robot reste immobile, 
		//puis il avance en tournant à gauche pendant 8s, puis avance en ligne droite pour le temps restant
		if (ros::Time::now().toSec()-t0>wait && ros::Time::now().toSec()-t0<60+wait)
		{
			command_robot.linear.x = 0.1; 	//en m/s
			command_robot.linear.y = 0;
			command_robot.linear.z = 0;
			command_robot.angular.x = 0;
			command_robot.angular.y = 0;
			command_robot.angular.z = (ros::Time::now().toSec()-t0-wait < 8)? 2*3.1415/40 : 0; //2*3.1415/40;	//en rad/s
			publisher_command.publish(command_robot);

		}	


		loop_rate.sleep();
	}

	return 0;
}
