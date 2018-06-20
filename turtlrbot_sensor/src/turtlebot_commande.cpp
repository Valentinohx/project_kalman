/*
    turtlebot_commande.cpp
    created at 2018-05-15
    Xiao SHI && Huaxin LIU
    objective: provide two kind of control methods to control the robot to move
         1. use keyboard, four keys to control its linear speed with x direction
            and angular speed that rotation respect z axis
         2. set fixed linear speed and angular speed in the code, which is useful for
            specified trajectory like a line motion or circle trajectory
            the commanded speed is published to /cmd_vel_mux/input/teleop topic.

    Subscribe : /key_typed, in which the typed key ascill code is captured
    Publish   : /cmd_vel_mux/input/teleop, in which control the robot to move with specified speed

    subscribe the captured key and publish a predefined command to realise the tests with the turtlebot
*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <stdlib.h>
#include <vector>

//key defined
#define PLUS_X  (int(56)) //The ASCII code for the increment key -> key = 8
#define MINUS_X (int(50)) //The ASCII code for the decrement key -> key = 2
#define PLUS_ANGLE (int(52))  // key = 4, since anti-clockwise is the positive angular speed direction
#define MINUS_ANGLE (int(54)) // key = 6, since clockwise is the negative angular speed direction
#define STOP (int(53)) // key = 5


//step size defined
#define DEFAULT_INCR_X (double(0.01))      /*0.01 m/s */
#define DEFAULT_INCR_ANGLE (double(1*M_PI/180.0))  /* 0.5 degrees */

//speed limits defined
#define X_MAX_SPEED (double(1.0)) //1m/s maximum by default
#define ANGLE_MAX_SPEED (double(30*M_PI/180.0))  //maximum 3 degree/s by default

using namespace std;

//Publisher
ros::Publisher publisher_command;
geometry_msgs::Twist command_robot;
//geometry_msgs::Twist last_command_robot;
vector<double> last_command_robot = {0,0};

int x_incr_key, x_decr_key ;//The acii key codes to increment and decrement for x
int angle_incr_key, angle_decr_key; //The acii key codes to increment and decrement for angle
int stop_incr_key;
double x_incr ;   //The increment value
double angle_incr;

bool stop = false;
double t0;
double wait_time;
//set mode
int set_commande_mode = 0;  // 0 corresponds to manually mode while 1 corresponds to auto fixed mode

/* /////////////////////////////////////////////////////////////////////*/
void keyHitCallback(std_msgs::Int16 msg_key_hit)
{
    if(set_commande_mode == 0)   //if the command mode is set as an manually control mode, through keyboard
    {
        if(msg_key_hit.data == x_incr_key)
        {
            stop = false;
            if( last_command_robot[0] + x_incr < X_MAX_SPEED )
            {
                cout << "x Moving up!" << endl ;
                command_robot.linear.x =  last_command_robot[0] + x_incr; 	//en m/s
                ROS_INFO("current x speed: %f\n", command_robot.linear.x );

                //command_robot.linear.y = 0;
                //                command_robot.linear.z = 0;
                //                command_robot.angular.x = 0;
                //                command_robot.angular.y = 0;
                command_robot.angular.z = last_command_robot[1]; 	//en rad/s

                last_command_robot[0] = command_robot.linear.x;
                last_command_robot[1] = command_robot.angular.z;
                //pulish command
                //                publisher_command.publish(command_robot);
            }
            else
                ROS_INFO("I cannot speed up anymore, I am a turtle please!");
        }

        else if(msg_key_hit.data == x_decr_key)
        {
            stop = false;
            if ( last_command_robot[0] - x_incr >= -X_MAX_SPEED )
            {
                cout << "x Moving down!" << endl ;
                command_robot.linear.x =  last_command_robot[0] - x_incr; 	//in m/s
                ROS_INFO("current x speed: %f\n", command_robot.linear.x );
               // command_robot.linear.y = 0;
                //                command_robot.linear.z = 0;
                //                command_robot.angular.x = 0;
                //                command_robot.angular.y = 0;
                command_robot.angular.z = last_command_robot[1]; 	//in rad/s

                last_command_robot[0] = command_robot.linear.x;
                last_command_robot[1] = command_robot.angular.z;

                //pulish command
                //                publisher_command.publish(command_robot);
            }
            else
                ROS_INFO("speed has reached its maximum!");
        }
        else if(msg_key_hit.data == angle_incr_key)
        {
            stop = false;
            if (last_command_robot[1] + angle_incr < ANGLE_MAX_SPEED )
            {
                cout << "angle Moving up!" << endl ;
                command_robot.linear.x =  last_command_robot[0]; 	//in m/s

                //command_robot.linear.y =  
                //                command_robot.linear.z =  0;
                //                command_robot.angular.x = 0;
                //                command_robot.angular.y = 0;
                command_robot.angular.z = last_command_robot[1] + angle_incr; 	//in rad/s
                ROS_INFO("current angle speed: %f\n", command_robot.angular.z );

                last_command_robot[0] = command_robot.linear.x;
                last_command_robot[1] = command_robot.angular.z;

                //pulish command
                //                publisher_command.publish(command_robot);
            }
            else
            {
                ROS_INFO(" angular speed has reached its maximum! ");
            }
        }
        else if(msg_key_hit.data == angle_decr_key)
        {
            stop = false;
            if ( last_command_robot[1] - angle_incr > -ANGLE_MAX_SPEED )
            {
                cout << "angle Moving down!" << endl ;
                command_robot.linear.x =  last_command_robot[0]; 	//en m/s  //I shound recored the last x state, here is not zero but the lat state

                command_robot.angular.z = last_command_robot[1] - angle_incr ; 	//en rad/s

                last_command_robot[0] = command_robot.linear.x;
                last_command_robot[1] = command_robot.angular.z;

                //pulish command
                //                publisher_command.publish(command_robot);
            }
            else
            {
                ROS_INFO(" angular speed has reached its maximum! ");
            }
        }
        else if(msg_key_hit.data == stop_incr_key)
        {
            //stop = true;

                command_robot.linear.x =  0; 	//en m/s  //I shound recored the last x state, here is not zero but the lat state
                command_robot.angular.z = 0;
				last_command_robot = {0, 0};
				ROS_INFO(" robot stop! ");


        }
    }
    else   //if the control mode is set as fixed speed mode
    {

        if (ros::Time::now().toSec()-t0 > wait_time && ros::Time::now().toSec()-t0<60 + wait_time)
        {
            command_robot.linear.x = 0.2; 	//en m/s
            //command_robot.angular.z = (ros::Time::now().toSec() - t0 - wait_time < 8) ? 2*3.1415/40 : 0; //2*3.1415/40;	//en rad/s
			command_robot.angular.z = 0.2;
            //            publisher_command.publish(command_robot);
        }

    }
}
/* /////////////////////////////////////////////////////////////////////*/

int main(int argc, char **argv)
{
    //ROS Initialization
    ros::init(argc, argv, "commande");
    ROS_INFO("Node commande_node connected to roscore");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.

    nh_.param("x_incr_key", x_incr_key, PLUS_X);
    ROS_INFO("x_Increment key: %d\n",x_incr_key) ;

    nh_.param("x_decr_key", x_decr_key, MINUS_X);
    ROS_INFO("x_Decrement key: %d\n",x_decr_key) ;

    nh_.param("incr_key", angle_incr_key, PLUS_ANGLE);
    ROS_INFO("angle_Increment key: %d\n",angle_incr_key) ;

    nh_.param("decr_key", angle_decr_key, MINUS_ANGLE);
    ROS_INFO("angle_Decrement key: %d\n",angle_decr_key) ;

    nh_.param("x_incr", x_incr, DEFAULT_INCR_X);
    ROS_INFO("x increment: %f\n",x_incr) ;

    nh_.param("joint_incr", angle_incr, DEFAULT_INCR_ANGLE);
    ROS_INFO("angle increment: %f\n",angle_incr) ;

    nh_.param("stop_incr_key", stop_incr_key, STOP);
    ROS_INFO("stop",stop_incr_key);

	command_robot.linear.y = 0;
    command_robot.linear.z =  0;
    command_robot.angular.x = 0;
    command_robot.angular.y = 0;
    //subscribing
    ros::Subscriber key_hit = nh.subscribe<std_msgs::Int16> ("/key_typed"  , 1, keyHitCallback);

    //Publishing
    publisher_command = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    t0 = ros::Time::now().toSec();
    wait_time = 10;

    if (set_commande_mode == 0)
        ROS_INFO("current mode : manually mode  ");
    else
        ROS_INFO("current mode : fixed speed mode");

    while (ros::ok())
    {

            publisher_command.publish(command_robot);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
