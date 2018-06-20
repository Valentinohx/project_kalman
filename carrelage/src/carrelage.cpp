/*
	carrelage.cpp
	Créé le 2018-01-31
	BOUVIER-LAMBERT Hugo, PETESCH Chloé
	Node principal. Réalise l'odométrie et implémente le filtre de Kalman étendu.
	Subscribe : /mobile_base/sensors/core, /capteur_carrelage_R, /capteur_carrelage_L
	Publish (utilisés uniquement pour l'analyse des résultats) : /odom_carrelage, /variance_carrelage, /position_capteur

	Récupère la donné brute des encodeurs des roues, la convertit en distance parcourue (entrée U).
	Estime l'état à partir du modèle d'évolution et des mesures issues des deux capteurs de joint de carrelage placés sous le robot.
	Implémente le filtre de Kalman utilisé pour la prise en compte des mesures.
*/

#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32MultiArray.h>
#include <kobuki_msgs/SensorState.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>				//Bibliothèque de calcul matriciel

using namespace std;
using namespace Eigen;

const double PI = 3.14159265359;

//----------------------------------------- VARIABLES GLOBALES ----------------------------------------------//
ros::Publisher publisher_state;
ros::Publisher publisher_variance;
ros::Publisher publisher_capteur;


//Variables principales
vector<VectorXd> mesureR;			//mesures estimees des capteurs, sensor measurement
vector<VectorXd> mesureL;			//mesures estimees des capteurs
VectorXd X;							//état  , with dynamic size, type is double, column vector
VectorXd U;							//déplacement sous les roues
VectorXi Encoders;					//encodage des positions des roues, interger with dynamic size

//Matrices de Kalman
MatrixXd A;							//matrice A de Kalman, with double type, and dynamic matrix
MatrixXd B;							//matrice B de Kalman
MatrixXd C_x;						//matrice C de Kalman du X
MatrixXd C_y;						//matrice C de Kalman du Y
MatrixXd P;							//propagation d'erreur
MatrixXd K;

//Matrices de variances
MatrixXd Qalpha;
MatrixXd Qbeta;
double Qgamma;

//Seuil de Mahalanobis
double mahaThreshold = 2.7;

//Variables géométriques et temporelles
double T = 0.02;							//période de mise à jour de l'odométrie
double t0;									//temps système du début du programme
double encoder_coef = 2578.33;				//facteur de conversion donnée codeur -> angle (en incrément/tour de roue)
double lr = 0.227*1.103;					//longueur entre les roues, track gauch
double rr = 0.035;							//rayon des roues, the radius of the wheel
double dC = 0.075;							//distance centre-capteur selon y
double t_carreau = 0.30;					//taille des carreaux (bat. D : 0.1017, bat. E : 0.30)


//---------------------------------------- MODELE D'EVOLUTION ----------------------------------------//
void evolutionModel()
{
    //On sauvegarde le dernier état pris en compte pour chaque codeur
	// de manière a obtenir une variation d'angle sans rater d'incrément

    //store the last state of the encoder to get the change of angle, and avoid to miss every increment
    static int left_encoder_prec;    //file scope: entire file, and it remains in memory until the program ends
	static int right_encoder_prec;
	
	//Les premières valeurs des codeurs étaient aberrantes, on ne les prend donc pas en compte
    //the first value of the encoder is abnormal, so we do not take it consideration

	//Cette modification impose un temps d'arrêt au début de la tajectoire du robot dans la commande
    //This modification imposes a pause at the beginning of the robot's tajectory in the command
	if(ros::Time::now().toSec()-t0>1.5)
	{
        //Variables de variation d'incrément codeur
        //Encoder increment variation variables
		int delta_left = Encoders[0]-left_encoder_prec;
		int delta_right = Encoders[1]-right_encoder_prec;

		//On prend en compte la limite des codeurs (2^16) au bout de laquelle ils bouclent à 0
        //We take into account the limit of the coders (2^16) at the end of which they loop to 0
		if (delta_left > 1<<15)
			delta_left += -1<<16;
		else if (delta_left < -1<<15)
			delta_left += 1<<16;

		if (delta_right > 1<<15)
			delta_right += -1<<16;
		else if (delta_right < -1<<15)
			delta_right += 1<<16;
		
		//Conversion en distance parcourue sous chaque roue
        //Conversion to traveled distance under each wheel
        U[0] = delta_left*rr*2*PI/encoder_coef;   //travel distance = (increment/total)*wheel_length
		U[1] = delta_right*rr*2*PI/encoder_coef;

		double dx = (U[0]+U[1])/2;
		double dtheta = (-U[0]+U[1])/lr;

		//Modèle d'évolution
		X[0] = X[0]+dx*cos(X[2]);
		X[1] = X[1]+dx*sin(X[2]);
		X[2] = X[2]+dtheta;
	}
	
	//Mise à jour des mémoires encodeurs
	left_encoder_prec = Encoders[0];
	right_encoder_prec = Encoders[1];
}

//-------------------------------------- CALLBACKS MESURE -------------------------------------------//
// Mesure issue du capteur droit
void mesureRCallback(std_msgs::Bool etat_capteur)
{
	//Test s'il s'agit d'un joint de carrelage
	if ( ! etat_capteur.data) return;

	VectorXd C(2);				//position estimée du capteur droit
	C[0] = X[0] + dC*sin(X[2]);
	C[1] = X[1] - dC*cos(X[2]);
	
	mesureR.push_back(C);
}

// Mesure issue du capteur gauche
void mesureLCallback(std_msgs::Bool etat_capteur)
{
	//Test s'il s'agit d'un joint de carrelage
	if ( ! etat_capteur.data) return;

	VectorXd C(2);				//position estimée du capteur gauche
	C[0] = X[0] - dC*sin(X[2]);
	C[1] = X[1] + dC*cos(X[2]);
	
	mesureL.push_back(C);
}

// Mise à jour les données brutes des encodeurs
void vitesseCallback(kobuki_msgs::SensorState etat_capteur)
{
	Encoders[0] = etat_capteur.left_encoder;
	Encoders[1] = etat_capteur.right_encoder;
}


//-------------------------------------- FONCTION MAIN ------------------------------------------//
int main(int argc, char **argv)
{
	//Initialisation des variables
	X = VectorXd::Zero(3);
	U = VectorXd(2);
	Encoders = VectorXi(2);
	A = MatrixXd(3,3);
	B = MatrixXd(3,2);
	C_x = MatrixXd(1,3);
	C_y = MatrixXd(1,3);
	P = MatrixXd(3,3);
	mesureR = vector<VectorXd>();
	mesureL = vector<VectorXd>();
	
	//Pose de départ
	X << 0, 0, 0;

	//Incertitude sur la position initiale
	P << pow(0.007,2), 0, 0,
		 0, pow(0.007,2), 0,
		 0, 0, pow(5*PI/180,2);

	//Matrices de variance
	Qalpha = MatrixXd(3,3);				//bruit sur l'état
	Qalpha << pow(0.003,2)*T, 0, 0,
			  0, pow(0.003,2)*T, 0,
			  0, 0, pow(0.2*PI/180,2)*T;
	Qbeta = MatrixXd(2,2);				//bruit sur l'entrée
	Qbeta << pow(0.00006,2), 0,
			 0, pow(0.00006,2);
	Qgamma = pow(0.005,2)/12;			//bruit sur la mesure

	//Initialisation ROS
	ros::init(argc, argv, "conversion_capteur");
	ROS_INFO("Node conversion_capteur_node connected to roscore");
	ros::NodeHandle nh;

	//Subscribing
	ROS_INFO("Subscribing to topics\n");
	ros::Subscriber subscriber_capteur_R = nh.subscribe<std_msgs::Bool>("/capteur_carrelage_R", 1, mesureRCallback);
	ros::Subscriber subscriber_capteur_L = nh.subscribe<std_msgs::Bool>("/capteur_carrelage_L", 1, mesureLCallback);
	ros::Subscriber subscriber_roue = nh.subscribe<kobuki_msgs::SensorState>("/mobile_base/sensors/core" , 1, vitesseCallback);

	//Publishing  
	publisher_state = nh.advertise<geometry_msgs::Pose2D>("/odom_carrelage", 1);
	publisher_variance = nh.advertise<geometry_msgs::Pose2D>("/variance_carrelage", 1);
	publisher_capteur = nh.advertise<geometry_msgs::Pose2D>("/position_capteur", 3);
	
	//Fréquence de rafraîchissement
	ros::Rate loop_rate(1/T);

	//Temps initial
	t0 = ros::Time::now().toSec();

	while (ros::ok())
	{
		ros::spinOnce();

		//Odométrie
		evolutionModel();

		//Calcul des matrices Jacobiennes du système
		A << 1, 0, -((U[0]+U[1])/2)*sin(X[2]),
			 0, 1, ((U[0]+U[1])/2)*cos(X[2]),
			 0, 0, 1;
		B << cos(X[2])/2, cos(X[2])/2,
			 sin(X[2])/2, sin(X[2])/2,
			 -1.0/lr, 1.0/lr;

//---------------------------------------------- FILTRE DE KALMAN ------------------------------------------//
		//Propagation d'erreur
		P = A*P*A.transpose()+B*Qbeta*B.transpose()+Qalpha;
		
		/* Lorsqu'une mesure est prise par l'un des capteurs, on calcule deux distances de Mahalanobis : 
		 * une en considérant que la ligne détectée est horizontale, l'autre verticale
		 * si l'une seulement est sous le seuil, elle est retenue, on utilise la matrice C correspondante
		 */
		//Prise en compte des mesures du capteur droit
		for (int i=0;i<mesureR.size();i++)
		{
			//Mesure
			VectorXd C_round(2);
			C_round[0] = round(mesureR[i][0]/t_carreau)*t_carreau;		//position du capteur arrondie à un carreau selon x
			C_round[1] = round(mesureR[i][1]/t_carreau)*t_carreau;		//position du capteur arrondie à un carreau selon y

			//Matrices de Kalman pour des lignes horizontales et verticales
			C_x << 1, 0, dC*cos(X[2]);
			C_y << 0, 1, dC*sin(X[2]);
			
			//Calculs des distances de Mahalanobis
			double delta_mesure_X = C_round[0] - mesureR[i][0];
			double dMaha_X = abs(delta_mesure_X)/sqrt((C_x*P*C_x.transpose())(0,0)+Qgamma);

			double delta_mesure_Y = C_round[1] - mesureR[i][1];
			double dMaha_Y = abs(delta_mesure_Y)/sqrt((C_y*P*C_y.transpose())(0,0)+Qgamma);

			//Publication des positions estimées du capteur (pour l'analyse de résultats)
			//Le champ theta renseigne sur la distance de Mahalanobis calculée pour les deux types de ligne
			geometry_msgs::Pose2D capteur;			
			capteur.x = mesureR[i][0];
			capteur.y = mesureR[i][1];
			capteur.theta = dMaha_X;
			publisher_capteur.publish(capteur);

			capteur.x = mesureR[i][0];
			capteur.y = mesureR[i][1];
			capteur.theta = 1000+dMaha_Y;
			publisher_capteur.publish(capteur);

			//Kalman du X
			if (dMaha_X < mahaThreshold && dMaha_Y > mahaThreshold)
			{

				K = P*C_x.transpose()/((C_x*P*C_x.transpose())(0,0)+Qgamma);
				//Mise à jour de l'état estimé
				X = X + K*delta_mesure_X;
				//Mise à jour de P
				P = (MatrixXd::Identity(3,3)-K*C_x)*P;
			}
			
			//Kalman du Y
			else if (dMaha_Y < mahaThreshold && dMaha_X > mahaThreshold)
			{
				K = P*C_y.transpose()/((C_y*P*C_y.transpose())(0,0)+Qgamma);
				//Mise à jour de l'état estimé
				X = X + K*delta_mesure_Y;
				//Mise à jour de P
				P = (MatrixXd::Identity(3,3)-K*C_y)*P;
			}
		}
		mesureR.clear();		//penser à vider le vecteur de mesure
		




		//Prise en compte des mesures du capteur gauche
		for (int i=0;i<mesureL.size();i++)
		{
			//Mesure
			VectorXd C_round(2);
			C_round[0] = round(mesureL[i][0]/t_carreau)*t_carreau;		//position du capteur arrondie à un carreau selon x
			C_round[1] = round(mesureL[i][1]/t_carreau)*t_carreau;		//position du capteur arrondie à un carreau selon y

			//Matrices de Kalman pour des lignes horizontales et verticales
			C_x << 1, 0, -dC*cos(X[2]);
			C_y << 0, 1, -dC*sin(X[2]);

			//Calculs des distances de Mahalanobis
			double delta_mesure_X = C_round[0] - mesureL[i][0];
			double dMaha_X = abs(delta_mesure_X)/sqrt((C_x*P*C_x.transpose())(0,0)+Qgamma);

			double delta_mesure_Y = C_round[1] - mesureL[i][1];
			double dMaha_Y = abs(delta_mesure_Y)/sqrt((C_y*P*C_y.transpose())(0,0)+Qgamma);

			//Publication des positions estimées du capteur (pour l'analyse de résultats)
			//Le champ theta renseigne sur la distance de Mahalanobis calculée pour les deux types de ligne
			geometry_msgs::Pose2D capteur;
			capteur.x = mesureL[i][0];
			capteur.y = mesureL[i][1];
			capteur.theta = dMaha_X;
			publisher_capteur.publish(capteur);

			capteur.x = mesureL[i][0];
			capteur.y = mesureL[i][1];
			capteur.theta = 1000+dMaha_Y;
			publisher_capteur.publish(capteur);

			//Kalman du X
			if (dMaha_X < mahaThreshold && dMaha_Y > mahaThreshold)
			{
				K = P*C_x.transpose()/((C_x*P*C_x.transpose())(0,0)+Qgamma);
				//Mise à jour de l'état estimé
				X = X + K*delta_mesure_X;
				//Mise à jour de P
				P = (MatrixXd::Identity(3,3)-K*C_x)*P;
			}

			//Kalman du Y
			else if (dMaha_Y < mahaThreshold && dMaha_X > mahaThreshold)
			{
				K = P*C_y.transpose()/((C_y*P*C_y.transpose())(0,0)+Qgamma);
				//Mise à jour de l'état estimé
				X = X + K*delta_mesure_Y;
				//Mise à jour de P
                P = (MatrixXd::Identity(3,3) - K*C_y)*P;
			}
		}
		mesureL.clear();		//penser à vider le vecteur de mesure

		//Publication de l'état et de la matrice d'incertitudes (coefficients diagonaux)
		geometry_msgs::Pose2D state;
		state.x = X[0];
		state.y = X[1];
		state.theta = X[2];
		publisher_state.publish(state);

		geometry_msgs::Pose2D variance;
		variance.x = P(0,0);
		variance.y = P(1,1);
		variance.theta = P(2,2);
		publisher_variance.publish(variance);

		loop_rate.sleep();
	}

	return 0;
}









