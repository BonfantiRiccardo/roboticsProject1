#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "eigen3/Eigen/Dense"
#include <cmath>

class pub_sub {

nav_msgs::Odometry messaggio;

private:
	ros::NodeHandle n;

	ros::Subscriber sub;

	ros::Publisher pub;

	ros::Timer timer;			//si può togliere se  facciamo publishing a hertz fissati (controllare se il subscriber legge a hertz fissati con comando)

	double reference_latitude = 45.477669461666665;
	double reference_longitude = 9.22674018;
	double reference_altitude = 169.039;

	double toRadians(double degrees) {
		return degrees * M_PI / 180;
	}

	Eigen::Vector3d toECEF(double latitude, double longitude, double altitude) {
		double a = 6378137.0;
		double b = 6356752.0;
		double eUp2 = 1 - ((pow(b, 2))/(pow(a, 2)));
		double nConstant = a / sqrt(1 - eUp2 * pow(sin(latitude), 2));

		double x = (nConstant + altitude) * cos(latitude) * cos(longitude);
		double y = (nConstant + altitude) * cos(latitude) * sin(longitude);
		double z = (nConstant * (1 - eUp2) + altitude) * sin(latitude);

		Eigen::Vector3d ECEF;
		ECEF << x, y, z;

		return ECEF;
	}

public:
  	pub_sub() {
  		sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
		pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
		timer = n.createTimer(ros::Duration(0.2), &pub_sub::callback1, this);
	}

	void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
		messaggio.header = msg->header;

		// to ECEF
		double latitude = toRadians(msg->latitude);
		double longitude = toRadians(msg->longitude);
		double altitude = msg->altitude;
		
		Eigen::Vector3d ECEF = toECEF(latitude, longitude, altitude);

		reference_latitude = toRadians(reference_latitude);
		reference_longitude = toRadians(reference_longitude);
		
		Eigen::Vector3d ECEF_reference = toECEF(reference_latitude, reference_longitude, reference_altitude);

		// to ENU
		Eigen::Vector3d vector1 = ECEF - ECEF_reference;

		Eigen::Matrix3d matrix2;
		matrix2 << 
		-sin(reference_longitude), cos(reference_longitude), 0,
		-sin(reference_latitude) * cos(reference_longitude), -sin(reference_latitude) * sin(reference_longitude), cos(reference_latitude),
		cos(reference_latitude) * cos(reference_longitude), cos(reference_latitude) * sin(reference_longitude), sin(reference_latitude);

		Eigen::MatrixXd ENU = matrix2 * vector1;

		messaggio.pose.pose.position.x = ENU(0, 0);
		messaggio.pose.pose.position.y = ENU(1, 0);
		messaggio.pose.pose.position.z = ENU(2, 0);

		// orientation don't know how to calculate it
	}

	void callback1(const ros::TimerEvent&) {
		pub.publish(messaggio);

  		ROS_INFO("Callback 1 triggered");
	}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "gps_to_odom");

	pub_sub my_pub_sub;
	ros::spin();

  	return 0;
}
