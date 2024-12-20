#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "tf/transform_datatypes.h"
#include <cmath>

class pub_sub {

nav_msgs::Odometry messaggio;

private:
	ros::NodeHandle n;

	ros::Subscriber sub;

	ros::Publisher pub;

	ros::Timer timer;

	double reference_latitude;
	double reference_longitude;
	double reference_altitude;

	Eigen::Vector3d previous_ENU;

	Eigen::Vector3d ECEF_reference;

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

	double calculateHeadingFromGPS(Eigen::Vector3d previous, Eigen::Vector3d current) {
		Eigen::Vector3d delta = current - previous;

		double heading = atan2(delta(1, 0), delta(0, 0));

		return heading;
	}

	Eigen::Vector3d rotateOnZAxis(Eigen::Vector3d vector, double angle) {
		Eigen::Matrix3d matrix;

		matrix <<
		cos(angle), -sin(angle), 0,
		sin(angle), cos(angle), 0,
		0, 0, 1;

		Eigen::Vector3d rotated = matrix * vector;

		return rotated;
	}


public:
  	pub_sub() {
  		sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
		pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);


		n.getParam("reference_latitude", reference_latitude);
		n.getParam("reference_longitude", reference_longitude);
		n.getParam("reference_altitude", reference_altitude);


		reference_latitude = toRadians(reference_latitude);
		reference_longitude = toRadians(reference_longitude);
		ECEF_reference = toECEF(reference_latitude, reference_longitude, reference_altitude);

		previous_ENU << 0, 0, 0;


	}

	void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
		messaggio.header = msg->header;

		// to ECEF
		double latitude = toRadians(msg->latitude);
		double longitude = toRadians(msg->longitude);
		double altitude = msg->altitude;

		//ROS_INFO("latitude: %f, longitude: %f, altitude: %f", msg->latitude, msg->longitude, msg->altitude);

		//ROS_INFO("reference_latitude: %f, reference_longitude: %f, reference_altitude: %f", reference_latitude, reference_longitude, reference_altitude);

		Eigen::Vector3d ECEF = toECEF(latitude, longitude, altitude);

		//ROS_INFO("ECEF: %f, %f, %f", ECEF(0, 0), ECEF(1, 0), ECEF(2, 0));

		//ROS_INFO("ECEF_reference: %f, %f, %f", ECEF_reference(0, 0), ECEF_reference(1, 0), ECEF_reference(2, 0));

		// to ENU
		Eigen::Vector3d deltaECEF = ECEF - ECEF_reference;

		Eigen::Matrix3d transformMatrix;

		transformMatrix <<
		-sin(reference_longitude), cos(reference_longitude), 0,
		-sin(reference_latitude) * cos(reference_longitude), -sin(reference_latitude) * sin(reference_longitude), cos(reference_latitude),
		cos(reference_latitude) * cos(reference_longitude), cos(reference_latitude) * sin(reference_longitude), sin(reference_latitude);

		Eigen::Vector3d ENU = transformMatrix * deltaECEF;


		// rotate on Z axis by 130 degrees
		ENU = rotateOnZAxis(ENU, toRadians(130));

		//ROS_INFO("ENU: %f, %f, %f", ENU(0, 0), ENU(1, 0), ENU(2, 0));

		messaggio.pose.pose.position.x = ENU(0, 0);
		messaggio.pose.pose.position.y = ENU(1, 0);
		messaggio.pose.pose.position.z = ENU(2, 0);

		// yaw
		double heading = calculateHeadingFromGPS(previous_ENU, ENU);

		// quaternion
		tf::Quaternion q;
		q.setRPY(0, 0, heading);
		messaggio.pose.pose.orientation.x = q.x();
		messaggio.pose.pose.orientation.y = q.y();
		messaggio.pose.pose.orientation.z = q.z();
		messaggio.pose.pose.orientation.w = q.w();

		previous_ENU = ENU;


		// // velocity
		// double velocity_x = (ENU(0, 0) - previous_ENU_x) / 0.2;
		// double velocity_y = (ENU(1, 0) - previous_ENU_y) / 0.2;
		// double velocity_z = (ENU(2, 0) - previous_ENU_z) / 0.2;

		// messaggio.twist.twist.linear.x = abs(velocity_x);
		// messaggio.twist.twist.linear.y = abs(velocity_y);
		// messaggio.twist.twist.linear.z = abs(velocity_z);

		// previous_ENU_x = ENU(0, 0);
		// previous_ENU_y = ENU(1, 0);
		// previous_ENU_z = ENU(2, 0);
		pub.publish(messaggio);
	}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "gps_to_odom");


	pub_sub my_pub_sub;
	ros::spin();

  	return 0;
}
