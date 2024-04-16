#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"

class pub_sub {

nav_msgs::Odometry messagio;

private:
	ros::NodeHandle n;

	ros::Subscriber sub;

	ros::Publisher pub;

	ros::Timer timer;			//si può togliere se  facciamo publishing a hertz fissati (controllare se il subscriber legge a hertz fissati con comando)


public:
  	pub_sub() {
  		sub = n.subscribe("/fix", 1, &pub_sub::callback, this);
		pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
		timer = n.createTimer(ros::Duration(0.2), &pub_sub::callback1, this);
	}

	void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
		messagio->header = *msg->header;
		messaggio.x = //...
	}

	void callback1(const ros::TimerEvent&) {
		pub.publish(messagio);

  		ROS_INFO("Callback 1 triggered");
	}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "gps_to_odom");

	pub_sub my_pub_sub;
	ros::spin();

  	return 0;
}
