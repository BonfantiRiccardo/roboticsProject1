#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

/*class pub_sub {

std_msgs::String messagio;
std_msgs::String messagio2;

private:
ros::NodeHandle n;

ros::Subscriber sub;
ros::Subscriber sub2;
ros::Publisher pub;
ros::Timer timer1;


public:
  	pub_sub(){
  	sub = n.subscribe("/chatter", 1, &pub_sub::callback, this);
	sub2 = n.subscribe("/chatter2", 1, &pub_sub::callback2, this);
	pub = n.advertise<std_msgs::String>("/rechatter", 1);
	timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback1, this);


}
void callback(const std_msgs::String::ConstPtr& msg){
messagio=*msg;

}

void callback2(const std_msgs::String::ConstPtr& msg){
messagio2=*msg;

}

void callback1(const ros::TimerEvent&)
{
	pub.publish(messagio);
	pub.publish(messagio2);
  	ROS_INFO("Callback 1 triggered");
}





};*/

int main(int argc, char **argv){

	ros::init(argc, argv, "gps_to_odom");
	ros::NodeHandle n;

	ros::Publisher gps_to_odom = n.advertise<nav_msgs::Odometry>("gps_odom", 1);

	ros::Rate loop_rate(10);

	int count = 0;

  	while (ros::ok()){

	    	nav_msgs::Odometry msg;

                msg.header.seq = counter;

                msg.header.stamp.sec = /*int type*/;
                msg.header.stamp.nsec = /*int type*/;

                msg.header.frame_id = /*string type*/;


                msg.child_frame_id = /*string type*/;


                msg.pose.pose.position.x = /*float64  type*/;
                msg.pose.pose.position.y = /*float64  type*/;
                msg.pose.pose.position.z = /*float64  type*/;

                msg.pose.pose.orientation.x = /*float64  type*/;
                msg.pose.pose.orientation.y = /*float64  type*/;
                msg.pose.pose.orientation.z = /*float64  type*/;
                msg.pose.pose.orientation.w = /*float64  type*/;

                msg.pose.covariance = /*float64[36] type*/;


                msg.twist.twist.linear.x = /*float64  type*/;
                msg.twist.twist.linear.y = /*float64  type*/;
                msg.twist.twist.linear.z = /*float64  type*/;

                msg.twist.twist.angular.x = /*float64  type*/;
                msg.twist.twist.angular.y = /*float64  type*/;
                msg.twist.twist.angular.z = /*float64  type*/;

                msg.twist.covariance = /*float64[36] type*/;

                //ROS_INFO("%s", msg.data.c_str());/more complex to print the info

    		gps_to_odom.publish(msg);

    		ros::spinOnce();

    		loop_rate.sleep();
    		++count;
  	}


  	return 0;
}
