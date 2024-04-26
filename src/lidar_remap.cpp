//PARAM RECONFIGURE OF THE frame_id OF THE DATA
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>


class pub_sub{

	//Maybe it has to be initialized
	std::string currentFrame;

	sensor_msgs::PointCloud2 messaggio;

private:

	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;


public:


	pub_sub(){
		//is 1 good for the queue lenght?
		sub = nh.subscribe("/os_cloud_node/points", 1, &pub_sub::callback, this);
		pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
	}

	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		ROS_INFO("FRAME before: %s", msg->header.frame_id.c_str());
		ROS_INFO("Current frame parameter: %s", currentFrame.c_str());
		//potrei dover creare una copia del msg
		//msg->header.frame_id = this->currentFrame;

		messaggio = *msg;
		ROS_INFO("FRAME in the middle: %s", messaggio.header.frame_id.c_str());

		messaggio.header.frame_id = this->currentFrame.c_str();

		pub.publish(messaggio);
		ROS_INFO("FRAME after: %s", messaggio.header.frame_id.c_str());
	}

	void configCallback(first_project::parametersConfig &config, uint32_t level){
		ROS_INFO("Reconficure request: %s", config.frame_id.c_str());
		currentFrame = config.frame_id.c_str();						//THE MISTAKE IS HERE CURRENT FRAME DOES NOT MAINTAIN HIS VALUE IN
		ROS_INFO("Parameter updated: %s", currentFrame.c_str());	//DIFFERENT CALLBACK CALLS
	}
};




int main(int argc, char **argv){
	ros::init(argc, argv, "lidar_remap");

	pub_sub my_pub_sub;

	dynamic_reconfigure::Server<first_project::parametersConfig> server;
	dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
	f = boost::bind(&pub_sub::configCallback, my_pub_sub, _1, _2);
	server.setCallback(f);


	ros::spin();

	return 0;
}
