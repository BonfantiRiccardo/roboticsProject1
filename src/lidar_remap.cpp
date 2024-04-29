#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

std::string currentFrame;

class pub_sub{
	sensor_msgs::PointCloud2 messaggio;

private:

	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;

public:


	pub_sub(){
		sub = nh.subscribe("/os_cloud_node/points", 1, &pub_sub::callback, this);
		pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
	}

	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		//ROS_INFO("Current frame parameter: %s", currentFrame.c_str());

		messaggio = *msg;

		messaggio.header.frame_id = currentFrame.c_str();

		pub.publish(messaggio);
	}

	void configCallback(first_project::parametersConfig &config, uint32_t level){
		//ROS_INFO("Reconficure request: %s", config.frame_id.c_str());
		nh.setParam("currentFrame", config.frame_id.c_str());

		nh.getParam("currentFrame", currentFrame);
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
