//PARAM RECONFIGURE OF THE frame_id OF THE DATA
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>


class pub_sub{
private:

	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	

public:

	//Maybe it has to be initialized
	std::string currentFrame;

	pub_sub(){

  		dynamic_reconfigure::Server<first_project::parametersConfig> server;
    	dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
  		f = boost::bind(&pub_sub::callback1, this, _1, _2);
  		server.setCallback(f);


		//is 1 good for the queue lenght?
		sub = nh.subscribe("/os_cloud_node/points", 1, &pub_sub::callback, this);
		pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);


	}

	void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		ROS_INFO("FRAME: %s", msg->header.frame_id.c_str());
		
	}

	void callback1(first_project::parametersConfig &config, uint32_t level){
		ROS_INFO("Reconficure request: %s", config.frame_id.c_str());
		currentFrame = config.frame_id.c_str();
	}

};

int main(int argc, char **argv){
	ros::init(argc, argv, "lidar_remap");

	pub_sub my_pub_sub;

	ros::spin();

	return 0;
}
