#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub{
public:
  	tf_sub_pub() {
        n.getParam("root_frame", root_frame);
        n.getParam("child_frame", child_frame);
        ROS_INFO("Root frame: %s, child frame: %s", root_frame.c_str(), child_frame.c_str());
  	    sub = n.subscribe("/input_odom", 1000, &tf_sub_pub::callback, this);
    }


    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        ROS_INFO("MSG POSE: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        ROS_INFO("MSG ORIENT: %f, %f, %f, %f", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
        transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_frame, child_frame));
    }

private:
    ros::NodeHandle n;
    tf::TransformBroadcaster br;
    ros::Subscriber sub;

    std::string root_frame;
    std::string child_frame;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "odom_to_tf");
    tf_sub_pub my_tf_sub_pub;
    ros::spin();
    return 0;
}
