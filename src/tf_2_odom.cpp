#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_2_odom");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("cart_odom", 10);
	tf::TransformListener listener;
	ros::Time time_curr;
	ros::Time time_prev;
	time_prev = ros::Time::now();
  while(nh.ok()){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/map", "/body_aligned_imu_link", ros::Time(0),transform);
    	}
		catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		nav_msgs::Odometry odom_msg;
		time_curr=transform.stamp_;
		if (time_curr > time_prev){
			odom_msg.header.stamp = ros::Time::now();
			odom_msg.header.frame_id = "/map";
			odom_msg.child_frame_id = "/body_aligned_imu_link";
			odom_msg.pose.pose.position.x = transform.getOrigin().x();	
			odom_msg.pose.pose.position.y = transform.getOrigin().y();
			odom_msg.pose.pose.position.z = transform.getOrigin().z();
			odom_msg.pose.pose.orientation.x = transform.getRotation().x();
			odom_msg.pose.pose.orientation.y = transform.getRotation().y();
			odom_msg.pose.pose.orientation.z = transform.getRotation().z();
			odom_msg.pose.pose.orientation.w = transform.getRotation().w();
			pub.publish(odom_msg);
			time_prev = time_curr;
		}
	}
	return 0;
}
