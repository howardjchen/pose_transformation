#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"


void arrayCallback(const geometry_msgs::PoseArray::ConstPtr& PoseArray)
{
	//cout << PoseArray->poses[0].position.x<< endl;
	ROS_INFO("x = %f",PoseArray->poses[0].position.x);
	ROS_INFO("y = %f",PoseArray->poses[0].position.y);
	ROS_INFO("z = %f",PoseArray->poses[0].position.z);

	ROS_INFO("qx = %f",PoseArray->poses[0].orientation.x);
	ROS_INFO("qy = %f",PoseArray->poses[0].orientation.y);
	ROS_INFO("qz = %f",PoseArray->poses[0].orientation.z);
	ROS_INFO("qw = %f",PoseArray->poses[0].orientation.w);
	return;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");
	ros::NodeHandle n;	
	ros::Subscriber sub3 = n.subscribe("/tag_detections_pose", 100, arrayCallback);

	ros::spin();
	return 0;
}

