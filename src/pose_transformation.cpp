/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2017, Howard Chen <s880367@gmail.com>. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

void arrayCallback(const geometry_msgs::PoseArray::ConstPtr& PoseArray)
{		
	if(&PoseArray->poses[0])
	{	
		ROS_INFO("x = %f", PoseArray->poses[0].position.x);
		ROS_INFO("y = %f", PoseArray->poses[0].position.y);
		ROS_INFO("z = %f", PoseArray->poses[0].position.z);

		ROS_INFO("qx = %f", PoseArray->poses[0].orientation.x);
		ROS_INFO("qy = %f", PoseArray->poses[0].orientation.y);
		ROS_INFO("qz = %f", PoseArray->poses[0].orientation.z);
		ROS_INFO("qw = %f", PoseArray->poses[0].orientation.w);
	}

	/*static tf::TransformBroadcaster brocaster;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(
		PoseArray->poses[0].position.x, 
		PoseArray->poses[0].position.y, 
		PoseArray->poses[0].position.z));

	transform.setRotation(tf::Quaternion(
		PoseArray->poses[0].orientation.x,
		PoseArray->poses[0].orientation.y,
		PoseArray->poses[0].orientation.z,
		PoseArray->poses[0].orientation.w));

	brocaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tag_0", "usb_cam"));*/
	return;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");
	ros::NodeHandle n;
	ros::Subscriber sub3 = n.subscribe("/usb_cam/tag_detections_pose", 100, arrayCallback);
	ros::spin();
	return 0;
}

