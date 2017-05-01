/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2017, Howard Chen <s880367@gmail.com>. All rights reserved.
 *
 * Author : Howard Chen <s880367@gmail.com>
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

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

 #define row 3
 #define col 4

struct Quaternion
{
	double x;
	double y;
	double z;
	double w;
};

double RotationMatrix[row][col];
string tcp_link, camera_link;
bool publish_tf;


/****************************************
* qw= √(1 + m00 + m11 + m22) /2
* qx = (m21 - m12)/( 4 *qw)
* qy = (m02 - m20)/( 4 *qw)
* qz = (m10 - m01)/( 4 *qw)
*
* example : 
  [ 1        0.0019  -0.0025   -0.6286
    -0.0019   1       -0.0011   71.9933
    0.025    0.0011   1        37.6066 ]
****************************************/
Quaternion Rot2Quaternion(double Rot[][col])
{
	Quaternion q;

	q.w = sqrt(1+Rot[0][0] + Rot[1][1] + Rot[2][2])/2;
	q.x = (Rot[2][1]-Rot[1][2])/(4*q.w);
	q.y = (Rot[0][2]-Rot[2][0])/(4*q.w);
	q.z = (Rot[1][0]-Rot[0][1])/(4*q.w);

	return q;
}

void Static_publisher(double rot[][col], Quaternion q)
{
	static tf::TransformBroadcaster brocaster;
	tf::Transform transform;

	transform.setOrigin( tf::Vector3(rot[0][3], rot[1][3], rot[2][3]));
	transform.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
	brocaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tcp_link, camera_link));
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "RotMatrix_convertor");
	ros::NodeHandle n;
	bool params_loaded = true;

	params_loaded *= n.getParam("tcp_link",tcp_link);
	params_loaded *= n.getParam("camera_link",camera_link);
	params_loaded *= n.getParam("publish_tf",publish_tf);

	if(!params_loaded)
	{
		ROS_ERROR("Couldn'tfind all parameters. Closing.....");
		return -1;
	}

	ROS_INFO("Please enter rotation matrix, order: a00-->a01-->a02.....-->a22\n");

	for (int i = 0; i < row; ++i)
	{
		for (int j = 0; j < col; ++j)
		{
			if(ros::ok())
				scanf("%lf",&RotationMatrix[i][j]);
			else
				return 0;
		}
	}

	printf("===============  the matrix : ===============\n");
	for (int i = 0; i < row; ++i)
	{	
		for (int j = 0; j < col; ++j)
		{
			printf("%lf ",RotationMatrix[i][j] );
		}
		printf("\n");
	}
	printf("=============================================\n");

	Quaternion q = Rot2Quaternion(RotationMatrix);
	ROS_INFO("Quaternion = [ %lf %lf %lf %lf ]\n",q.x, q.y, q.z, q.w );

	while(ros::ok() && publish_tf == true)
		Static_publisher(RotationMatrix,q);

	return 0;
}

