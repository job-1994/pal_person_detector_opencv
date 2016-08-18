/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Job van Dieten <job.1994@gmail.com> */

// PAL Headers
#include <pal_detection_msgs/Detections2d.h>

//ROS Headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>


// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class TiagoMove
{
public:
	TiagoMove(ros::NodeHandle& nh);
	~TiagoMove();
    ros::Subscriber detection_sub, laser_sub;
	ros::Publisher pub_laser, pub_vel;

	std::vector<pal_detection_msgs::Detection2d> det_vector;
	int pos_x;
    bool move_laser= true;
    bool move_detection = false;

    ros::ServiceClient head_client;
protected:
	void detectionsCB(const pal_detection_msgs::Detections2d& frame);
	void laserCB(const sensor_msgs::LaserScan::ConstPtr& scan);
	void movement();	

};

TiagoMove::TiagoMove(ros::NodeHandle& nh)
{
	detection_sub = nh.subscribe("person_detector/detections", 1, &TiagoMove::detectionsCB, this);
    pub_vel = nh.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel/", 1000);
    laser_sub = nh.subscribe("/scan", 1, &TiagoMove::laserCB, this);
    head_client = nh.serviceClient<std_srvs::Empty>("find_head");
}

TiagoMove::~TiagoMove(){}


void TiagoMove::detectionsCB(const pal_detection_msgs::Detections2d& frame)
{
	det_vector = frame.detections;

	det_vector.resize(1);
	pos_x = det_vector.front().x + det_vector.front().width/2;
    if(det_vector.front().x == 0 && det_vector.front().y == 0)
        move_detection = false;
    else
        move_detection = true;
}

void TiagoMove::laserCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	int vector_size = scan->ranges.size();
	float dist_vals[vector_size];
	int start = (vector_size/2) - (0.1*vector_size); 
	int end = (vector_size/2) + (0.1*vector_size); 
    move_laser= true;
	for(int i = start; i<end; ++i)
	{
    	dist_vals[i] = scan->ranges[i];
    	if(dist_vals[i] < 1 && dist_vals[i] != 0)
    	{
            move_laser= false;
            break;
    	}
	}

	this->movement();
//    if(move_laser== false)
//    {
//        std_srvs::Empty emp;
//        if(head_client.call(emp))
//            ROS_INFO("called service");
//    }
}

void TiagoMove::movement()
{
	geometry_msgs::Twist cmd;
	int mid_x = 320;
	if(pos_x > 650)
		pos_x = 0;
	float diff = std::abs (pos_x - mid_x);
	float angular_speed = diff/500;

    if(move_laser== true && move_detection == true)
		cmd.linear.x = 0.3;
	else
		cmd.linear.x = 0;

	if(pos_x < mid_x && pos_x !=0)
		cmd.angular.z = angular_speed;
	else if(pos_x > mid_x && pos_x!=0)
		cmd.angular.z = -angular_speed;
	else
	{
		cmd.angular.z = 0;
	}
	
	pub_vel.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"tiago_move"); 
  ros::NodeHandle nh;

  TiagoMove tm(nh);

  ros::spin();
  return 0;
}
