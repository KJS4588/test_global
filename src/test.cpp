#include "ros/ros.h"
#include "iostream"
#include "ackermann_msgs/AckermannDrive.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "visualization_msgs/Marker.h"
#include "tf/tf.h"
#include <geometry_msgs/Pose2D.h>


#define _USE_MATH_DEFINES
using namespace std;
double ggx, ggy, ggz;
double lx, ly, lz, yaw_d;
double pre_angle;

void odomCallback(const nav_msgs::Odometry::ConstPtr &odomsg){
	lx = odomsg->pose.pose.position.x;
	ly = odomsg->pose.pose.position.y;
	lz = odomsg->pose.pose.position.z;
	tf::Quaternion q(
		odomsg->pose.pose.orientation.x,
		odomsg->pose.pose.orientation.y,
		odomsg->pose.pose.orientation.z,
		odomsg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	yaw_d = yaw*180/M_PI;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
    
	ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd",10);
   	ros::Subscriber sub_o = nh.subscribe("/odom", 1 , odomCallback);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
 	double cur_steer;

    nh.setParam("global_goal_x", ggx);
    nh.setParam("global_goal_y", ggy);

    ackermann_msgs::AckermannDriveStamped ackerData_;
    ackerData_.drive.steering_angle = 0;
    ackerData_.drive.speed = 0;
    
    while(ros::ok()){
        nh.getParam("global_goal_x", ggx);
        nh.getParam("global_goal_y", ggy);
	visualization_msgs::Marker points;
	points.header.frame_id = "map";
	points.header.stamp = ros::Time::now();
	points.ns = "points_and_lines";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 1; 
	points.scale.y = 1;
	points.color.a = 1.0;
	points.color.g = 1.0f;
	geometry_msgs::Point p;
	p.x = ggx;
	p.y = ggy;
	p.z = 0;
	points.points.push_back(p);
	
	double dx = ggx-lx;
        double dy = ggy-ly;

	double dist = sqrt(dx*dx + dy*dy);
	double dist_hp = 0.5; //50cm

	double ab_degree = atan2(ggy-ly, ggx-lx)*180/M_PI;
	double true_angle;
	pre_angle = ackerData_.drive.steering_angle;
	if (dist < dist_hp){
 	    	ackerData_.drive.speed = 0;
		ackerData_.drive.steering_angle = 0;
	}
	else{
		if (dx>=0 && dy>=0) true_angle = 90.0-ab_degree;
		else if (dx>=0 && dy<=0) true_angle = 90.0-ab_degree;
		else if (dx<=0 && dy>=0) true_angle = 270.0-ab_degree;
		else if (dx>=0 && dy<=0) true_angle = 270.0-ab_degree;
		
		cur_steer = true_angle - pre_angle;
		if(cur_steer<-180) cur_steer = 360 - pre_angle + true_angle;
		else if(cur_steer>180) cur_steer = true_angle - pre_angle - 360;

		ackerData_.drive.steering_angle = cur_steer;
		//ackerData_.drive.steering_angle = 25;			
		ackerData_.drive.speed = 2;
		}
	//cout << ackerData_ << endl;
	//cout << M_PI;
	cout << "steering angle: " << ackerData_.drive.steering_angle << endl;
	pub.publish(ackerData_);
	marker_pub.publish(points);
	ros::spinOnce();
	}
	return 0;
}
