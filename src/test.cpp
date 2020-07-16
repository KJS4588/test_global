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
double lx, ly, lz, yaw_d, yaw;
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
	double roll, pitch;
	m.getRPY(roll, pitch, yaw);

	yaw_d = yaw*180/M_PI;
	//cout << yaw_d << endl;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;
    
	ros::Publisher pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ctrl_cmd",10);
   	ros::Subscriber sub_o = nh.subscribe("/odom", 1 , odomCallback);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
	
	double dx = (ggx-lx);
    double dy = (ggy-ly);
	double local_x, local_y;
	double dist = sqrt(dx*dx + dy*dy);
	double dist_hp = 1; //50cm

	double ab_degree = abs(atan2(ggy-ly, ggx-lx));
	double true_angle;
	double degree = atan2(dy, dx);

	double steer;
	pre_angle = ackerData_.drive.steering_angle;
	if (dist < dist_hp){
 	    ackerData_.drive.speed = 0;
		ackerData_.drive.steering_angle = 0;
		ackerData_.drive.brake = 200;
	}
	else if(dist < 1.5 && dist > dist_hp){
		ackerData_.drive.speed = 1;
		ackerData_.drive.steering_angle = pre_angle;

	}
	else{
		cout << "yaw: " << yaw_d << endl;
		if (yaw_d >= 0 && yaw_d < 90){	
			local_x = dx*cos(yaw) + dy*sin(yaw);
			local_y = -dx*sin(yaw) + dy*cos(yaw);
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
			cout << "state1" << endl;
		}
		else if (yaw_d >= 90 && yaw_d < 180){
			local_x = -dx*cos(M_PI-yaw) + dy*sin(M_PI-yaw);
			local_y = -dx*sin(M_PI-yaw) - dy*cos(M_PI-yaw);
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
			cout << "state2" << endl;
		}
		else if (yaw_d >= -90 && yaw_d < 0){
			local_x = dx*cos(abs(yaw)) - dy*sin(abs(yaw));
			local_y = dx*sin(abs(yaw)) + dy*cos(abs(yaw));
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
			cout << "state3" << endl;
		}
		else if (yaw_d >= -180 && yaw_d < -90){
			local_x = -dx*cos(M_PI-abs(yaw)) - dy*sin(M_PI-abs(yaw));
			local_y = dx*sin(M_PI-abs(yaw)) - dy*cos(M_PI-abs(yaw));
			steer = atan(local_y/local_x);
			if (local_x>0) steer = steer; 
			else 
				if (local_y>0) steer = M_PI - abs(steer);
				else steer = -(M_PI - abs(steer));
			/*if (dy<0 && dx<0){
				steer = -(M_PI-steer);
				cout << "####" << endl;}
			else if (dy>0 && dx<0) steer = M_PI-abs(steer);*/
			cout << "state4" << endl;
		}
		
		/*if(dx>=0 && dy>0) true_angle = M_PI/2 - degree;
		else if(dx>=0 && dy<0) true_angle = M_PI/2 - degree;
		else if(dx<0 && dy<0) true_angle = 3/2*M_PI - degree;
		else if(dx<0 && dy>0) true_angle = 3/2*M_PI - degree;
		
		double cur_steer = (true_angle-yaw)* 180/M_PI;
        
		ackerData_.drive.steering_angle = cur_steer;*/
		ackerData_.drive.steering_angle = -steer*180/M_PI;
		pre_angle = ackerData_.drive.steering_angle;
		cout << ackerData_.drive.steering_angle << endl;
		ackerData_.drive.speed = 2;
		}
	//cout << ackerData_ << endl;
	//cout << M_PI;
	//cout << "steering angle: " << ackerData_.drive.steering_angle << endl;
	//cout << ackerData_ << endl;
	pub.publish(ackerData_);
	marker_pub.publish(points);
	ros::spinOnce();
	}
	return 0;
}
