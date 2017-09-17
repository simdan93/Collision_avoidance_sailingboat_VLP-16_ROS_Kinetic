/*
	#Anchorpoint - by Daniel Simonsen

	This program is responsible for creating an array of waypoints around a certain GPS coordinate point Autonomus will follow for 5 minutes before getting one waypoint which is the last point for which the boat will conclude its mission.
		
	Note! Exactly how far Autonomus can stay away from the initial point is uncertain. But we will for now guess 20 meters from the point.  
*/
//#include "utregninger.h"
//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
//MATH
#include <math.h>
//general
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <string>
#include <chrono>

using namespace std;

typedef chrono::high_resolution_clock Time;
typedef chrono::duration<float> fsec;

#define PI 3.1415926535897

ros::Publisher waypoint_pub;
ros::Publisher endOfTask_pub;

bool startTimer_ = false;
bool waypointsExistence_ = false;
int count_ = 0;
int zoneRadius_ = 20;
int radius_ = zoneRadius_ / 2;
int numberWaypoints_ = 8;
int distPoint = zoneRadius_ / 2; 
double longitude_ref[8], latitude_ref[8];
double angles[8];
float compass_ = 45.0;
float compass_radian_;

std_msgs::String anchorWaypoint;
std_msgs::String lastWaypoint;

string createAnchorPoints(int count, double zone_latitude, double zone_longitude);
void kompass_update(const std_msgs::Float64::ConstPtr& msg);
void loop_finished(const std_msgs::Bool::ConstPtr& msg);
void reached_zone(const std_msgs::Bool::ConstPtr& msg);

int 
main(int argc, char **argv)
{	
	ros::init(argc, argv, "anchor_waypoints");
	ros::NodeHandle n;
	
	//SUBS
	ros::Subscriber kompass_sub = n.subscribe("yaw_in_euler_angles", 1, kompass_update);
	ros::Subscriber WaypointsExistence_sub = n.subscribe("waypointsExistence", 1, loop_finished);
	ros::Subscriber reachedZone_sub = n.subscribe("zoneReached", 1, reached_zone);
	//PUB
	waypoint_pub = n.advertise<std_msgs::String>("waypoint", 1);
	
	string msg_out;
	string latAndLon;
	int count2 = 0;
	double zoneLatitude = 59.432885; 
	double zoneLongitude = 10.470121;
	double endWaypointLon = 59.429502;
	double endWaypointLat = 10.465930;

	auto t0 = Time::now();
	fsec fs;

	ros::Rate loop_rate(1);
	while (n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		if(count_ == 0)
		{
			cout<<"sending zone waypoint..."<<endl;
			msg_out = "i;0;" + to_string(zoneLatitude) + ";" + to_string(zoneLongitude)  + ";" + to_string(zoneRadius_);
			anchorWaypoint.data = msg_out;
			waypoint_pub.publish(anchorWaypoint);
		}
		else if(count_ == 1)
		{
			cout<<endl;
			cout<<"creating "<<numberWaypoints_<<" waypoints around zone"<<endl;
			cout<<"Referancepoint: "<<zoneLatitude<<";"<<zoneLongitude<<endl;	
			cout<<"Zone radius: "<<zoneRadius_<<endl; 
			cout<<"Radius of acceptance from waypoints: "<<radius_<<endl;
			cout<<"angle of boat: "<<compass_<<endl;
			cout<<endl;

			while(count2 < numberWaypoints_)
			{
				sleep(1);
				latAndLon = createAnchorPoints(count2, zoneLatitude, zoneLongitude);
				msg_out = "i;" + to_string(count2+1) + ";" + latAndLon + ";" + to_string(radius_);
				anchorWaypoint.data = msg_out;
				waypoint_pub.publish(anchorWaypoint);
				count2++;
				
			}
		}
		else if(count_ == 2 && startTimer_ == true)
		{
			cout<<"Autonomus is inside zone. Starting timer..."<<endl;
			while (n.ok() && fs.count() < 300) // 5 min ~ 300 sec
			{
				ros::spinOnce();
				loop_rate.sleep();
				//republish waypoints previously created
				if(waypointsExistence_ == false)
				{
					cerr << "There are no waypoints!" << endl << "republishing waypoints..."<<endl;
					for(int i=0;i<numberWaypoints_;i++)
					{
						sleep(1);
						msg_out = "i;" + to_string(i) + ";" + to_string(longitude_ref[i]) + ";" + to_string(latitude_ref[i])  + ";" + to_string(radius_);
						cout << msg_out << endl;
						anchorWaypoint.data = msg_out;
						waypoint_pub.publish(anchorWaypoint);
					}
					cerr << "waypoints published " << endl;
				}
				auto t1 = Time::now();
				fs = t1 - t0;
				cout << "Seconds: " << fs.count() << "/300" << endl;
			}
			if(fs.count() >= 300)
			{
				cerr << "Times up." << endl;
				//Send flag for deletion of waypoints
				cerr<<"Sending last waypoint before shutting down..."<<endl;
				msg_out = "i;0;" + to_string(endWaypointLon) + ";" + to_string(endWaypointLat)  + ";" + to_string(radius_);
				cout<<"Last msg: "<<msg_out<<endl; 
				
				lastWaypoint.data = msg_out;
				waypoint_pub.publish(lastWaypoint);
			}	
		}
		count_++;
	}
	
	//ros::spin();
	return 0;
}

/** UPDATES FROM SUBSCRIBERS**/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
kompass_update(const std_msgs::Float64::ConstPtr& msg)
{	
	compass_ = (float)msg->data;
}
void
loop_finished(const std_msgs::Bool::ConstPtr& msg)
{
	waypointsExistence_ = (bool)msg->data;
	if(waypointsExistence_ == false)
		cout<<"loop finished"<<endl;
	else
		cout<<"loop not finished"<<endl;
}
void
reached_zone(const std_msgs::Bool::ConstPtr& msg)
{
	startTimer_ = (bool)msg->data;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

string
createAnchorPoints(int count, double zone_latitude, double zone_longitude)
{
	double a;
	double dlongitude, dlatitude;
	string result;
	double aQuarterPI = PI / (numberWaypoints_/2);
	//Creating 8 points around a set point (values longitude and latitude)
	compass_radian_ = compass_ * (PI / 180);
	if(count == 0)
	{	
		if(compass_radian_ > PI)
			angles[count] = compass_radian_ - PI;
		else
			angles[count] = compass_radian_ + PI;	
	
		//angles[count] = angles[count] - (PI / (numberWaypoints_/2)); //First angle value is 45 degrees less for a smoother turn to the circle we are creating
	}
	else
	{
		if(angles[count-1] < aQuarterPI)
		{
			a = aQuarterPI - angles[count-1];
			angles[count] = (2 * PI) - a;
		}
		else
		{
			angles[count] = angles[count-1] - aQuarterPI;
		}
	}
	dlatitude = (sin (angles[count]) * distPoint) * 0.00001;
	dlongitude = (cos (angles[count]) * distPoint) * 0.00002;
	latitude_ref[count] = zone_latitude + dlatitude;
	longitude_ref[count] = zone_longitude + dlongitude;

	result = to_string(latitude_ref[count]) + ";" + to_string(longitude_ref[count]);
	cout<<"waypoint "<<count<<": "<<result<<endl;
	return result;		
}
