//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
//MATH
#include <math.h>
//general
#include <stdio.h>
#include <string>

using namespace pcl;
using namespace std;

typedef PointCloud<PointXYZI> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef typename Cloud::Ptr CloudPtr;

#define PI 3.1415926535897

CloudConstPtr cloud_;
CloudPtr cloud_allowed_;

ros::Publisher waypoint_pub;

std_msgs::String waypoint_msg_;

boost::mutex mtx_;

bool waypointReached_ = false;
bool changeTurn_ = false;
int sideToTurn_ = 0;
double farestRightX_, farestLeftX_, farestRightY_, farestLeftY_;
double newWaypointX_, newWaypointY_;
float positionOfBoatLon_;
float positionOfBoatLat_;

float compass_degrees_; //for testing
int distanceFromRefPoint_ = 15;
int counter_ = 0;
string referancePointGPS_;
string result_;

//check if valid hindrance
int counterAfterDetection_ = 0;
bool pointsDetected_  = false;

void cloud_callback(const CloudConstPtr &cloud);
void GPS_update(const sensor_msgs::NavSatFix::ConstPtr& msg);
void kompass_update(const std_msgs::Float64::ConstPtr& msg);
void waypointReached_update(const std_msgs::Bool::ConstPtr& msg);
void changeTurn_update(const std_msgs::Bool::ConstPtr& msg);

void checkIfValidCloud(const CloudConstPtr &cloud, Cloud &result);
void turnDecider(const CloudConstPtr &cloud);
void findNewWaypoint();
string convertXY_toLatLon(double pointX, double pointY);


int
main (int argc, char ** argv)
{
	// iniitialize ROS node
	ros::init(argc, argv, "antiCollision");
  	ros::NodeHandle nh;
 	
	cout << "Ready and waiting to recieve pointclouds from topic: filter" << endl;

	// SUBS
	ros::Subscriber filter_sub = nh.subscribe<Cloud> ("filter", 1, cloud_callback);
	ros::Subscriber GPS_sub = nh.subscribe("fix", 1, GPS_update);
	ros::Subscriber kompass_sub = nh.subscribe("yaw", 1, kompass_update);	
	ros::Subscriber waypointReached_sub = nh.subscribe("waypointReached", 1, waypointReached_update);
	ros::Subscriber changeTurn_sub = nh.subscribe("changeTurn", 1, changeTurn_update);
	// PUB
	waypoint_pub = nh.advertise <std_msgs::String> ("waypoint", 1);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
		if(changeTurn_ == true)
		{
			//Change the stored turnvalue to the opposite
			if(sideToTurn_ == 2)	
			{
				sideToTurn_ = 1;
				newWaypointX_ = farestLeftX_ - distanceFromRefPoint_; 
				newWaypointY_ = farestLeftY_;
			}
			else
			{
				sideToTurn_ = 2;
				newWaypointX_ = farestRightX_ + distanceFromRefPoint_; 
				newWaypointY_ = farestRightY_;
			}
			changeTurn_ = false;
		}
		findNewWaypoint();
	}
	//ros::spin();
	return 0;
}

//VLP Grabber's cloud Callback function
void
cloud_callback(const CloudConstPtr &cloud)
{
	//boost::mutex::scoped_lock lock (mtx_);
	cout<<endl<<"#Points in: "<<cloud->width<<endl;

	//A little check if points found are still present after 20 spins (~2 seconds), if not 
	//then the points were not a hindrance, but most likely water. 
	cloud_allowed_.reset (new Cloud);
	checkIfValidCloud(cloud, *cloud_allowed_);

	cout << "Current position:" << endl << "Longitude: " << positionOfBoatLon_ << endl << "Latitude: " << positionOfBoatLat_ << endl;
	//find what side to turn and to what point (XY)
	turnDecider(cloud_allowed_);
}

/** UPDATES FROM SUBSCRIBERS**/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//callback method for GPS update
void 
GPS_update(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
	positionOfBoatLon_= (float)msg->longitude;
	positionOfBoatLat_= (float)msg->latitude;
}
//Callback for the yaw(heading)
void 
kompass_update(const std_msgs::Float64::ConstPtr& msg)
{	
	compass_degrees_ = (float)msg->data;
}
//Callback for if waypoint has been reached
void 
waypointReached_update(const std_msgs::Bool::ConstPtr& msg)
{
	waypointReached_ = (bool)msg->data;
}
//Callback if change of turn is necessary
void 
changeTurn_update(const std_msgs::Bool::ConstPtr& msg)
{
	changeTurn_ = (bool)msg->data;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
checkIfValidCloud(const CloudConstPtr &cloud, Cloud &result)
{		
	if(cloud->width > 0 && pointsDetected_ == false)
	{
		cout<<"Points detected!"<<endl;
		pointsDetected_ = true;
	}
	if(pointsDetected_ == true && cloud->width > 0)
	{
		cout<<"Counter hindrance exists: "<<counterAfterDetection_<<endl;
		counterAfterDetection_++;
	}
	
	if(pointsDetected_ == true && cloud->width == 0)
	{
		cerr<<"Invalid hindrance."<<endl;
		counterAfterDetection_ = 0;
		pointsDetected_ = false;
	}

	if(counterAfterDetection_ == 20)
	{
		result.width = cloud->width;
		result.height = cloud->height;
		result.is_dense = cloud->is_dense;

		counterAfterDetection_ = 0;
		pointsDetected_ = false;

		cout<<"Points has been around for "<<counterAfterDetection_<<" iterations. Valid hindrance."<<endl;
		
		for (size_t i = 0; i < cloud->points.size (); i++)
		{
			PointXYZI point;
	  		point.x = cloud->points[i].x;
	  		point.y = cloud->points[i].y;
	  		point.z = cloud->points[i].z;
			point.intensity = cloud->points[i].intensity;
	  		result.points.push_back (point);
		}
	}
}

void
turnDecider(const CloudConstPtr &cloud)
{
	int countRight, countLeft = 0;
	int index = 0;
	farestRightX_, farestLeftX_ = 0;
	farestRightY_, farestLeftY_ = 0;

	if(cloud->points.size() == 0)
		sideToTurn_ = 0;
	else 
	{
		farestRightX_, farestLeftX_ =  cloud->points[0].x;
		for(size_t i=1;i<cloud->points.size();i++)
		{
			if((cloud->points[i].x) <= 0)
				countLeft++;
			else
				countRight++;

			if(farestRightX_ < cloud->points[i].x)
			{
				farestRightX_ = cloud->points[i].x;
				farestRightY_ = cloud->points[i].y;
			}
			if(farestLeftX_ > cloud->points[i].x)
			{
				farestLeftX_ = cloud->points[i].x;
				farestLeftY_ = cloud->points[i].y;
			}
		}
		if(countLeft > countRight || countLeft == countRight){
			sideToTurn_ = 2;
			cout << "Turn right" << endl;
			//referancePointGPS_ = convertXY_toLonLat(farestRightX_, farestRightY_);
			newWaypointX_ = farestRightX_ + distanceFromRefPoint_; 
			newWaypointY_ = farestRightY_;
		}
		else{
			sideToTurn_ = 1;
			cout << "Turn left" << endl;
			//referancePointGPS_ = convertXY_toLonLat(farestLeftX_, farestLeftY_);
			newWaypointX_ = farestLeftX_ - distanceFromRefPoint_; 
			newWaypointY_ = farestLeftY_;
		}
	}
}

void
findNewWaypoint()
{
	int radius = 10;
	string latAndLon;
	if(sideToTurn_ == 1 || sideToTurn_ == 2)
	{
		// Convert XY to GPS-format
		//cout << "New point 15 m from referancepoint: " << endl;
		latAndLon = convertXY_toLatLon(newWaypointX_, newWaypointY_);
		cout<<"Waypoint reached? "<<waypointReached_<<" counter: "<<counter_<<endl;
		if(waypointReached_ == false && counter_ != 0)
			result_ = "o;0;" + latAndLon + ";" + to_string(radius);
		else
		{
			result_ = "i;0;" + latAndLon + ";" + to_string(radius);
			counter_ = 1;
			waypointReached_ = false;
		}
		//cout << "Resulting msg: " << result_ << endl << endl;
		waypoint_msg_.data = result_;
		waypoint_pub.publish (waypoint_msg_);
	}
	else
		cout << "Go straight, no change of waypoint" << endl;
}

string convertXY_toLatLon(double pointX, double pointY)
{
	double distPoint;
	double a, b;
	double sinx, pointAngle, temp;
	double dlatitude, dlongitude;
	double vinkelTilPunktFraSensor, vinkelTilPunktFraSensor_temp;
	double pointGPSLon_, pointGPSLat_;
	float compass_radian; //for testing
	string result;

	cout << "pointXY-value in: " << pointX << ";" << pointY << endl;
	
	//1. Find the distance to waypoint using x and y
	if(pointX == 0 || pointY == 0)
	{
		cerr << "dLatM or dLonM is zero" << endl;
		if(pointX == 0 && pointY == 0)
			cerr << "both are zero, no distance" << endl;
		else
		{
			if(pointX == 0)
				distPoint = pointY;
			if(pointY < 0)
				distPoint = pointX;
		}
	}				
	else
	{
		a = pointX * pointX;
		b = pointY * pointY;
		distPoint = sqrt(a + b);	
	}
	//cout << "Distance to point: " << distPoint << endl;

	//2. Find the angle from the waypoint to the sensor with respect of the XY-value
	sinx = pointX / distPoint;
	pointAngle = asin(sinx);
	//cout << "Angle (degrees) from waypoint (XY): " <<  (pointAngle * (180 / PI)) << endl;
	//cout << "Angle (radian) from waypoint (XY): " <<  pointAngle << endl;

	compass_radian = compass_degrees_ * (PI / 180);

	//3. Find the angle from the sensor to the waypoint with respect to the compass
	vinkelTilPunktFraSensor_temp = compass_radian + pointAngle;		
	if (vinkelTilPunktFraSensor_temp < 0)
		vinkelTilPunktFraSensor = (2 * PI) + vinkelTilPunktFraSensor_temp;
	else if (vinkelTilPunktFraSensor_temp > (2 * PI))
	{
		temp = (2 * PI) - vinkelTilPunktFraSensor_temp;
		vinkelTilPunktFraSensor = (-1) * temp;
	}
	else
		vinkelTilPunktFraSensor = vinkelTilPunktFraSensor_temp;
	//cout << "Angle (radian) to waypoint (GPS): " <<  vinkelTilPunktFraSensor << endl;
	
	//4. calculate longitude and latitude with the given angle and distance	
	dlatitude = (cos (vinkelTilPunktFraSensor) * distPoint) * 0.00001; 
	dlongitude = (sin (vinkelTilPunktFraSensor) * distPoint) * 0.00002;
	//cout << "Change in lat and long: " << dlatitude << ", " << dlongitude << endl;

	//6. Add the distance in lon and lat with the our current position and return the result
	pointGPSLat_ = positionOfBoatLat_ + dlatitude;
	pointGPSLon_ = positionOfBoatLon_ + dlongitude;
	cout << "New waypoint (GPS): "  << pointGPSLat_ << ";" << pointGPSLon_ << endl;
	
	//7. Return the result
	result = to_string(pointGPSLat_) + ";" + to_string(pointGPSLon_);
	return result;
}
