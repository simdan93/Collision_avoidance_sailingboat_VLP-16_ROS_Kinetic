#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <std_msgs/Int8.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>

#include <boost/algorithm/string.hpp>
#include <pcl/console/parse.h>
//general
#include <iostream>
#include <vector>
#include <string>
#include <typeinfo>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

typedef PointCloud<PointXYZI> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef typename Cloud::Ptr CloudPtr;

CloudConstPtr cloud_;
boost::shared_ptr<PCLVisualizer> viewer_ (new PCLVisualizer ("PCL VLP Cloud Visualizer"));
boost::mutex mtx_;
string topic_("filter");

void callback (const CloudConstPtr& msg);
void usage (char ** argv);

int
main (int argc, char ** argv)
{
	ros::init(argc, argv, "visualize");
	ros::NodeHandle nh;
/*
	cout << "Default topic: downsampled" << endl
            << "Choose topics:  -topic <topic> "<<endl 
			<< "\tpassthrough" << endl
			<< "\tsegmented" << endl
			<< "\tkeypoints" << endl;

	if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  	{
    	usage (argv);
    	return (0);
  	}
*/
	console::parse_argument( argc, argv, "-topic", topic_ );
	cout << "-topic : " << topic_ << endl;

	viewer_->addCoordinateSystem (1.0, "global");
	viewer_->setBackgroundColor (0, 0, 0);
	viewer_->initCameraParameters ();
	viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	viewer_->setCameraClipDistances (0.0, 100.0);

	ros::Subscriber sub = nh.subscribe<Cloud>(topic_, 1, callback);
	ros::spin();
	return 0;
}

void 
callback (const CloudConstPtr& msg)
{
	viewer_->spinOnce ();

	cloud_.reset(new Cloud);
	CloudConstPtr cloud2;
	cloud_ = msg;
	
	if (mtx_.try_lock ())
	{
		cloud_.swap (cloud2);   		
		mtx_.unlock ();
	}
	PointCloudColorHandlerCustom<PointXYZI> grey (cloud2, 100, 100, 100);
	PointCloudColorHandlerCustom<PointXYZI> red (cloud2, 250, 0, 0);
	PointCloudColorHandlerCustom<PointXYZI> blue (cloud2, 0, 0, 250);
	PointCloudColorHandlerGenericField<PointXYZI> instensity (cloud2, "intensity");
	if (cloud2)
	{
		// checks topic for how to colour point cloud
		//if (!topic_.compare("filter"))
			if (!viewer_->updatePointCloud (cloud2, instensity, "msg_cloud"))
				viewer_->addPointCloud (cloud2, instensity, "msg_cloud");
		/*if (!topic_.compare("segmented"))
			if (!viewer_->updatePointCloud (cloud2, red, "msg_cloud"))
				viewer_->addPointCloud (cloud2, red, "msg_cloud");
		if (!topic_.compare("keypoints"))
			if (!viewer_->updatePointCloud (cloud2, blue, "msg_cloud"))
				viewer_->addPointCloud (cloud2, blue, "msg_cloud");*/
	}
}

void
usage (char ** argv)
{
	cout << "usage: " << argv[0] << endl
            << " [-topic <point_cloud>]" << endl
            << " [-help]" << endl;

  	cout << argv[0] << " -h | --help : shows this help" << endl;
  	return;
}

