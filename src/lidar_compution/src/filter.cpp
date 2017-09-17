//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/boost.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/console/parse.h>
//ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
//MATH
#include <math.h>
//general
#include <stdio.h>
//#include <pcl/common/time.h> //fps calculations
//#include <pcl/common/transforms.h>

#define PI 3.1415926535897

using namespace std;
using namespace pcl;
using namespace pcl::console;

template <typename PointType>

class CloudModifier
{
  public:
	typedef PointCloud<PointType> Cloud;

    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename Cloud::Ptr CloudPtr;

    CloudModifier (	boost::shared_ptr<VLPGrabber> grabber ) 
		: grabber_ (grabber)
    {}

	void 
	filterPassThroughVerticaly (const CloudConstPtr &cloud, Cloud &result)
  	{
    	PassThrough<PointType> pass;
    	pass.setFilterFieldName ("z"); // den horisontale aksen
    	pass.setFilterLimits (-0.4, 4.0); // alt som er under 30 centimeter blir slettet
    	pass.setInputCloud (cloud);
    	pass.filter (result);
  	}
	void 
	filterPassThroughY (const CloudConstPtr &cloud, Cloud &result)
  	{
    	PassThrough<PointType> pass;
    	pass.setFilterFieldName ("y");
    	pass.setFilterLimits (4.0, 100);
    	pass.setInputCloud (cloud);
    	pass.filter (result);
  	}
	void 
	filterPassThroughX (const CloudConstPtr &cloud, Cloud &result)
  	{	
    	PassThrough<PointType> pass;
    	pass.setFilterFieldName ("x"); 
    	pass.setFilterLimits (xLimitRevert_, xLimit_); //xLimitRevert_ og xLimit_ regnet ut globalt
    	pass.setInputCloud (cloud);
    	pass.filter (result);
  	}

  	void 
	gridSample (const CloudConstPtr &cloud, 
				Cloud &result)
  	{
		double leaf_size = 0.03;
		VoxelGrid<PointType> grid;
    	grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
    	grid.setInputCloud (cloud);
    	grid.filter (result);
  	}

	void
	filterIntensity(const CloudConstPtr &cloud, Cloud &result)
	{
		// Checks the intensity-value of the point to see if it's high enough. Under 25 is most likely water or something similar.
		result.width = cloud->width;
		result.height = cloud->height;
		result.is_dense = cloud->is_dense;
		for (size_t i = 0; i < cloud->points.size(); i++)
		{
			//Checking if the value of intensity is high enough. Too low and water will be included. 
			if(cloud->points[i].intensity < 30)
				result.width--;
			else
			{
				cout<<cloud->points[i].intensity<<endl;
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
	filter_angle (const CloudConstPtr &cloud, Cloud &result)
	{
		result.width = cloud->width;
		result.height = cloud->height;
		result.is_dense = cloud->is_dense;
		for (size_t i = 0; i < cloud->points.size (); i++)
		{
			xValue = cloud->points[i].x;
			yValue = cloud->points[i].y;
			/* finds the invers tan (radians) from x and y axis of the single point
			   and finds the degree */
			dResult = (atan (xValue / yValue)) * (180 / PI); // x value MOST be before y value
			// degree most be between visionAngle_ and -visionAngle_ degrees
			if(dResult > visionAngle_ && dResult < (-1) * visionAngle_)
				result.width--;
			else
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

	//VLP Grabber's cloud Callback function
    void
    cloud_callback (const CloudConstPtr &cloud)
    {
		boost::mutex::scoped_lock lock (mtx_);	

		//Filter roughly what we don't want by cutting in the XYZ axis from the sensor
		cloud_pass_z_.reset (new Cloud);
		filterPassThroughVerticaly (cloud, *cloud_pass_z_);
		cloud_pass_y_.reset (new Cloud);
		filterPassThroughY (cloud_pass_z_, *cloud_pass_y_);
		cloud_pass_x_.reset (new Cloud);
		filterPassThroughX (cloud_pass_y_, *cloud_pass_x_);
		//Filter unecessary dense clusters
		cloud_pass_downsampled_.reset (new Cloud);
		gridSample (cloud_pass_x_, *cloud_pass_downsampled_);
		//Filter points with low intensity (water gains low intensity)
		cloud_no_water_.reset (new Cloud);
		filterIntensity(cloud_pass_downsampled_, *cloud_no_water_);
		//Filter every point that isn't within a certain angle to sensor
		cloud_filtered_angle_.reset (new Cloud);
		filter_angle(cloud_no_water_, *cloud_filtered_angle_);
  	}	

    void
    run ()
    {	
      	boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (&CloudModifier::cloud_callback, this, _1);
	  	boost::signals2::connection cloud_connection = grabber_->registerCallback (cloud_cb);

		grabber_->setMinimumDistanceThreshold(fMin_);
		grabber_->setMaximumDistanceThreshold(fMax_);
      	grabber_->start ();

		ros::NodeHandle nh;
		ros::Publisher pubCloud = nh.advertise<Cloud> ("filter", 1);
		ros::Rate loop_rate(10);
		while (nh.ok())
		{
			ros::spinOnce();
			loop_rate.sleep ();

			boost::mutex::scoped_try_lock lock( mtx_ );
			if( lock.owns_lock() && cloud_filtered_angle_)
			{
				pcl_conversions::toPCL(ros::Time::now(), cloud_filtered_angle_->header.stamp);
				pubCloud.publish (cloud_filtered_angle_);
			}
		}
		grabber_->stop ();
      	cloud_connection.disconnect();
    }
	
	double xValue, yValue;
	double dResult;
	double visionMaxDistance_ = 30.0;
	//Find the max value of x according to the max distance and max angle
	double visionAngleRadian = visionAngle_ * (PI / 180);
	double xLimit_ = sin (visionAngleRadian) * visionMaxDistance_;
	double xLimitRevert_ = (-1) * xLimit_;
	float fMin_ = 4;
	float fMax_ = 30;
	int visionAngle_ = 10;
	
	CloudConstPtr cloud_;  	
	CloudPtr cloud_pass_z_;
	CloudPtr cloud_pass_y_;
	CloudPtr cloud_pass_x_;
  	CloudPtr cloud_pass_downsampled_;
	CloudPtr cloud_filtered_angle_;
	CloudPtr cloud_no_water_;

	boost::shared_ptr<VLPGrabber> grabber_;
  	boost::mutex mtx_;
	//double computation_time_;
};


void
usage (char ** argv)
{
	cout << "usage: " << argv[0] << endl
            << " [-ipaddress <192.168.1.70>]" << endl
            << " [-port <2368>]" << endl
            << " [-pcap] <" << argv[1] << ".pcap>]" << endl
            << " [-help]" << endl;

  	cout << argv[0] << " -h | --help : shows this help" << endl;
}

int
main (int argc, char ** argv)
{
	// iniitialize ROS node
	ros::init (argc, argv, "filter");
  	// Command-Line Argument Parsing
	if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  	{
    	usage (argv);
    	return (0);
  	}
	// default values
	string ipaddress( "192.168.1.77" );
    string port( "2368");
	string pcap; 

	console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    console::parse_argument( argc, argv, "-port", port );
    console::parse_argument( argc, argv, "-pcap", pcap );

    cout << "-ipadress : " << ipaddress << endl;
    cout << "-port : " << port << endl;
    cout << "-pcap : " << pcap << endl;
	
	// VLP Grabber
    boost::shared_ptr<VLPGrabber> grabber;
    if( !pcap.empty() ){
        cout << "Capture from PCAP..." << endl;
        grabber = boost::shared_ptr<VLPGrabber>(new VLPGrabber(pcap));
    }
    else if( !ipaddress.empty() && !port.empty() ){
        cout << "Capture from Sensor..." << endl;    
		grabber = boost::shared_ptr<VLPGrabber>(new VLPGrabber(boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port )));
    }

    CloudModifier<PointXYZI> modifier (grabber);
	modifier.run ();

  	return (0);
}


