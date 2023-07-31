/*!
 *  \file	color_depth_example.cpp
 *  \author	Toshio Ueshiba
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace depth_camera_tutorial
{
/************************************************************************
*   inline functions							*
************************************************************************/
//! Convert depth value to meters.
template <class T>
inline float	to_meters(T depth)		{ return depth; }
inline float	to_meters(uint16_t depth)	{ return 0.001f * depth; }

/************************************************************************
*   class ColorDepthExample						*
************************************************************************/
class ColorDepthExample
{
  private:
    using sync_t = message_filters::TimeSynchronizer<sensor_msgs::Image,
						     sensor_msgs::Image,
						     sensor_msgs::CameraInfo>;

  public:
		ColorDepthExample(ros::NodeHandle& nh)			;

  private:
    void	camera_cb(const sensor_msgs::ImageConstPtr& color,
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& camera_info);
    template <class T>
    sensor_msgs::PointCloud2ConstPtr
		create_cloud_from_color_and_depth(
		    const sensor_msgs::ImageConstPtr& color,
		    const sensor_msgs::ImageConstPtr& depth,
		    const sensor_msgs::CameraInfoConstPtr& camera_info)
								const	;

  private:
    image_transport::ImageTransport			 _it;
    image_transport::SubscriberFilter			 _color_sub;
    image_transport::SubscriberFilter			 _depth_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> _camera_info_sub;
    sync_t						 _sync;
    const ros::Publisher				 _cloud_pub;
};

ColorDepthExample::ColorDepthExample(ros::NodeHandle& nh)
    :_it(nh),
     _color_sub(_it, "/color", 1),
     _depth_sub(_it, "/depth", 1),
     _camera_info_sub(nh, "/camera_info", 1),
     _sync(_color_sub, _depth_sub, _camera_info_sub, 1),
     _cloud_pub(nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1))
{
  // Register callback for subscribing synched color, depth and camera_info.
    _sync.registerCallback(&ColorDepthExample::camera_cb, this);
}

void
ColorDepthExample::camera_cb(const sensor_msgs::ImageConstPtr& color,
			     const sensor_msgs::ImageConstPtr& depth,
			     const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    if (color->encoding != sensor_msgs::image_encodings::RGB8)
    {
	ROS_ERROR_STREAM("Unknown color encoding[" << color->encoding << ']');
	return;
    }

    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
	_cloud_pub.publish(create_cloud_from_color_and_depth<uint16_t>(
			       color, depth, camera_info));
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
	_cloud_pub.publish(create_cloud_from_color_and_depth<float>(
			       color, depth, camera_info));
    }
    else
    {
	ROS_ERROR_STREAM("Unknown depth type[" << depth->encoding << ']');
    }
}


template <class T> sensor_msgs::PointCloud2ConstPtr
ColorDepthExample::create_cloud_from_color_and_depth(
		const sensor_msgs::ImageConstPtr& color,
		const sensor_msgs::ImageConstPtr& depth,
		const sensor_msgs::CameraInfoConstPtr& camera_info) const
{
  // Allocate memory for output pointcloud from heap and setup header and size.
    const sensor_msgs::PointCloud2Ptr	cloud(new sensor_msgs::PointCloud2);
    cloud->header	= depth->header;
    cloud->height	= depth->height;
    cloud->width	= depth->width;
    cloud->is_bigendian = false;
    cloud->is_dense	= false;		// cloud->height is not one.

  // Setup fields for 3D coordinates and color of each 3D point.
  // Two fields, point_step and row_step of pointcloud is setup appropriately
  // and required internal buffer, i.e. data, is allocated based on the values
  // of height, width and row_step fields.
    sensor_msgs::PointCloud2Modifier	modifier(*cloud);
    modifier.setPointCloud2Fields(4,
				  "x",   1, sensor_msgs::PointField::FLOAT32,
				  "y",   1, sensor_msgs::PointField::FLOAT32,
				  "z",   1, sensor_msgs::PointField::FLOAT32,
				  "rgb", 1, sensor_msgs::PointField::FLOAT32);

  // Convert camera calibration matrix K and lens distortions D
  // to OpenCV's matrix format.
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());

  // Allocate line buffers for pixel and canonical image coordinates.
    cv::Mat_<cv::Point2f>	uv(cloud->width, 1), xy(cloud->width, 1);
    for (uint32_t u = 0; u < cloud->width; ++u)
	uv(u).x = u;			// Setup horizontal coodinates.

  // Set 3D coordinates and color for each point in the output cloud.
    sensor_msgs::PointCloud2Iterator<float>	xyz(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t>	bgr(*cloud, "rgb");
    for (uint32_t v = 0; v < cloud->height; ++v)
    {
	for (uint32_t u = 0; u < cloud->width; ++u)
	    uv(u).y = v;		// Setup vertical coodinates.

      // Convert pixel coordinates (u, v) in the input depth image
      // to canonical image coordinates (x, y) with an unity focal length.
	cv::undistortPoints(uv, xy, K, D);

	auto	p = reinterpret_cast<const T*>(depth->data.data()
					       + v*depth->step);
	auto	q = reinterpret_cast<const uint8_t*>(color->data.data()
						     + v*color->step);
	for (uint32_t u = 0; u < cloud->width; ++u)
	{
	    const auto	d = to_meters(*p++);		// depth in meters

	    if (d != 0.0f)				// valid depth?
	    {
	    	xyz[0] = xy(u).x * d;	// X
	    	xyz[1] = xy(u).y * d;	// Y
	    	xyz[2] = d;		// Z
		bgr[0] = q[2];		// blue
		bgr[1] = q[1];		// green
		bgr[2] = q[0];		// red
	    }
	    else
	    {
	    	xyz[0] = xyz[1] = xyz[2]
		       = std::numeric_limits<float>::quiet_NaN(); // null point
		bgr[0] = bgr[1] = bgr[2] = 0;
	    }

	    q += 3;
	    ++xyz;
	    ++bgr;
	}
    }

    return cloud;
}

/************************************************************************
*  class ColorDepthExampleNodelet: requried only for building nodelets	*
************************************************************************/
class ColorDepthExampleNodelet : public nodelet::Nodelet
{
  public:
			ColorDepthExampleNodelet()			{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<ColorDepthExample>	_node;
};

void
ColorDepthExampleNodelet::onInit()
{
    NODELET_INFO("depth_camera_tutorial::ColorDepthExampleNodelet::onInit()");
    _node.reset(new ColorDepthExample(getPrivateNodeHandle()));
}
}	// namespace depth_camera_tutorial

// required only for building nodelets
PLUGINLIB_EXPORT_CLASS(depth_camera_tutorial::ColorDepthExampleNodelet,
		       nodelet::Nodelet);

/************************************************************************
*   main(): required only for building conventional(non-nodelet) nodes	*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "color_depth_example");

    try
    {
	ros::NodeHandle					nh("~");
	depth_camera_tutorial::ColorDepthExample	example(nh);

	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
