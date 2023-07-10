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
*   static functions							*
************************************************************************/
//! Convert depth value to meters.
template <class T>
inline float	to_meters(T depth)		{ return depth; }
template <>
inline float	to_meters(uint16_t depth)	{ return 0.001f * depth; }

/************************************************************************
*   class ColorDepthExample						*
************************************************************************/
class ColorDepthExample
{
  private:
    using image_t	 = sensor_msgs::Image;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using camera_info_t	 = sensor_msgs::CameraInfo;
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using cloud_t	 = sensor_msgs::PointCloud2;
    using cloud_p	 = sensor_msgs::PointCloud2Ptr;
    using sync_t	 = message_filters::TimeSynchronizer<
				image_t, image_t, camera_info_t>;

  public:
		ColorDepthExample(ros::NodeHandle& nh)			;

  private:
    void	camera_cb(const image_cp& color,
			  const image_cp& depth,
			  const camera_info_cp& camera_info)		;
    template <class T>
    void	create_cloud_from_color_and_depth(
		    const image_cp& color,
		    const image_cp& depth,
		    const camera_info_cp& camera_info)			;

  private:
    image_transport::ImageTransport		_it;
    image_transport::SubscriberFilter		_color_sub;
    image_transport::SubscriberFilter		_depth_sub;
    message_filters::Subscriber<camera_info_t>	_camera_info_sub;
    sync_t					_sync;
    const ros::Publisher			_cloud_pub;
    const cloud_p				_cloud;
};

ColorDepthExample::ColorDepthExample(ros::NodeHandle& nh)
    :_it(nh),
     _color_sub(_it, "/color", 1),
     _depth_sub(_it, "/depth", 1),
     _camera_info_sub(nh, "/camera_info", 1),
     _sync(_color_sub, _depth_sub, _camera_info_sub, 1),
     _cloud_pub(nh.advertise<cloud_t>("pointcloud", 1)),
     _cloud(new cloud_t)
{
  // Register callback for subscribing synched color, depth and camera_info.
    _sync.registerCallback(&ColorDepthExample::camera_cb, this);
}

void
ColorDepthExample::camera_cb(const image_cp& color,
			     const image_cp& depth,
			     const camera_info_cp& camera_info)
{
    if (color->encoding != sensor_msgs::image_encodings::RGB8)
    {
	ROS_ERROR_STREAM("Unknown color encoding[" << color->encoding << ']');
	return;
    }

    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
	create_cloud_from_color_and_depth<uint16_t>(color, depth, camera_info);
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
	create_cloud_from_color_and_depth<float>(color, depth, camera_info);
    }
    else
    {
	ROS_ERROR_STREAM("Unknown depth type[" << depth->encoding << ']');
	return;
    }

    _cloud_pub.publish(_cloud);
}

template <class T> void
ColorDepthExample::create_cloud_from_color_and_depth(
			const image_cp& color,
			const image_cp& depth,
			const camera_info_cp& camera_info)
{
  // Setup fields for 3D coordinates and color of each 3D point.
    sensor_msgs::PointCloud2Modifier	modifier(*_cloud);
    modifier.setPointCloud2Fields(4,
				  "x",   1, sensor_msgs::PointField::FLOAT32,
				  "y",   1, sensor_msgs::PointField::FLOAT32,
				  "z",   1, sensor_msgs::PointField::FLOAT32,
				  "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(depth->height * depth->width);

  // Setup header and size of the output cloud.
    _cloud->header	 = depth->header;
    _cloud->height	 = depth->height;
    _cloud->width	 = depth->width;
    _cloud->row_step	 = _cloud->width * _cloud->point_step;
    _cloud->is_bigendian = false;
    _cloud->is_dense	 = false;		// _cloud->height is not one.

  // Convert camera calibration matrix K and lens distortions D
  // to OpenCV's matrix format.
    cv::Mat_<float>	K(3, 3);
    std::copy_n(std::begin(camera_info->K), 9, K.begin());
    cv::Mat_<float>	D(1, 4);
    std::copy_n(std::begin(camera_info->D), 4, D.begin());

  // Allocate line buffers for pixel and canonical image coordinates.
    cv::Mat_<cv::Point2f>	uv(_cloud->width, 1), xy(_cloud->width, 1);
    for (uint32_t u = 0; u < _cloud->width; ++u)
	uv(u).x = u;			// Setup horizontal coodinates.

  // Set 3D coordinates and color for each point in the output cloud.
    sensor_msgs::PointCloud2Iterator<float>	xyz(*_cloud, "x");
    sensor_msgs::PointCloud2Iterator<uint8_t>	bgr(*_cloud, "rgb");
    for (uint32_t v = 0; v < _cloud->height; ++v)
    {
	for (uint32_t u = 0; u < _cloud->width; ++u)
	    uv(u).y = v;		// Setup vertical coodinates.

      // Convert pixel coordinates (u, v) in the input depth image
      // to canonical image coordinates (x, y) with an unity focal length.
	cv::undistortPoints(uv, xy, K, D);

	auto	p = reinterpret_cast<const T*>(depth->data.data()
					       + v*depth->step);
	auto	q = reinterpret_cast<const uint8_t*>(color->data.data()
						     + v*color->step);
	for (uint32_t u = 0; u < _cloud->width; ++u)
	{
	    const auto	d = to_meters<T>(*p++);		// depth in meters

	    if (d != 0.0f)				// valid depth?
	    {
	    	xyz[0] = xy(u).x * d;	// x
	    	xyz[1] = xy(u).y * d;	// y
	    	xyz[2] = d;		// z
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
}

/************************************************************************
*  class ColorDepthExampleNodelet					*
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

PLUGINLIB_EXPORT_CLASS(depth_camera_tutorial::ColorDepthExampleNodelet,
		       nodelet::Nodelet);

/************************************************************************
*   global functions							*
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
