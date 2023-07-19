/*!
 *  \file	depth_example.cpp
 *  \author	Toshio Ueshiba
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
*   class DepthExample							*
************************************************************************/
class DepthExample
{
  private:
    using camera_info_cp = sensor_msgs::CameraInfoConstPtr;
    using image_cp	 = sensor_msgs::ImageConstPtr;
    using cloud_t	 = sensor_msgs::PointCloud2;
    using cloud_p	 = sensor_msgs::PointCloud2Ptr;
    using cloud_cp	 = sensor_msgs::PointCloud2ConstPtr;

  public:
		DepthExample(ros::NodeHandle& nh)			;

  private:
    void	camera_cb(const image_cp& depth,
			  const camera_info_cp& camera_info)		;
    template <class T>
    cloud_cp	create_cloud_from_depth(const image_cp& depth,
					const camera_info_cp& camera_info);

  private:
    image_transport::ImageTransport	_it;
    image_transport::CameraSubscriber	_camera_sub;
    const ros::Publisher		_cloud_pub;
};

DepthExample::DepthExample(ros::NodeHandle& nh)
    :_it(nh),
     _camera_sub(_it.subscribeCamera("/depth", 1,
				     &DepthExample::camera_cb, this)),
     _cloud_pub(nh.advertise<cloud_t>("pointcloud", 1))
{
}

void
DepthExample::camera_cb(const image_cp& depth,
			const camera_info_cp& camera_info)
{
    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
	_cloud_pub.publish(create_cloud_from_depth<uint16_t>(depth,
							     camera_info));
    }
    else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
	_cloud_pub.publish(create_cloud_from_depth<float>(depth, camera_info));
    }
    else
    {
	ROS_ERROR_STREAM("Unknown depth type[" << depth->encoding << ']');
    }
}

template <class T> DepthExample::cloud_cp
DepthExample::create_cloud_from_depth(const image_cp& depth,
				      const camera_info_cp& camera_info)
{
  // Setup fields for 3D coordinates of each 3D point.
    const cloud_p			cloud(new cloud_t);
    sensor_msgs::PointCloud2Modifier	modifier(*cloud);
    modifier.setPointCloud2Fields(3,
				  "x", 1, sensor_msgs::PointField::FLOAT32,
				  "y", 1, sensor_msgs::PointField::FLOAT32,
				  "z", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(depth->height * depth->width);

  // Setup header and size of the output cloud.
    cloud->header	 = depth->header;
    cloud->height	 = depth->height;
    cloud->width	 = depth->width;
    cloud->row_step	 = cloud->width * cloud->point_step;
    cloud->is_bigendian = false;
    cloud->is_dense	 = false;		// cloud->height is not one.

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

  // Set 3D coordinates for each point in the output cloud.
    sensor_msgs::PointCloud2Iterator<float>	xyz(*cloud, "x");
    for (uint32_t v = 0; v < cloud->height; ++v)
    {
	for (uint32_t u = 0; u < cloud->width; ++u)
	    uv(u).y = v;		// Setup vertical coodinates.

      // Convert pixel coordinates (u, v) in the input depth image
      // to canonical image coordinates (x, y) with an unity focal length.
	cv::undistortPoints(uv, xy, K, D);

	auto	p = reinterpret_cast<const T*>(depth->data.data()
					       + v*depth->step);
	for (uint32_t u = 0; u < cloud->width; ++u)
	{
	    const auto	d = to_meters<T>(*p++);		// depth in meters

	    if (d != 0.0f)	// valid depth?
	    {
	    	xyz[0] = xy(u).x * d;
	    	xyz[1] = xy(u).y * d;
	    	xyz[2] = d;
	    }
	    else
	    {
	    	xyz[0] = xyz[1] = xyz[2]
		       = std::numeric_limits<float>::quiet_NaN(); // null point
	    }

	    ++xyz;
	}
    }

    return cloud;
}

/************************************************************************
*  class DepthExampleNodelet: requried only for building nodelets	*
************************************************************************/
class DepthExampleNodelet : public nodelet::Nodelet
{
  public:
			DepthExampleNodelet()				{}

    virtual void	onInit()					;

  private:
    boost::shared_ptr<DepthExample>	_node;
};

void
DepthExampleNodelet::onInit()
{
    NODELET_INFO("depth_camera_tutorial::DepthExampleNodelet::onInit()");
    _node.reset(new DepthExample(getPrivateNodeHandle()));
}
}	// namespace depth_camera_tutorial

// required only for building nodelets
PLUGINLIB_EXPORT_CLASS(depth_camera_tutorial::DepthExampleNodelet,
		       nodelet::Nodelet);

/************************************************************************
*   main(): required only for building conventional(non-nodelet) nodes	*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "depth_example");

    try
    {
	ros::NodeHandle				nh("~");
	depth_camera_tutorial::DepthExample	example(nh);

	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}
