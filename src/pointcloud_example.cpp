/*!
 *  \file	pointcloud_example.cpp
 *  \author	Toshio Ueshiba
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace threed_camera_tutorial
{
/************************************************************************
*   class PointCloudExample						*
************************************************************************/
class PointCloudExample
{
  private:
    using cloud_cp = sensor_msgs::PointCloud2ConstPtr;
    using image_t  = sensor_msgs::Image;
    using image_p  = sensor_msgs::ImagePtr;
    
  public:
		PointCloudExample(ros::NodeHandle& nh)			;

  private:
    void	cloud_cb(const cloud_cp& cloud)				;

  private:
    const ros::Subscriber		_cloud_sub;
    image_transport::ImageTransport	_it;
    const image_transport::Publisher	_color_pub;
    const image_p			_color;
};

PointCloudExample::PointCloudExample(ros::NodeHandle& nh)
    :_cloud_sub(nh.subscribe("/pointcloud", 1,
			     &PointCloudExample::cloud_cb, this)),
     _it(nh),
     _color_pub(_it.advertise("color", 1)),
     _color(new image_t)
{
}

void
PointCloudExample::cloud_cb(const cloud_cp& cloud)
{
  // Check if the input cloud consists of a 2D map of 3D points.
    if (cloud->height == 1)
    {
	ROS_ERROR_STREAM("Cannot create color image from unorganized pointcloud!");
	return;
    }

  // Check if each 3D point contains color information.
    if (std::find_if(cloud->fields.begin(), cloud->fields.end(),
		     [](const auto& field){ return field.name == "rgb"; })
	== cloud->fields.end())
    {
	ROS_ERROR_STREAM("No color information found in the input pointcloud!");
	return;
    }

  // Setup fields in the color image and allocate data buffer.
    _color->header	 = cloud->header;
    _color->height	 = cloud->height;
    _color->width	 = cloud->width;
    _color->encoding	 = sensor_msgs::image_encodings::RGB8;
    _color->is_bigendian = false;
    _color->step	 = _color->width * 3*sizeof(uint8_t);
    _color->data.resize(_color->height * _color->step);

  // Extract color information from each point in the input cloud.
    sensor_msgs::PointCloud2ConstIterator<uint8_t>	bgr(*cloud, "rgb");
    for (uint32_t v = 0; v < _color->height; ++v)
    {
	auto	rgb = reinterpret_cast<uint8_t*>(_color->data.data()
						 + v*_color->step);

	for (uint32_t u = 0; u < _color->width; ++u)
	{
	    rgb[0] = bgr[2];
	    rgb[1] = bgr[1];
	    rgb[2] = bgr[0];

	    rgb += 3;
	    ++bgr;
	}
    }

  // Publish color image.
    _color_pub.publish(_color);
}
}	// namespace threed_camera_tutorial

/************************************************************************
*   global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    ros::init(argc, argv, "pointcloud_example");

    try
    {
	ros::NodeHandle					nh("~");
	threed_camera_tutorial::PointCloudExample	example(nh);

	ros::spin();
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
	return 1;
    }

    return 0;
}

