depth_camera_tutorial ([Japanese](./README.md))
==================================================
## Abstract
This package describes the basic knowledge required to develop ROS nodes to process point clouds using a depth camera. Specifically, the following items are explained and sample code of ROS nodes that process the output from the depth camera is provided as implementation examples.
- Structure of the depth camera and its output ([1](#1-depth_camera))
- How to subscribe to the pointcloud output from the depth camera and process the individual 3D points that make up the pointcloud ([3.1](#31-pointcloud_example))
- How to subscribe to multiple topics output by the depth camera while keeping time synchronization ([3.2](#32-depth_example), [3.3](#33-color_depth_example))
- How to convert a depth image output by the depth camera into a 3D pointcloud ([3.2](#32-depth_example))
- How to handle the depth and color images output by the depth camera simultaneously([3.3](#33-color_depth_example))
- How to realize zero-copy communication between multiple nodes running on the same host by implementing ROS nodes with [nodelets](http://wiki.ros.org/nodelet)([4](#4-zero_copy_communication_within_the_same_process_using_nodelets))

## 1. Depth camera
### 1.1 Output of depth camera
Many depth cameras have both depth and color (or gray) sensors to capture depth and color information of the object, respectively. Usually, the two are separate hardware (e.g., [Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)), but sometimes the same hardware plays both roles (e.g., [Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)).

The ROS driver for the depth camera outputs
- **depth image** ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type): A 2D array of pixels each value of which is the distance along the optical axis from the camera center to the object
- **depth sensor camera parameters** ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) type): horizontal/vertical focal lengths, principal point, lens distortions, etc.
- **color image** ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type): A 2D array of color values of pixels obtained from the object
- **pointcloud** ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) type): Point cloud consisting of 3D points computed from depth image and camera parameters

Each point in a pointcloud of type [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.htm) can optionally store its color value and a 3D vector representing the normal.

The old type [sensor_msgs/PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html) is now deprecated.
### 1.2 Depth value type
The ROS driver of the depth camera outputs each pixel of the depth image as `float` or [uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html) type, the former in meters and the latter in millimeters ([see here](https://ros.org/reps/rep-0118.html)). The distinction between the two is reflected in the `encoding` field of the [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type message.
- encoding = **sensor_msgs::image_encodings::TYPE_32FC1**: `float` type
- encoding = **sensor_msgs::image_encodings::TYPE_16UC1**: [uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html) type

Most drivers output depth values as `float` type, but [original driver for Realsense](https://github.com/IntelRealSense/realsense-ros) uses [uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html). In this case, the minimum unit is 1mm, so when the distance between the camera and the object is small(close-up), the 3D point cloud recovered from the depth value may have a staircase-like artifact due to quantization error. To avoid this, another driver modified to output depths as float type is [available](https://gitlab.com/art-aist/realsense-ros), which is recommended to be used for close-up shots.

### 1.3 Handling of Invalid Pixels
Generally, the depth image acquired by a depth camera contains invalid pixels for which no depth value can be obtained. Typical causes of invalid pixels include occlusions such as 3D points observed from only one sensor (in the case of stereo), 3D points not illuminated by the projector (in the case of coded light pattern), and lack of reflected light such as black objects.

Invalid pixels are represented on the topic message as follows ([see here](https://ros.org/reps/rep-0118.html)).
- **pointcloud** ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) type): Set [NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html) to x, y and z coordinates to indicate invalid pixels.
- **depth image** ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type): Set `0` to the depth value to indicate invalid pixels.

### 1.4 Organized/unorganized pointcloud
There are two formats of pointcloud of type [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html). One is `unorganized pointcloud` which arranges all the observed points in a one-dimensional array, and the other is `organized pointcloud` which represents the point set as a two-dimensional array corresponding to each pixel of the depth sensor. The former is compact because it does not include invalid pixels for which no 3D information was obtained, but the adjacency between pixels is lost. The latter consumes storage space for invalid pixels as well, so the overall size is larger, but since the adjacency between pixels is preserved, it is easy to know the neighboring points for any point in the pointcloud.

The distinction between the two is reflected in the `height`, `width`, and `is_ distance` fields of [sensor_msgs/PointCloud2]() type messages.
- **unorganized pointcloud**: `height` = 1, `width` = number of points, `is_dense` = true
- **organized pointcloud**: `height` = number of rows in 2D array, `width` = number of columns in 2D array, `is_dense` = false

### 1.5 Pointcloud or depth image?
When exchanging depth data between ROS nodes, you can choose between pointcloud and depth image formats. The advantages and disadvantages of both formats are as follows.

In case of pointcloud
- The role of calculating 3D coordinates from the depth values can be entrusted to the camera driver.
- If `organized pointcloud` format is adopted, the total amount of data is larger than the depth image, and the communication load increases.
- Color information can be included for each point in the pointcloud, but color values for invalid pixels without depth values are lost.

In case of depth image
- 3D coordinates must be calculated from the depth values in the user program
- Both depth image and camera parameters need to be published/subscribed
- If color information is required, color images must be published/subscribed
- Total data volume is smaller than pointcloud, and the communication load is lighter.
- Using [image_transport](http://wiki.ros.org/image_transport), the communication load can be reduced by image compression
## 2. Build the package
First, install `OpenCV`, then download and build it as follows.
```
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist/depth_camera_tutorial
$ catkin build detph_camera_tutorial
```
In order to run the sample program, you need a depth camera. Either any model of [Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html) or [Photoneo](https://www.photoneo.com/) will do.

## 3. Sample programs
### 3.1 pointcloud_example
[pointcloud_example](src/pointcloud_example.cpp) is a sample ROS node that demonstrates how to retrieve properties of each 3D point in the pointcloud. Specifically, it inputs the pointcloud from the depth camera, extracts color information, and outputs it as a 2D image.
- **Input topic**: pointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) type) from depth camera
- **Output topic**: 2D image ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type ) consisting of color values assigned to each point in the pointcloud

Start the node as follows.
````
$ roslaunch depth_camera_tutorial run.launch prog:=pointcloud_example camera_name:=[realsense|phoxi]
````
The main points of the program is as follows.
- [Check](src/pointcloud_example.cpp#L45) that the `height` of the subscribe pointcloud is not `1` to make sure it is an *organized* pointcloud.
- [Confirm](src/pointcloud_example.cpp#L52-54) that the pointcloud contains color information by checking if there is a field named `rgb` in its `fields`.
- [Access](src/pointcloud_example.cpp#L71) the `rgb` field of a 3D point in pointcloud via [sensor_msgs::PointCloud2ConstIterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2ConstIterator.html)
- Note that the order of color components in the `rgb` field is `b`, `g`, `r` from the lower byte ([see code](src/pointcloud_example.cpp#L79-81))
- [Publish](src/pointcloud_example.cpp#L89) a 2-D color image through a [publisher](src/pointcloud_example.cpp#L36-37) generated using [image_transport](http://wiki.ros.org/image_transport).
- The local variable `color` representing the color image is not of type `sensor_msgs::Image` but of type `sensor_msg::ImagePtr` ([see code](src/pointcloud_example.cpp#L61)), which is indeed ` boost::shared_ptr<sensor_msgs::Image>`. By allocating the memory area of the image from the heap via `shared_ptr`, serialization/deserialization can be omitted when publishing/subscribing the image in the same process ([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing)), which reduces communication load.
- The contents of color images held via `shared_ptr` must not be changed after publishing the message ([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing)). Therefore, the memory area pointed to by the variable `color` must be allocated from the heap every frame.

### 3.2 depth_example
[depth_example](src/depth_example.cpp) is a sample ROS node that shows how to convert each pixel in a depth image into a 3D point. Specifically, it inputs the depth image and camera parameters from the depth camera, calculates the 3D coordinates of each pixel, and outputs them as a pointcloud.
- **Input topic**: The depth image ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type) from the depth camera and its camera parameters ([ sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html))
- **Output topic**: pointcloud computed from two input topics ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html ) type)

Start the node as follows.
```
$ roslaunch depth_camera_tutorial run.launch prog:=depth_example camera_name:=[realsense|phoxi]
```

The two subscribed topics, the depth image and the camera parameters, must be synchronized in time. In general, to subscribe multiple topics synchronously in ROS, 
[message_filters](http://wiki.ros.org/message_filters) is used. However, if the topics to be synchronized are of type [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) and [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) types, the [CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html) provided by [image_transport](http://wiki.ros.org/image_transport) package is more convenient, which is used in this implementation.

[CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html) is used to subscribe not only depth images, but also general monochrome/color images with camera parameters.

The main points of the program are as follows.
- A [CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html) is [generated](src/depth_example.cpp#L50-52) using [image_transport](http://wiki.ros.org/image_transport). Although the topic name of the camera parameter is not specified, don't worry because the topic `camera_info` in the namespace to which the image topic belongs is automatically subscribed to.
- Determine the type of the depth value from the `encoding` field of the input depth image, call the member function for pointcloud generation accordingly, and publish the result ([see code](src/depth_example.cpp#L61-69))
- Allocate the area of the output pointcloud and [set](src/depth_example.cpp#L82-87) its size. The reason for allocating the memory area from the heap via `shared_ptr` is the same as in [pointcloud_example](src/pointcloud_example.cpp).
- Specify the information that each point in the pointcloud preserves [sensor_msgs::PointCloud2Modifier](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Modifier.html). The modifier allocates the internal buffer area of the pointcloud and sets the information for accessing it. Here, only the 3D coordinate values are specified ([see code](src/depth_example.cpp#L93-97)).
- To calculate the 3D coordinates from the depth value at a pixel (u, v), it is necessary to first remove the effect of lens distortion from (u, v) and then convert it to `canonical image coordinates`(x, y), where the origin is the image principal point and the focal length is `1`. This can be achieved using OpenCV`s [cv::undistortPoints()](https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#ga55c716492470bfe86b0ee9bf3a1f0f7e).
- For this purpose, an array of pixel coordinates `uv` is [created](src/depth_example.cpp#L115-116) for each row of the input depth image, and [passed](src/depth_example.cpp#L120) to [cv::undistortPoints()](https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#ga55c716492470bfe86b0ee9bf3a1f0f7e) with the camera's internal parameter matrix `K` and lens distortion parameter `D` to compute an array `xy` of `canonical image coordinates
- [Access](src/depth_example.cpp#L112) the `x`, `y`, `z` fields via [sensor_msgs::PointCloud2Iterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Iterator.html) in order to compute 3D coordinates of each point in the pointcloud
- If depth value is of type [uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html), [revert to meters](src/depth_example.cpp#L126)
- Multiply `xy` by depth value to compute x, y coordinates of 3D point. z-coordinates use depth value as it is ([see code](src/depth_example.cpp#L130-132))
- If the depth value is `0`, the pixel is invalid, so [NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html) is assigned to the coordinate value ([see code](src/depth_example.cpp#L136-137))

### 3.3 color_depth_example
[color_depth_example](src/color_depth_example.cpp) is a sample ROS node that shows how to transform each pixel into a 3D point with color information by fusing a color image and a depth image. Specifically, it inputs the color image, depth image, and camera parameters from the depth camera, calculates the 3D coordinates of each pixel, assigns a color value to it, and outputs it as a pointcloud with color information.
- **Input topic**: color image and depth image ([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) type) from depth camera and camera parameters ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html) type)
- **Output topics**: pointcloud with color information computed from the three input topics ([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) type)

Start the node as follows.
````
$ roslaunch depth_camera_tutorial run.launch prog:=color_depth_example [camera_name:=realsense|phoxi]
````

Here we need to subscribe to three topics: the synchronized color image, the depth image and the camera parameters, so we use the [message_filters::TimeSynchronizer< M0, M1, M2, M3, M4, M5, M6, M7, M8 >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html) is used. In addition, the camera parameters are defined by [message_filters::Subscriber< M >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html). color and depth images can also be subscribed using this, but [image_transport::SubscriberFilter](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html), which compresses images to reduce communication load [image_transport](http://wiki.ros.org/image_transport), which compresses images to reduce the communication load.

The main points of the program are as follows.
- [Define](src/color_depth_example.cpp#L71-72) the subscriber for the color and depth images as [image_transport::SubscriberFilter](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html) type
- [Define](src/color_depth_example.cpp#L73) the subscriber for the camera parameter as [message_filters::Subscriber< M >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html) type
- [Synchronize](src/color_depth_example.cpp#L74) the three input topics with `synchronizer`([message_filters::TimeSynchronizer< M0, M1, M2, M3, M4, M5, M6, M7, M8 >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html) type) 
- [Set](src/color_depth_example.cpp#L87) `synchronizer` callback function to receive three synchronized input topics
- Calculate 3D coordinates from depth values in the same way as [depth_example](src/depth_example.cpp)
- [Access](src/color_depth_example.cpp#L179) the `rgb` field via [sensor_msgs::PointCloud2Iterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Iterator.html) to assign color information to each point in the pointcloud
- For valid pixels with non-zero depth value, [set the color value obtained from the input color image](src/color_depth_example.cpp#L202)
- For invalid pixels, [set color value 0](src/color_depth_example.cpp#L208)

## 4. Zero-copy communication within the same process using nodelets
In general, the amount of image and pointcloud data is large, so sending and receiving them between ROS nodes is a heavy load. For this reason, [roscpp](http://wiki.ros.org/roscpp) is equipped with a mechanism that does not copy data entities by exchanging pointers that point to them during topic communication within the same process ([see here](http ://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing)). The [nodelet](http://wiki.ros.org/nodelet) uses this to provide a foundation for loading multiple ROS nodes into a single process to greatly reduce the load of inter-node communication. Specifically, each node is implemented as a shared library, dynamically loaded into a pre-invoked `nodelet manager`, and allocated a separate thread for each node to run.

The sample program in this package corresponds to [nodelet](http://wiki.ros.org/nodelet) and is also an example of its use. The [Realsense](https://gitlab.com/art-aist/realsense-ros) and [PhoXi]() drivers also support [nodelet](http://wiki.ros.org/nodelet). If you run the driver and the sample program on the same host, you can realize zero-copy communication between them.

### 4.1 How to start as a nodelet
To invoke the sample program as [nodelet](http://wiki.ros.org/nodelet), enter the following.
```
$ roslaunch depth_camera_tutorial run_nodelet.launch manager:=manager prog:=[pointcloud_example|depth_example|color_depth_example] camera_name:=[realsense|phoxi]
```
The `nodelet manager` named ``manager`` is invoked, the driver and sample programs are loaded into it, and execution is started.

Note that it is also possible to start the nodes implemented as [nodelet](http://wiki.ros.org/nodelet) as individual processes. To do so, simply omit the `manager` specification above.
```
$ roslaunch depth_camera_tutorial run_nodelet.launch prog:=[pointcloud_example|depth_example|color_depth_example] camera_name:=[ realsense|phoxi]
```
If there is a bug in the implemented nodelet node, the whole `nodelet manager` may fall down, involving other nodes, so it is better to start them as separate processes when debugging.
### 4.2 Implementation of a ROS node as a nodelet
Using [pointcloud_example](src/pointcloud_example.cpp) as an example, we explain how to implement a ROS node as a [nodelet](http://wiki.ros.org/nodelet).
- [Include](src/pointcloud_example.cpp#L11-12) the header file required to use nodelet.
- [Define](src/pointcloud_example.cpp#L95) a ROS node of nodelet type as a derived class of [nodelet::Nodelet](http://docs.ros.org/en/noetic/api/nodelet/html/classnodelet_1_1Nodelet.html)
- [Implement](src/pointcloud_example.cpp#L106-111) `onInit()` member function of the class defined above so that the node is initialized with a node handle obtained from [nodelet::Nodelet::getPrivateNodeHandle()](http://docs.ros.org/en/noetic/api/nodelet/html/classnodelet_1_1Nodelet.html)
- [Export](src/pointcloud_example.cpp#L115-116) the defined class so that the nodelet can be loaded dynamically.

### 4.3 Build Configuration
- Set [CMakeLists.txt](CMakeLists.txt#L85-132) to build nodelet nodes as shared libraries.
- Set the path to the shared library to [nodelet_description.xml](nodelet_description.xml) to dynamically load nodelet nodes.
- Add package for using nodelet to [package.xml](package.xml#L17)
- Register the filename in [package.xml](package.xml#L18-20) to indicate that the path to the shared library is written in [nodelet_description.xml](nodelet_description.xml)
