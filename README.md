depth_camera_tutorial
==================================================
## 概要
本パッケージは，深度カメラからの出力を処理するROSノードのサンプルコードを提供し，ROSで深度カメラを用いた点群処理を行うために必要な基礎知識を解説する．特に，以下の項目に重点を置く．
- 深度カメラの構造とその出力
- 深度カメラが出力するpointcloudをsubscribeし，それを構成する個々の3D点を処理する方法
- 深度カメラが出力する複数のtopicを，時間的同期を取りながらsubscribeする方法
- 深度カメラが出力する深度画像を3D点群に変換する方法
- 深度カメラが出力する深度画像とカラー画像を同時に扱う方法

## 深度カメラの構造とその出力
[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html), 
[sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html), 
[sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html),
[image_transport](http://wiki.ros.org/image_transport)
[message_filters](http://wiki.ros.org/message_filters)

## 本パッケージのインストール

OpenCVをインストールした後，次のようにビルドする．
```
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist/depth_camera_tutorial
$ catkin build detph_camera_tutorial
```

## サンプルプログラムの使用法
3つのサンプルプログラムがある．
### pointcloud_example

