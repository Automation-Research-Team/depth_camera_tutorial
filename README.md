depth_camera_tutorial
==================================================
## 概要
本パッケージは，深度カメラからの出力を処理するROSノードのサンプルコードを提供し，ROSで深度カメラを用いた点群処理を行うプログラムを開発するために必要な基礎知識を解説する．特に，以下の項目に重点を置く．
- 深度カメラの構造とその出力
- 深度カメラが出力するpointcloudをsubscribeし，それを構成する個々の3D点を処理する方法
- 深度カメラが出力する複数のtopicを，時刻同期を取りながらsubscribeする方法
- 深度カメラが出力する深度画像を3D点群に変換する方法
- 深度カメラが出力する深度画像とカラー画像を同時に扱う方法


## 1 深度カメラ
### 1.1 深度カメラの出力
多くの深度カメラは，深度センサとカラー（または濃淡）センサの両方を備える．通常両者は別のハードウェアである（[Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)など）が，同じハードウェアで兼ねる場合もある（[PhoXi](https://www.photoneo.com/phoxi-3d-scanner)など）．

深度カメラのROSドライバは，
- 深度/カラー画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)
- 深度/カラーセンサのカメラパラメータ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)型)
- 画像とカメラパラメータから計算された3D点から成るpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)

を出力する．
[message_filters](http://wiki.ros.org/message_filters)

### 1.2 深度値の型
深度カメラのROSドライバは，深度画像の各画素を`float`または[uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型で出力する．前者の単位はメートル，後者はミリメートルである．両者の区別は，[sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型メッセージの`encoding`フィールドに反映される．
- **encoding = sensor_msgs::image_encodings::TYPE_32FC1**: `float`型
- **encoding = sensor_msgs::image_encodings::TYPE_16UC1**: `uint16_t`型

深度値を`float`型で出力するドライバが大半であるが，[オリジナルのRealsense用ドライバ](https://github.com/IntelRealSense/realsense-ros)は`uint16_t`型で出力する．この時，最小単位が1mmとなるので，カメラと対象物体の距離が近い(接写)場合，深度値から復元された3D点群に量子化誤差による階段状のアーチファクトが生じることがある．これを防ぐため[float型で出力するように修正したドライバ](https://gitlab.com/art-aist/realsense-ros)があるので，接写する場合はこちらを使用することを薦める．

### 1.3 無効画素の扱い
一般に，深度カメラで取得した深度画像には，深度値が得られない無効画素が含まれる．片方のセンサからしか観測されない3D点（ステレオの場合）やプロジェクタの光が照射されない3D点（coded light patternの場合）などのオクルージョンによるもの，黒色物体など反射光の不足によるものが代表的である．このような無効画素は，トピックメッセージ上で次のように表現される([see here](https://ros.org/reps/rep-0118.html))．
- **pointcloud**([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型):x, y, z座標値に[NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html)を入れて無効画素を表す
- **深度画像**([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型): 深度値に`0`を入れて無効画素を表す
## 2 パッケージのビルド
まず`OpenCV`をインストールしてから次のようにダウンロード，ビルドする．
```
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist/depth_camera_tutorial
$ catkin build detph_camera_tutorial
```
サンプルプログラムを動かすためには，深度カメラとして[Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)の任意の機種またはPhotoneo社の任意の機種のいずれかが必要である．
## 3 サンプルプログラム
### 3.1 pointcloudトピックのsubscribeとその処理
`pointcloud_example`は，深度カメラからpointcloudを入力し，カラー情報を取り出して2次元画像として出力する．
- **入力トピック**: 深度カメラからのpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)
- **出力トピック**: pointcloud中の各点に付与されたカラー情報から成る2次元画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=pointcloud_example [camera_name:=realsense|phoxi]
```
プログラムの要点は，以下のとおりである．
- subscribeしたpointcloudの`height`が1でないことをチェックして，`organized` pointcloudであることを確認
- pointcloudの`fields`に`rgb`という名前のフィールドがあるかェックして，カラー情報を含むことを確認
- [sensor_msgs::PointCloud2ConstIterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2ConstIterator.html)を介してpointcloud中の3D点の`rgb`フィールドにアクセス
- `rgb`フィールド中のカラーコンポーネントの並びは，下位バイトから`b`, `g`, `r`の順であることに注意
- [image_transport](http://wiki.ros.org/image_transport)を用いて生成されたpublisherを介して，2次元カラー画像をpublish
- カラー画像のメンバ変数`_color`は，`sensor_msgs::Image`型ではなく，`sensor_msg::ImagePtr`型になっており，これは`boost::shared_ptr<sensor_msgs::Image>`の別名である．`shared_ptr`を介して画像のメモリ領域をheapから獲得することにより，同一プロセス内で画像をpublish/subscribeする時にserialize/deserializeを省略することができ，パフォーマンスが向上する([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))．