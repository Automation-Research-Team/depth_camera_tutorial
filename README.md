depth_camera_tutorial
==================================================
## 概要
本パッケージは，ROSでdepthカメラを用いた点群処理を行うプログラムを開発するために必要な基礎知識を解説する．具体的には，以下の項目について説明し，その実装例としてdepthカメラからの出力を処理するROSノードのサンプルコードを提供する．
- depthカメラの構造とその出力
- depthカメラが出力するpointcloudをsubscribeし，それを構成する個々の3D点を処理する方法
- depthカメラが出力する複数のtopicを，時刻同期を取りながらsubscribeする方法
- depthカメラが出力するdepth画像を3D pointcloudに変換する方法
- depthカメラが出力するdepth画像とcolor画像を同時に扱う方法


## 1. depthカメラ
### 1.1 depthカメラの出力
多くのdepthカメラは，depthセンサとcolor（あるいはgrey）センサの両方を備える．通常両者は別のハードウェアである（[Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)など）が，同じハードウェアで兼ねる場合もある（[PhoXi](https://www.photoneo.com/phoxi-3d-scanner)など）．

depthカメラのROSドライバは，
- **depth画像**: [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型．カメラ中心から観測対象までの光軸に沿った距離を画素値とする二次元配列
- **depthセンサのカメラパラメータ**: [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)型．水平/垂直方向の焦点距離や画像主点など
- **color画像**: [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型．観測対象のカラーを画素値とする二次元配列
- **colorセンサのカメラパラメータ**: [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)型．水平/垂直方向の焦点距離や画像主点など
- **depth画像とカメラパラメータから計算された3D点から成るpointcloud**: [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型

を出力する．`sensor_msgs/PointCloud2`型のpointcloudの各点には，その3D座標値のほか，オプションとしてカラー値や法線を表す3Dベクトルを格納することができる．

かつてはpointcloudの型として[sensor_msgs/PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html)が使われていたが，現在はほとんど`sensor_msgs/PointCloud2`に移行している．

### 1.2 depth値の型
depthカメラのROSドライバは，depth画像の各画素を`float`または[uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型で出力し，前者の単位はメートル，後者はミリメートルである([see here](https://ros.org/reps/rep-0118.html))．両者の区別は，[sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型メッセージの`encoding`フィールドに反映される．
- encoding = **sensor_msgs::image_encodings::TYPE_32FC1**: `float`型
- encoding = **sensor_msgs::image_encodings::TYPE_16UC1**: `uint16_t`型

depth値を`float`型で出力するドライバが大半であるが，[オリジナルのRealsense用ドライバ](https://github.com/IntelRealSense/realsense-ros)は`uint16_t`型で出力する．この時，最小単位が1mmとなるので，カメラと対象物体の距離が近い(接写)場合，depth値から復元された3D点群に量子化誤差による階段状のアーチファクトが生じることがある．これを避けるために[float型で出力するように修正したドライバ](https://gitlab.com/art-aist/realsense-ros)があるので，接写する場合はこちらを使用することを薦める．

### 1.3 無効画素の扱い
一般に，depthカメラで取得したdepth画像には，depth値が得られない無効画素が含まれる．片方のセンサからしか観測されない3D点（ステレオの場合）やプロジェクタの光が照射されない3D点（coded light patternの場合）などのオクルージョンによるもの，黒色物体など反射光の不足によるものが代表的である．このような無効画素は，トピックメッセージ上で次のように表現される([see here](https://ros.org/reps/rep-0118.html))．
- **pointcloud**([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型):x, y, z座標値に[NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html)を入れて無効画素を表す
- **depth画像**([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型): depth値に`0`を入れて無効画素を表す

### 1.4 organized/unorganized pointcloud
`sensor_msgs/PointCloud2`型のpointcloudは，観測された全ての点を一次元配列に並べる`unorganized pointcloud`と，depthセンサの各画素に対応した二次元配列として表現する`organized pointcloud`の2つのフォーマットがある．前者は3D情報が得られなかった無効画素を含まないのでコンパクトであるが，画素間の隣接関係は失われる．後者は無効画素も記憶領域を消費するので全体の容量は大きくなるが，画素間の隣接関係が保存される．そのため，pointcloud中の任意の点に対してその近傍点を知ることは容易である．

両者の区別は，[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型メッセージの`height`, `width`, `is_dense`フィールドに反映される．
- **unorganized pointcloud**: `height` = 1, `width` = 点の個数, `is_dense` = true
- **organized pointcloud**: `height` = 二次元配列の行数, `width` = 二次元配列の列数, `is_dense` = false

[message_filters](http://wiki.ros.org/message_filters)

### 1.5 pointcloudとdepth画像のどちらを選ぶか？
複数のROSノード間でdepthデータを交換する場合，pointcloud形式とdepth画像形式のいずれかを選択できる．両者の得失は，以下のとおりである．

pointcloudの場合
- depth値からの3D座標の計算はカメラドライバに任せることができる
- `organized pointcloud`を選んだ場合，データ総量はdepth画像よりも大きくなり，通信の負担が増す
- カラー情報を含めることができるが，depth値のない無効画素におけるカラー値は失われる

一方，depth画像の場合
- ユーザプログラムの中でdepth値から3D座標を計算する必要がある
- depth画像とカメラパラメータの両方をやりとりする必要がある
- データ総量はpointcloudよりも小さく，通信の負担が軽い
- [image_transport](http://wiki.ros.org/image_transport)を使えば，画像情報を圧縮して通信の負担を軽減できる
- カラー情報が必要な場合は，カラー画像を別途やりとりする必要がある．
## 2. パッケージのビルド
まず`OpenCV`をインストールしてから次のようにダウンロード，ビルドする．
```
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist/depth_camera_tutorial
$ catkin build detph_camera_tutorial
```
サンプルプログラムを動かすためには，depthカメラとして[Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)の任意の機種またはPhotoneo社の任意の機種のいずれかが必要である．
## 3. サンプルプログラム
### 3.1 pointcloudトピックのsubscribeとその処理
[pointcloud_example](src/pointcloud_example.cpp)は，depthカメラからpointcloudを入力し，color情報を取り出して2次元画像として出力する．
- **入力トピック**: depthカメラからのpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)
- **出力トピック**: pointcloud中の各点に付与されたcolor情報から成る2次元画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=pointcloud_example [camera_name:=realsense|phoxi]
```
プログラムの要点は，以下のとおりである．
- subscribeしたpointcloudの`height`が`1`でないことを[チェック](src/pointcloud_example.cpp#L50)して，*organized* pointcloudであることを確認
- pointcloudの`fields`に`rgb`という名前のフィールドがあるか[チェック](src/pointcloud_example.cpp#L57-59)して，color情報を含むことを確認
- [sensor_msgs::PointCloud2ConstIterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2ConstIterator.html)を介してpointcloud中の3D点の`rgb`フィールドにアクセス([see code](src/pointcloud_example.cpp#L76))
- `rgb`フィールド中のcolorコンポーネントの並びは，下位バイトから`b`, `g`, `r`の順であることに注意([see code](src/pointcloud_example.cpp#L84-86))
- [image_transport](http://wiki.ros.org/image_transport)を用いて生成された[publisher](src/pointcloud_example.cpp#L41-42)を介して，2次元color画像を[publish](src/pointcloud_example.cpp#L94)
- color画像を表す変数`color`は，`sensor_msgs::Image`型ではなく，`sensor_msg::ImagePtr`型になっており([see code](src/pointcloud_example.cpp#L66))，これは`boost::shared_ptr<sensor_msgs::Image>`の別名である．`shared_ptr`を介して画像のメモリ領域をheapから獲得することにより，同一プロセス内で画像をpublish/subscribeする時にserialize/deserializeを省略することができ([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))，パフォーマンスが向上する．
- `shared_ptr`を介して保持されたカラー画像の内容をpublish後に変更することはできない([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))．そのため，変数`color`が指すメモリ領域は，各フレーム毎にheapから獲得しなければならない．

### 3.2 depth画像トピックのsubscribeと3D座標値の計算
[depth_example](src/depth_example.cpp)は，depthカメラからdepth画像とカメラパラメータを入力し，各画素の3D座標を計算してpointcloudとして出力する．
- **入力トピック**: depthカメラからのdepth画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)とそのカメラパラメータ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html)型)
- **出力トピック**: depth画像とカメラパラメータから計算されたpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=depth_example [camera_name:=realsense|phoxi]
```
プログラムの要点は，以下のとおりである．

## 4. nodeletを用いた同一プロセス内でのゼロコピー通信