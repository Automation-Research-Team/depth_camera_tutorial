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

を出力する．[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型のpointcloudの各点には，その3D座標値のほか，オプションとしてカラー値や法線を表す3Dベクトルを格納することができる．

かつてはpointcloudの型として[sensor_msgs/PointCloud](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud.html)が使われていたが，現在はほとんど[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)に移行している．

### 1.2 depth値の型
depthカメラのROSドライバは，depth画像の各画素を`float`または[uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型で出力し，前者の単位はメートル，後者はミリメートルである([see here](https://ros.org/reps/rep-0118.html))．両者の区別は，[sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型メッセージの`encoding`フィールドに反映される．
- encoding = **sensor_msgs::image_encodings::TYPE_32FC1**: `float`型
- encoding = **sensor_msgs::image_encodings::TYPE_16UC1**: [uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型

depth値を`float`型で出力するドライバが大半であるが，[オリジナルのRealsense用ドライバ](https://github.com/IntelRealSense/realsense-ros)は[uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型で出力する．この時，最小単位が1mmとなるので，カメラと対象物体の距離が近い(接写)場合，depth値から復元された3D点群に量子化誤差による階段状のアーチファクトが生じることがある．これを避けるために[float型で出力するように修正したドライバ](https://gitlab.com/art-aist/realsense-ros)があるので，接写する場合はこちらを使用することを薦める．

### 1.3 無効画素の扱い
一般に，depthカメラで取得したdepth画像には，depth値が得られない無効画素が含まれる．無効画素が生じる原因としては，片方のセンサからしか観測されない3D点（ステレオの場合）やプロジェクタの光が照射されない3D点（coded light patternの場合）などのオクルージョン，黒色物体など反射光の不足等が代表的である．

無効画素は，トピックメッセージ上で次のように表現される([see here](https://ros.org/reps/rep-0118.html))．
- **pointcloud**: [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型．x, y, z座標値に[NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html)を入れて無効画素を表す
- **depth画像**: [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型．depth値に`0`を入れて無効画素を表す

### 1.4 organized/unorganized pointcloud
[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型のpointcloudは，観測された全ての点を一次元配列に並べる`unorganized pointcloud`と，depthセンサの各画素に対応した二次元配列として表現する`organized pointcloud`の2つのフォーマットがある．前者は3D情報が得られなかった無効画素を含まないのでコンパクトであるが，画素間の隣接関係は失われる．後者は無効画素も記憶領域を消費するので全体の容量は大きくなるが，画素間の隣接関係が保存されるので，pointcloud中の任意の点に対してその近傍点を知ることは容易である．

両者の区別は，[sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型メッセージの`height`, `width`, `is_dense`フィールドに反映される．
- **unorganized pointcloud**: `height` = 1, `width` = 点の個数, `is_dense` = true
- **organized pointcloud**: `height` = 二次元配列の行数, `width` = 二次元配列の列数, `is_dense` = false


### 1.5 pointcloudとdepth画像のどちらを選ぶか？
複数のROSノード間でdepthデータを交換する場合，pointcloud形式とdepth画像形式のいずれかを選択できる．両者の得失は，以下のとおりである．

pointcloudの場合
- depth値からの3D座標の計算はカメラドライバに任せることができる
- `organized pointcloud`を選んだ場合，データ総量はdepth画像よりも大きくなり，通信の負担が増す
- pointcloud中の各点にカラー情報を含めることができるが，depth値のない無効画素におけるカラー値は失われる

depth画像の場合
- ユーザプログラムの中でdepth値から3D座標を計算する必要がある
- depth画像とカメラパラメータの両方を送受信する必要がある
- データ総量はpointcloudよりも小さく，通信の負担が軽い
- [image_transport](http://wiki.ros.org/image_transport)を使えば，画像情報を圧縮して通信の負担を軽減できる
- カラー情報が必要な場合は，別途カラー画像を送受信する必要がある．
## 2. パッケージのビルド
まず`OpenCV`をインストールしてから次のようにダウンロード，ビルドする．
```
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist/depth_camera_tutorial
$ catkin build detph_camera_tutorial
```
サンプルプログラムを動かすためには，depthカメラとして[Realsense](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html)の任意の機種またはPhotoneo社の任意の機種のいずれかが必要である．
## 3. サンプルプログラム
### 3.1 pointcloud_example
[pointcloud_example](src/pointcloud_example.cpp)は，pointcloudに含まれる各3D点の情報を取り出す方法を示すサンプルプログラムである．具体的には，depthカメラからpointcloudを入力し，color情報を取り出して2次元画像として出力する．
- **入力トピック**: depthカメラからのpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)
- **出力トピック**: pointcloud中の各点に付与されたcolor情報から成る2次元画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=pointcloud_example camera_name:=[realsense|phoxi]
```
プログラムの要点は，以下のとおりである．
- subscribeしたpointcloudの`height`が`1`でないことを[チェック](src/pointcloud_example.cpp#L45)して，*organized* pointcloudであることを確認
- pointcloudの`fields`に`rgb`という名前のフィールドがあるか[チェック](src/pointcloud_example.cpp#L52-54)して，color情報を含むことを確認
- [sensor_msgs::PointCloud2ConstIterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2ConstIterator.html)を介してpointcloud中の3D点の`rgb`フィールドにアクセス([see code](src/pointcloud_example.cpp#L71))
- `rgb`フィールド中のcolorコンポーネントの並びは，下位バイトから`b`, `g`, `r`の順であることに注意([see code](src/pointcloud_example.cpp#L79-81))
- [image_transport](http://wiki.ros.org/image_transport)を用いて生成された[publisher](src/pointcloud_example.cpp#L36-37)を介して，2次元color画像を[publish](src/pointcloud_example.cpp#L89)
- color画像を表すローカル変数`color`は，`sensor_msgs::Image`型ではなく，`sensor_msg::ImagePtr`型になっており([see code](src/pointcloud_example.cpp#L61))，これは`boost::shared_ptr<sensor_msgs::Image>`の別名である．`shared_ptr`を介して画像のメモリ領域をheapから獲得することにより，同一プロセス内で画像をpublish/subscribeする時にserialize/deserializeを省略することができ([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))，通信の負担が軽減する．
- `shared_ptr`を介して保持されたカラー画像の内容をpublish後に変更してはならない([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))．そのため，変数`color`が指すメモリ領域は，各フレーム毎にheapから獲得しなければならない．

### 3.2 depth_example
[depth_example](src/depth_example.cpp)は，depth画像の各画素を3D点に変換する方法を示すサンプルプログラムである．具体的には，depthカメラからdepth画像とカメラパラメータを入力し，各画素の3D座標を計算してpointcloudとして出力する．
- **入力トピック**: depthカメラからのdepth画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)とそのカメラパラメータ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html)型)
- **出力トピック**: 2つの入力トピックから計算されたpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=depth_example camera_name:=[realsense|phoxi]
```

subscribeされるdepth画像とカメラパラメータの2つのトピックは，時間的に同期している必要がある．一般に，ROSにおいて複数のトピックを同期させた上でsubscribeするには
[message_filters](http://wiki.ros.org/message_filters)を使う．しかし，[sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型と[sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html)型の2つトピックをsubscribeする場合は，[image_transport](http://wiki.ros.org/image_transport)パッケージに含まれる[CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html)を使用するのが便利なので，ここではそれを用いて実装している．

[CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html)は，depth画像に限らず，一般のモノクロ/カラー画像をカメラパラメータと共にsubscribeする時に使用される．

プログラムの要点は，以下のとおりである．
- [image_transport](http://wiki.ros.org/image_transport)を用いて[CameraSubscriber](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html)を生成([see code](src/depth_example.cpp#49-51))．カメラパラメータのトピック名が指定されていないが，画像トピックが属する名前空間にある`camera_info`というトピックが自動的にsubscribeされる．
- 入力depth画像の`encoding`フィールドからdepth値の型を判定し，それに応じてpointcloud生成のメンバ関数を呼び出し，結果をpublish([see code](src/depth_example.cpp#60-68))
- 出力pointcloudの領域を確保し，そのサイズを[設定](src/depth_example.cpp#81-86)．`shared_ptr`を介してメモリ領域をheapから獲得する理由は[pointcloud_example](src/pointcloud_example.cpp)と同様
- [sensor_msgs::PointCloud2Modifier](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Modifier.html)によってpointcloud中の各点が持つ情報を指定する．これによってpointcloudの内部バッファ領域が確保されるとともに，それにアクセスするための情報がセットされる．ここでは3D座標値のみを指定している([see code](src/depth_example.cpp#92-96))
- ある画素(u, v)におけるdepth値から3D座標を計算するには，まず(u, v)からレンズ歪を取り除き，さらに画像主点を原点とし焦点距離が`1`である`canonical image coordinates`(x, y)に変換することが必要である．これは，`OpenCV`の[cv::undistortPoints()](https://docs.opencv.org/4.4.0/d9/d0c/group__calib3d.html#ga55c716492470bfe86b0ee9bf3a1f0f7e)を用いて実現する
- そのために，入力depth画像の1行毎に画素座標の配列`uv`を[作り](src/depth_example.cpp#114-115)，カメラの内部パラメータ行列`K`とレンズ歪パラメータ`D`とともに[cv::undistortPoints()に渡して](src/depth_example.cpp#119)`canonical image coordinates`の配列`xy`を計算する
- pointcloud中の各点に3D座標を与えるために，[sensor_msgs::PointCloud2Iterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Iterator.html)を介して`x`, `y`, `z`フィールドにアクセスする([see code](src/depth_example.cpp#111))
- depth値が[uint16_t](https://cpprefjp.github.io/reference/cstdint/uint16_t.html)型の場合は[メートル単位に直す](src/depth_example.cpp#125)
- `xy`にdepth値を乗じて3D点のx, y座標を計算する．z座標はdepth値をそのまま用いる([see code](src/depth_example.cpp#129-131))
- depth値が`0`の場合は無効画素なので，座標値に[NaN](https://cpprefjp.github.io/reference/limits/numeric_limits/quiet_nan.html)を入れる([see code](src/depth_example.cpp#135-136))

### 3.3 color_depth_example
[color_depth_example](src/color_depth_example.cpp)は，color画像とdepth画像を融合して各画素をカラー情報付きの3D点に変換する方法を示すサンプルプログラムである．具体的には，depthカメラからcolor画像，depth画像およびカメラパラメータを入力し，各画素の3D座標を計算するとともにそれにカラー値を付与してカラー情報付きpointcloudとして出力する．
- **入力トピック**: depthカメラからのcolor画像とdepth画像([sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)型)および後者のカメラパラメータ([sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msgCameraInfo.html)型)
- **出力トピック**: 3つの入力トピックから計算されたカラー情報付きpointcloud([sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html)型)

次のように起動する．
```
$ roslaunch depth_camera_tutorial run.launch prog:=color_depth_example [camera_name:=realsense|phoxi]
```

ここでは，同期したカラー画像，depth画像およびカメラパラメータの3つのtopicをsubscribeする必要があるため，[message_filters](http://wiki.ros.org/message_filters)パッケージに含まれる[message_filters::TimeSynchronizer< M0, M1, M2, M3, M4, M5, M6, M7, M8 >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html)を使う．また，カメラパラメータは[message_filters::Subscriber< M >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html)によってsubscribeする．color画像とdepth画像もこれを用いてsubscribeできるが，[image_transport::SubscriberFilter](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html)を使うと，画像圧縮により通信の負担を軽減する[image_transport](http://wiki.ros.org/image_transport)の機能を享受できる．

プログラムの要点は，以下のとおりである．
- カラー画像とdepth画像のsubscriberを[image_transport::SubscriberFilter](http://docs.ros.org/en/noetic/api/image_transport/html/classimage__transport_1_1SubscriberFilter.html)型で[定義](src/color_depth_example.cpp#54-55)
- カメラパラメータのsubscriberを[message_filters::Subscriber< M >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1Subscriber.html)型で[定義](src/color_depth_example.cpp#56)
- 3つの入力トピックを同期させる`synchronizeer`([message_filters::TimeSynchronizer< M0, M1, M2, M3, M4, M5, M6, M7, M8 >](http://docs.ros.org/en/noetic/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html)型)を[定義](src/color_depth_example.cpp#57)
- `synchronizer`に同期された3つの入力トピックに対するコールバック関数を[設定](src/color_depth_example.cpp#70)
- depth値から3D座標を計算する方法は[depth_example](src/depth_example.cpp)と同様
- pointcloud中の各点にカラー情報を与えるために，[sensor_msgs::PointCloud2Iterator< T >](http://docs.ros.org/en/melodic/api/sensor_msgs/html/classsensor__msgs_1_1PointCloud2Iterator.html)を介して`rgb`フィールドにアクセスする([see code](src/color_depth_example.cpp#140))
- depth値が`0`でない有効画素には[入力カラー画像から得た値を設定](src/color_depth_example.cpp#163-165)
- 無効画素には[カラー値0を設定](src/color_depth_example.cpp#171)

## 4. nodeletを用いた同一プロセス内でのゼロコピー通信
一般に画像やpointcloudのデータ量は大きいので，それらをROSノード間で送受信するのは重い負荷となる．そのため，[roscpp](http://wiki.ros.org/roscpp)には，同一プロセス内でのtopic通信においてデータの受け渡しをそれを指すポインタのやりとりで済ませ，データ実体をコピーしない仕組みが備えられている([see here](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Intraprocess_Publishing))．[nodelet](http://wiki.ros.org/nodelet)はこれを利用して，複数のROSノードを単一のプロセスにロードしてノード間通信の負荷を大幅に軽減するための基盤を提供する．具体的には，各ノードを共有ライブラリとして実装し，予め起動しておいた`nodelet manager`に動的にロードして各々に個別のスレッドを割り当てて走らせる．

本パッケージのサンプルプログラムは[nodelet](http://wiki.ros.org/nodelet)に対応しており，その使用例にもなっている．[Realsense](https://gitlab.com/art-aist/realsense-ros)や[PhoXi]()のドライバも[nodelet](http://wiki.ros.org/nodelet)に対応しているので，同一ホストでドライバとサンプルプログラムを動かせば，両者の間でゼロコピー通信が実現できる．

### 4.1 nodeletとしての起動方法
[nodelet](http://wiki.ros.org/nodelet)としてサンプルプログラムを起動するには，次を入力する．
```
$ roslaunch depth_camera_tutorial run_nodelet.launch manager:=manager prog:=[pointcloud_example|depth_example|color_depth_example] camera_name:=[realsense|phoxi]
```
`manager`という名前の`nodelet manager`が起動され，それにドライバとサンプルプログラムがロードされて実行が開始される．

なお，[nodelet](http://wiki.ros.org/nodelet)として実装されているノードを個別のプロセスとして起動することもできる．そのためには，上記の`manager`の指定を省略すれば良い．
```
$ roslaunch depth_camera_tutorial run_nodelet.launch prog:=[pointcloud_example|depth_example|color_depth_example] camera_name:=[realsense|phoxi]
```
実装されたnodeletノードにバグがあると，他のノードを巻き込んで`nodelet manager`ごと落ちることがあるので，デバッグ時は個別のプロセスとして起動した方が良い．
### 4.2 ROSノードのnodeletとしての実装
[pointcloud_example](src/pointcloud_example.cpp)を例にして，[nodelet](http://wiki.ros.org/nodelet)としてROSノードを実装する方法を説明する．
- nodeletを使うためのヘッダファイルを[インクルード](src/pointcloud_example.cpp#11-12)
- nodeletとしてのROSノードを[nodelet::Nodelet](http://docs.ros.org/en/noetic/api/nodelet/html/classnodelet_1_1Nodelet.html)の派生クラスとして[定義](src/pointcloud_example.cpp#95)
- 定義したクラスの`onInit()`メンバ関数を，[nodelet::Nodelet::getPrivateNodeHandle()](http://docs.ros.org/en/noetic/api/nodelet/html/classnodelet_1_1Nodelet.html)から得たノードハンドルを用いてROSノードを初期化するように[実装](src/pointcloud_example.cpp#106-111)
- 定義したクラスが動的にロードできるよう，必要な情報を[export](src/pointcloud_example.cpp#115-116)

### 4.3 ビルド設定
- nodeletノードを共有ライブラリとしてビルドするよう，[CMakeLists.txtに設定](CMakeLists.txt#85-132)
- nodeletノードを動的にロードするために，共有ライブラリへのパスを[nodelet_description.xml](nodelet_description.xml)に設定
- nodeletを使うためのパッケージを[package.xmlに追加](package.xml#17)
- 共有ライブラリへのパスが[nodelet_description.xml](nodelet_description.xml)に書かれていることを示すために，そのファイル名を[package.xmlに登録](package.xml#18-20)