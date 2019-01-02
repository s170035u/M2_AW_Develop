/*******************************************************************************
* Autowareでポテンシャルフィールドを生成するためのプログラム
* 作成者： TAKASHI NOHARA
* 目的： Autoware上で提供されているポテンシャルフィールドを研究用に改造、効果検証する
* 使用法： README.mdを参照
* 参照： https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/
semantics/packages/object_map/nodes/potential_field/potential_field.cpp
*******************************************************************************/

//---------------->重要度の低い警告<----------------------

//>>>>>>>>>>>>>>>>>主要セクションヘッダ<<<<<<<<<<<<<<<<<<<

/*----------------------------------------------------------------------------*\
 *                                                                            *
\-----------------------------------------------------------------------------*/

/*
 * セクションの開始
 * ^^^^^^^^^^^^^^
 *
 * 以下の段落でこのセクションの目的とその仕組みを解説
 *
 */

/*
 * 中程度重要コメント
 * 10数行のコードについて解説
 * 言葉は**協調**できる
 */

// 行コメント

/*******************************************************************************
********************************************************************************
********************警告：本プログラムはAutowareに、2018/07/25********************
********************時点で
********************
********************
************************************************************
*******************************************************************************/


/*
* インクルード
* ^^^^^^^^^^^^^^
* 別ファイルの埋め込み
* <>：ツール側が提供してくれているファイルをインクルード、設定したフォルダ内のファイルを表示
* ""：自分で作ったファイルをインクルード、今のフォルダのファイル
*/
// tfを使って座標系フレームの変化を取得する
#include "tf/transform_listener.h"
//
#include <cmath>

#include <geometry_msgs/PointStamped.h>

#include <grid_map_msgs/GridMap.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <iostream>

#include <jsk_recognition_msgs/BoundingBox.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
// comp_tutrial/adder.h　adder.msgから生成されたメッセージを定義しているヘッダ
#include "autoware_msgs/DetectedObject.h"

#include "autoware_msgs/DetectedObjectArray.h"

#include <pcl_conversions/pcl_conversions.h>
// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>

#include <visualization_msgs/MarkerArray.h>
/*
* 名前空間
* ^^^^^^^^^^^^^^
* 名前の衝突を防ぐ
* 外から呼び出す際に先頭に名前空間の修飾をつける必要がある
* operator(::)を使って
*/
using namespace grid_map;
 /*
 * ポテンシャルフィールドClass
 * ^^^^^^^^^^^^^^
 * private：非公開部分：データメンバ
 * public：公開部分：メンバ関数
 */
 class PotentialField {
 private:
  // プロセスのノードへのハンドラを作成
  // 初めの NodeHandle は ノードの初期化を行い，最後のハンドラが破棄されるときに， ノードが使っていたリソースをすべてクリア
  ros::NodeHandle nh_;
  // Publisherとしての定義
  ros::Publisher publisher_;
  // 特定のトピックを購読対象として登録しメッセージを受信するノード
  ros::Subscriber waypoint_subscriber_;
  // 特定のトピックを購読対象として登録しメッセージを受信するノード
  ros::Subscriber vscan_subscriber_;
  // 特定のトピックを購読対象として登録しメッセージを受信するノード
  ros::Subscriber obj_subscriber_;
  // フラグ管理用論理型変数
  bool use_target_waypoint_;
  // フラグ管理用論理型変数
  bool use_obstacle_box_;
  // フラグ管理用論理型変数
  bool use_vscan_points_;
  // ポテンシャルフィールドマップの長辺長さ
  double map_x_size_;
  // ポテンシャルフィールドマップの短辺長さ
  double map_y_size_;
  // ポテンシャルフィールドマップの解像度
  double map_resolution_;
  // 車両先端から車両制御位置までの進行方向長さ
  double tf_x_;
  // 車両先端から車両制御位置までの垂直方向長さ
  double tf_z_;
  // 車両進行方向のオフセット：なぜ必要になるのか不明
  double map_x_offset_; //
  // GridMap クラスのオブジェクトmap_を作成
  GridMap map_; //
  /*
  * オブスタクルフィールドパラメータClass
  * ^^^^^^^^^^^^^^
  * private：非公開部分：データメンバ
  * public：公開部分：メンバ関数
  */
  //
  class ObstacleFieldParameter {
  public:
    ObstacleFieldParameter() : ver_x_p(0.9), ver_y_p(0.9) {}
    //
    double ver_x_p;
    double ver_y_p;
  };
  /*
  * ターゲットウェイポイントパラメータClass
  * ^^^^^^^^^^^^^^
  * private：非公開部分：データメンバ
  * public：公開部分：メンバ関数
  */
  class TargetWaypointFieldParamater {
  public:
    TargetWaypointFieldParamater() : ver_x_p(1.0), ver_y_p(1.0) {}
    double ver_x_p;
    double ver_y_p;
  };
  /*
  * VscanポイントフィールドパラメータClass
  * ^^^^^^^^^^^^^^
  * private：非公開部分：データメンバ
  * public：公開部分：メンバ関数
  */
  class VscanPointsFieldParamater {
  public:
    VscanPointsFieldParamater() : around_x(0.5), around_y(0.5) {}
    double around_x;
    double around_y;
  };
  /*
  * オブスタクルフィールドClass
  * ^^^^^^^^^^^^^^
  * private：非公開部分：データメンバ
  * public：公開部分：メンバ関数
  */
  //
  void obj_callback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg);
  //
  //
  void target_waypoint_callback(
      visualization_msgs::Marker::ConstPtr target_point_msgs);
  //
  void vscan_points_callback(sensor_msgs::PointCloud2::ConstPtr vscan_msg);
  //
  void publish_potential_field();
  /*
  * オブスタクルフィールドClass
  * ^^^^^^^^^^^^^^
  * private：非公開部分：データメンバ
  * public：公開部分：メンバ関数
  */
  public:
  PotentialField();
  void run();
  void init();
};// 終：ポテンシャルフィールドクラス

 // ポテンシャルフィールドクラスのポテンシャルフィールドコンストラクタ
 // 最初の頃、これが良く分からなくて調べてみると、
 // メンバー変数や親クラスを初期化するものだと分かりました。
PotentialField::PotentialField()
    : tf_x_(1.2), tf_z_(2.0),
      map_({"potential_field", "obstacle_field", "target_waypoint_field",
            "vscan_points_field"}) {
  // 第1に、roscppプログラム内で内部ノードのRAII形式の起動とシャットダウンを行います。
  // 第2に、サブコンポーネントの書き込みを容易にする名前空間解決の余分な層を提供します。
  // プライベートパラメータを読みだすためのノードハンドル
  ros::NodeHandle private_nh("~");
  // use_obstacle_box_フラグ管理
  if (!private_nh.getParam("use_obstacle_box", use_obstacle_box_)) {
    ROS_INFO("use obstacle_box");
    use_obstacle_box_ = true;
  }
  // use_vscan_points_フラグ管理
  if (!private_nh.getParam("use_vscan_points", use_vscan_points_)) {
    ROS_INFO("use vscan points");
    use_vscan_points_ = true;
  }
  // use_target_waypoint_フラグ管理
  if (!private_nh.getParam("use_target_waypoint", use_target_waypoint_)) {
    ROS_INFO("don't use target_waypoint");
    use_target_waypoint_ = false;
  }
  // map解像度
  if (!private_nh.getParam("map_resolution", map_resolution_)) {
    map_resolution_ = 0.25;
    ROS_INFO("map resolution %f", map_resolution_);
  }
  if (!private_nh.getParam("map_x_size", map_x_size_)) {
    map_x_size_ = 40.0;
    ROS_INFO("map x size %f", map_x_size_);
  }
  if (!private_nh.getParam("map_y_size", map_y_size_)) {
    map_y_size_ = 25.0;
    ROS_INFO("map y size %f", map_y_size_);
  }
  if (!private_nh.getParam("map_x_offset", map_x_offset_)) {
    map_x_offset_ = 10.0;
    ROS_INFO("map x offset %f", map_x_offset_);
  }
  // ノードはマスターに新しいメッセージを"/potential_field"というトピックに送ることを、
  publisher_ =
      nh_.advertise<grid_map_msgs::GridMap>("/potential_field", 1, true);
  // もし"obstacle_box"を使う場合
  if (use_obstacle_box_)
    //
    obj_subscriber_ = nh_.subscribe("/detected_objects", 1,
                                    &PotentialField::obj_callback, this);
  // もし"vcan_points"を使う場合
  if (use_vscan_points_)
    //
    vscan_subscriber_ = nh_.subscribe(
        "/vscan_points", 1, &PotentialField::vscan_points_callback, this);
  // もし"target_waypoint"を１11111]

  if (use_target_waypoint_)
    waypoint_subscriber_ =
        nh_.subscribe("/next_target_mark", 1,
                      &PotentialField::target_waypoint_callback, this);
}
// init()関数の内容
void PotentialField:
:init() {
  // c言語では，printf関数により文字を画面に出力しました．ROSではROS_INFO関数を使用します．
  ROS_INFO("Created map");
  // グリッドマップのフレームIDを設定します
  // パラメーター：frameId	設定するフレームID
  map_.setFrameId("/potential_field_link");
  // グリッドマップのジオメトリを設定します。すべてのデータをクリアします。
    //パラメーター:長さ	グリッドマップ[m]のx方向およびy方向の辺の長さ。
    //            :解決	セルサイズは[m /セル]です。
    //            :ポジション	グリッドマップフレーム[m]内のグリッドマップの2d位置。
  map_.setGeometry(Length(map_x_size_, map_y_size_), map_resolution_);
  // <初期文>：GridMapIterator it(map_)：
  // <条件>　：!it.isPastEnd()：
  // <反復分>：++it：
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    //　
    Position position;
    map_.getPosition(*it, position);
    map_.at("obstacle_field", *it) = 0.0;
    map_.at("target_waypoint_field", *it) = 0.0;
    map_.at("vscan_points_field", *it) = 0.0;
    map_.at("potential_field", *it) = 0.0;
  }
  // c言語では，printf関数により文字を画面に出力しました．ROSではROS_INFO関数を使用します
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
           map_.getLength().x(), map_.getLength().y(), map_.getSize()(0),
           map_.getSize()(1));
}
// コールバック関数を繰り返し呼び出す。whileループが必要な場合はspinOnce()を使う
void PotentialField::run() { ros::spin(); }
// パブリッシャー
void PotentialField::publish_potential_field() {
  // グリッドマップオブジェクトのすべてのレイヤーをROSグリッドマップメッセージに変換します。
     grid_map_msgs::GridMap message;
  // ポテンシャルフィールド＝オブスタクルフィールドとVscanポイントフィールドの間の最大値+ターゲットウェイポイントフィールド
  // Max. values between two layers  :  map["max"] = map["layer_1"].cwiseMax(map["layer_2"]);
  // オブスタクルフィールドとVscanポイントフィールドの間の最大値
  // ２つのレイヤーを足す。
  // map_["obstacle_field"].cwiseMax(map_["vscan_points_field"])：２つのレイヤーの最大値をとる。
  //
      map_["potential_field"] =
      map_["obstacle_field"].cwiseMax(map_["vscan_points_field"]) +
      map_["target_waypoint_field"];
  // Converts all layers of a grid map object to a ROS grid map message.
    //Parameters:
    //[in]	gridMap	the grid map object.
    //[out]	message	the grid map message to be populated.
  GridMapRosConverter::toMessage(map_, message);
  // void ros::Publisher::publish	(	const M & 	message	 ) 	const [inline]
    //Publish a message on the topic associated with this Publisher.
    publisher_.publish(message);
  // define ROS_INFO_THROTTLE_NAMED	(	rate,
    //name,
    //... 		 ) 	   ROS_LOG_THROTTLE(rate, ::ros::console::levels::Info, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                    message.info.header.stamp.toSec());
}
// callback関数：obstacle_boxの新しいメッセージが
void PotentialField::obj_callback(
    // Speed、yaw、yaw_rate、静的/動的クラスのような情報をDetectedObject msgに追加しました。
    autoware_msgs::DetectedObjectArray::ConstPtr obj_msg) { // Create grid map.
  //
  static ObstacleFieldParameter param;
  //
  double ver_x_p(param.ver_x_p);
  //
  double ver_y_p(param.ver_y_p);
  // ros :: Timeインスタンスとして現在の時刻を取得する：
  ros::Time time = ros::Time::now();
  //
  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
  //
    Position position;
    map_.getPosition(*it, position);
    map_.at("obstacle_field", *it) = 0.0;
    for (int i(0); i < (int)obj_msg->objects.size(); ++i) {
      double pos_x =
          obj_msg->objects.at(i).pose.position.x + tf_x_ - map_x_offset_;
      double pos_y = obj_msg->objects.at(i).pose.position.y;
      double len_x = obj_msg->objects.at(i).dimensions.x / 2.0;
      double len_y = obj_msg->objects.at(i).dimensions.y / 2.0;

      double r, p, y;
      tf::Quaternion quat(obj_msg->objects.at(i).pose.orientation.x,
                          obj_msg->objects.at(i).pose.orientation.y,
                          obj_msg->objects.at(i).pose.orientation.z,
                          obj_msg->objects.at(i).pose.orientation.w);
      tf::Matrix3x3(quat).getRPY(r, p, y);

      double rotated_pos_x = std::cos(-1.0 * y) * (position.x() - pos_x) -
                             std::sin(-1.0 * y) * (position.y() - pos_y) +
                             pos_x;
      double rotated_pos_y = std::sin(-1.0 * y) * (position.x() - pos_x) +
                             std::cos(-1.0 * y) * (position.y() - pos_y) +
                             pos_y;

      if (-0.5 < pos_x && pos_x < 4.0) {
        if (-1.0 < pos_y && pos_y < 1.0)
          continue;
      }
      if (pos_x - len_x < rotated_pos_x && rotated_pos_x < pos_x + len_x) {
        if (pos_y - len_y < rotated_pos_y && rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) =
              std::max(std::exp(0.0),
                       static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      } else if (rotated_pos_x < pos_x - len_x) {
        if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y - len_y < rotated_pos_y &&
                   rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x - len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      } else if (pos_x + len_x < rotated_pos_x) {
        if (rotated_pos_y < pos_y - len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y - len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y + len_y / 2.0 < rotated_pos_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_y - (pos_y + len_y)), 2.0) /
                           std::pow(2.0 * ver_y_p, 2.0))) +
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        } else if (pos_y - len_y < rotated_pos_y &&
                   rotated_pos_y < pos_y + len_y) {
          map_.at("obstacle_field", *it) = std::max(
              std::exp(
                  (-1.0 * (std::pow((rotated_pos_x - (pos_x + len_x)), 2.0) /
                           std::pow(2.0 * ver_x_p, 2.0)))),
              static_cast<double>(map_.at("obstacle_field", *it)));
        }
      }
    }
  }
  // Publish grid map.
  map_.setTimestamp(time.toNSec());
  publish_potential_field();
}

void PotentialField::target_waypoint_callback(
    visualization_msgs::Marker::ConstPtr target_point_msgs) {
  static TargetWaypointFieldParamater param;
  double ver_x_p(param.ver_x_p);
  double ver_y_p(param.ver_y_p);
  ros::Time time = ros::Time::now();
  geometry_msgs::PointStamped in, out;
  in.header = target_point_msgs->header;
  in.point = target_point_msgs->pose.position;
  tf::TransformListener tflistener;
  try {
    ros::Time now = ros::Time(0);
    tflistener.waitForTransform("/map", "/potential_field_link", now,
                                ros::Duration(10.0));
    tflistener.transformPoint("/potential_field_link", in.header.stamp, in,
                              "/map", out);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    Position position;
    map_.getPosition(*it, position);
    map_.at("target_waypoint_field", *it) = 0.0;
    map_.at("target_waypoint_field", *it) -=
        0.5 * std::exp((-1.0 * (std::pow((position.y() - (out.point.y)), 2.0) /
                                std::pow(2.0 * ver_y_p, 2.0))) +
                       (-1.0 * (std::pow((position.x() - (out.point.x)), 2.0) /
                                std::pow(2.0 * ver_x_p, 2.0))));
  }
  map_.setTimestamp(time.toNSec());
}

void PotentialField::vscan_points_callback(
    sensor_msgs::PointCloud2::ConstPtr vscan_msg) {
  static VscanPointsFieldParamater param;
  double around_x(param.around_x);
  double around_y(param.around_y);

  ros::Time time = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> pcl_vscan;
  pcl::fromROSMsg(*vscan_msg, pcl_vscan);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_vscan_ptr(
      new pcl::PointCloud<pcl::PointXYZ>(pcl_vscan));

  double length_x = map_.getLength().x() / 2.0;
  double length_y = map_.getLength().y() / 2.0;

  for (GridMapIterator it(map_); !it.isPastEnd(); ++it) {
    Position position;
    map_.getPosition(*it, position);
    map_.at("vscan_points_field", *it) = 0.0;
    for (int i(0); i < (int)pcl_vscan.size(); ++i) {
      double point_x = pcl_vscan.at(i).x - map_x_offset_;
      if (3.0 < pcl_vscan.at(i).z + tf_z_ || pcl_vscan.at(i).z + tf_z_ < 0.3)
        continue;
      if (length_x < point_x && point_x < -1.0 * length_x)
        continue;
      if (length_y < pcl_vscan.at(i).y && pcl_vscan.at(i).y < -1.0 * length_y)
        continue;
      if ((point_x + tf_x_) - around_x < position.x() &&
          position.x() < point_x + tf_x_ + around_x) {
        if (pcl_vscan.at(i).y - around_y < position.y() &&
            position.y() < pcl_vscan.at(i).y + around_y) {
          map_.at("vscan_points_field", *it) = 1.0; // std::exp(0.0) ;
        }
      }
    }
  }
  map_.setTimestamp(time.toNSec());
  publish_potential_field();
}
  // argcは引数の個数、argvには引数の実際の値が入る
int main(int argc, char **argv) {
  // 初期化のためのAPI
  // このノードは"potential_field"という名前であるという意味
  ros::init(argc, argv, "potential_field");
  //
  //
  // ②
  PotentialField potential_field;
  //
  //
  // ③
  potential_field.init();
  //
  //
  // ④
  potential_field.run();
  return 0;
}

//