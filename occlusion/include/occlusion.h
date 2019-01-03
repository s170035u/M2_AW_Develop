/*******************************************************************************************
 * パブリック情報をヘッダーファイルにまとめる
 * ・モジュールの機能とユーザーが利用できるものを明確に説明するコメント部分
 * ・外部からアクセスできるクラス定義
 * ・
 ********************************************************************************************/
// ヘッダーファイルは定数やデータ構造を定義するのに役立つ
#ifndef OCCLUSION_CALC_H
// 同じファイルを複数回インクルードしないようにシンボルを定義するようにする
#define OCCLUSION_CALC_H
// 入出力機能に関する基本的な型や関数を使用する目的でヘッダーをインクルード
#include <iostream>
// ベクトル
#include <vector>
// 文字列
#include <string>
// 時間
#include <chrono>
// 計算
#include <cmath>
// ROSに必要なヘッダー
#include <ros/ros.h>
// 物体検出された物体の位置と大きさ
#include <jsk_recognition_msgs/BoundingBox.h>
// BoundingBoxが検出された分だけ入っている
#include <jsk_recognition_msgs/BoundingBoxArray.h>
// これは自由空間での速度をその直線部分と角度部分に分けて表します。
// 参照座標フレームとタイムスタンプのねじれ
#include <geometry_msgs/TwistStamped.h>
// pointcloud と poinycloud2 の橋渡しを行う。
#include <sensor_msgs/point_cloud_conversion.h>
// ポイントクラウド
#include <sensor_msgs/PointCloud.h>
// ポイントクラウド２
#include <sensor_msgs/PointCloud2.h>
// PCLデータ型およびROSメッセージ型からの変換を提供します
#include <pcl_conversions/pcl_conversions.h>
// ポイントクラウド２
#include <pcl/PCLPointCloud2.h>
// ポイントクラウド変換
#include <pcl_ros/transforms.h>
// ポイントクラウド
#include <pcl_ros/point_cloud.h>
// ポイントクラウドの型
#include <pcl/point_types.h>
// フィルタが満たす必要のある条件に基づいて、メッセージを取り込み、
// 後でそれらのメッセージを出力することができるメッセージフィルタのセット。
// Subscriberフィルタは、他のフィルタのソースを提供するROSサブスクリプションの単なるラッパーです。 
// 購読者フィルタは、他のフィルタの出力に接続することはできません。代わりに、入力としてROSトピックを使用します。
#include <message_filters/subscriber.h>
// TimeSynchronizerフィルタは、受信チャネルをそのヘッダに含まれているタイムスタンプで同期させ、同じ数のチャネルを
//受け取る単一のコールバックの形式でそれらを出力します。 C ++の実装は、最大9つのチャネルを同期できます。
#include <message_filters/synchronizer.h>
// タイムスタンプのおおよそ一致を確認
#include <message_filters/sync_policies/approximate_time.h>
// グリッドマップライブラリ用のROSインタフェース。複数のデータレイヤを持つ2次元グリッドマップを管理します。
#include <grid_map_ros/grid_map_ros.hpp>
// バッグファイルからグリッドマップを読み込んで公開
#include <grid_map_msgs/GridMap.h>
// グリッドマップとOpenCV画像間の変換
#include <grid_map_cv/grid_map_cv.hpp>
// tf
#include <tf/tf.h>
// ADASマップを扱う：Autoware定義
#include <vector_map/vector_map.h>
// 座標変換ライブラリ
#include <tf/transform_listener.h> 
// object_map パッケージの共有ファイル
#include "object_map_utils.hpp"
// これは、参照座標フレームとタイムスタンプを持つPointを表します。
#include <geometry_msgs/PointStamped.h>
// DetectedObject 型メッセージクラス
#include "autoware_msgs/DetectedObject.h"
// DetectedObject 型の配列
#include "autoware_msgs/DetectedObjectArray.h"
// 基本立体をrvizに送るためのvisualization_msgs/Markerメッセージ
#include <visualization_msgs/Marker.h>
// visualization_msgs/Markerメッセージの配列
#include <visualization_msgs/MarkerArray.h>

#endif
// これはROS ImageメッセージとOpenCVイメージを変換するCvBridgeを含みます。
// #include <cv_bridge/cv_bridge.h>

namespace object_map
{

	class WayareaToGrid
	{
	public:配列
		WayareaToGrid();

		void Run();

	private:
		// ノードのハンドラを定義：Pablish時に使用する
        // 最初：ノードの初期化＋最後：破壊時ノードが使っていたリソースを破壊
		ros::NodeHandle         node_handle_;
        // ノードのハンドラを定義：初期化時に使用する
        // 最初：ノードの初期化＋最後：破壊時ノードが使っていたリソースを破壊
		ros::NodeHandle         private_node_handle_;
        // パブリッシャー宣言：grid_map形式のmsgを配信するためのPublisher
        // ①トピックに対してメッセージを発信できるpublish()メソッドを含む
        // ② スコープから出ると自動的に発信しなくなる
		ros::Publisher          publisher_grid_map_;
        // パブリッシャー宣言：ros nav_msgs :: OccupancyGrid形式のmsgを配信するためのPublisher
        // ①トピックに対してメッセージを発信できるpublish()メソッドを含む
        // ② スコープから出ると自動的に発信しなくなる
		ros::Publisher          publisher_occupancy_;

		grid_map::GridMap       gridmap_;

		std::string             sensor_frame_;
		std::string             map_frame_;

		const std::string       grid_layer_name_ = "wayarea";

		double                  grid_resolution_;
		double                  grid_length_x_;
		double                  grid_length_y_;
		double                  grid_position_x_;
		double                  grid_position_y_;

		tf::TransformListener   tf_listener_;

		int                     OCCUPANCY_ROAD      = 128;
		int                     OCCUPANCY_NO_ROAD   = 255;
		const int               grid_min_value_     = 0;
		const int               grid_max_value_     = 255;

		std::vector<std::vector<geometry_msgs::Point>> area_points_;

		/*!
		 * Initializes Ros Publisher, Subscribers and sets the configuration parameters
		 */
		void InitializeRosIo();


	};

}  // namespace object_map

#endif  // WAYAREA_TO_GRID

class Occlusion
{
	// ノードハンドラ
	ros::NodeHandle                     node_handle_;
	// パブリッシャー：navmsg
	ros::Publisher                      publisher_occupancy_grid_;
	// パブリッシャー：gridmap
	ros::Publisher                      publisher_grid_map_;
	// GridMapクラス
	grid_map::GridMap                   gridmap_;
	// 座標変換
	tf::TransformListener*              transform_listener_;
	// Road Occupancy Processor が格納されているレイヤーの中身 
	std::string                         occupancy_layer_name_;
	// Occlusion情報を格納するGidMapレイヤーの名前
	std::string                         output_layer_name_;
	// Grid Map フレーム
	std::string                         input_gridmap_frame_;
	// Grid Map 解像度
	double                              input_gridmap_resolution_;
	// Grid Map 長さ(typedef Eigen::Array2d)
	grid_map::Length                    input_gridmap_length_;
	// Grid Map 位置(typedef Eigen::Vector2d)
	grid_map::Position                  input_gridmap_position_;
	// Grid Map 形式のメッセージをサブスクライブする
	ros::Subscriber                                         gridmap_subscriber_;
	// Detected Object 形式のメッセージをサブスクライブする
	ros::Subscriber                                         objects_subscriber_;
	// Occupancy Processor のグリッドマップ設定
	const int                           grid_min_value_         = 0;
	const int                           grid_max_value_         = 255;
	int                                 OCCUPANCY_ROAD_UNKNOWN  = 128;
	int                                 OCCUPANCY_ROAD_FREE     = 75;
	int                                 OCCUPANCY_ROAD_OCCUPIED = 0;
	int                                 OCCUPANCY_NO_ROAD       = 255;
	/*!
	 * 引数in_grid_imageで道路レイヤをリセットします
	 * @param in_grid_map 置き換えるマップ
	 * @param in_grid_image レイヤーに必要なデータを含む画像
	 * @return 交換が可能であれば真。 それ以外の場合はfalse
	 */
	bool LoadRoadLayerFromMat(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image);
	/*!
	 * 現在のインスタンスに含まれているGridMapオブジェクトを公開します。
	 * @param[in] 公開するGridMap
	 * @param[in] OccupancyGridとして公開するレイヤーの名前
	 */
	void PublishGridMap(grid_map::GridMap& in_grid_map, const std::string& in_layer_publish);
	// なんだろう
	void Convert3dPointToOccupancy(grid_map::GridMap& in_grid_map,
	                               const geometry_msgs::Point& in_point,
	                               cv::Point& out_point);
	/*!
	 * ビットマップ内の2点間に線を引きます
	 * @param in_grid_map 変更するGridMapオブジェクト
	 * @param in_grid_image 線を描画するビットマップ
	 * @param in_start_point 線の始点
	 * @param in_end_point 線の終点
	 * @param in_value 行内の点に割り当てるためのValid
	 */
	void DrawLineInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_start_point,
	                       const geometry_msgs::Point& in_end_point,
	                       uchar in_value);

	/*!
	 * 占有グリッドに点を描画します
	 * @param in_grid_map 変更するGridMapオブジェクト
	 * @param in_grid_image 点を描画するビットマップ
	 * @param in_point pointポイントを描画するポイント
	 * @param in_value 全体に設定する値
	 */
	void SetPointInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_point,
	                       uchar in_value);

	/*!
	 * コールバック関数
	 * GridMapメッセージを受け取り、その形状、占有ビットマップを抽出します
	 * @param in_message 受信したメッセージ
	 */
	void GridMapCallback(const grid_map_msgs::GridMap& in_message);
	/*!
	 * コールバック関数
	 * DetectedObjectArrayメッセージを受け取り、その位置、姿勢、大きさを抽出する
	 * @param in_message 受信したメッセージ
	 */
	void ObjectCallback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg);

	/*!
	 * コマンドラインからパラメータを取得し、購読者と発行者を初期化します。
	 * in_private_handleこのノードのパラメータを取得するためのRosプライベートハンドル。
	 */
	void InitializeRosIo(ros::NodeHandle& in_private_handle);

	/*!
	 * TFツリーのin_source_frameとin_target_frameの間の変換を検索する
	 * @param in_target_frameターゲットフレーム名
	 * @param in_source_frameソースフレーム名
	 * @return in_source_frameからin_target_frameに変換する変換がある場合は、それを返します。
	 */
	tf::StampedTransform FindTransform(const std::string& in_target_frame, const std::string& in_source_frame);

	/*!
	 * 指定された変換を使ってPointを変換する
	 * @param in_point 変換するポイント
	 * @param in_transform 変換データ
	 * @return 返還後の点
	 */
	geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_transform);

public:
	 void init();
	 void run();
	Occlusion();
};
