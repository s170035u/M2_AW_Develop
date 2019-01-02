/*******************************************************************************************
 * パブリック情報をヘッダーファイルにまとめる
 * ・モジュールの機能とユーザーが利用できるものを明確に説明するコメント部分
 * ・外部からアクセスできるクラス定義
 * ・
 * 
 * 
 * 
 * 
 * 
 * 
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

	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
			sensor_msgs::PointCloud2> SyncPolicyT;

	message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_ground_subscriber_, *cloud_no_ground_subscriber_;
	//message_filters::Subscriber<grid_map_msgs::GridMap>     *gridmap_subscriber_;
	ros::Subscriber                                         gridmap_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

	size_t                              radial_dividers_num_;

	const double                        radial_divider_angle_   = 0.1;
	const double                        concentric_divider_distance_ = 1;

	const int                           grid_min_value_         = 0;
	const int                           grid_max_value_         = 255;

	int                                 OCCUPANCY_ROAD_UNKNOWN  = 128;
	int                                 OCCUPANCY_ROAD_FREE     = 75;
	int                                 OCCUPANCY_ROAD_OCCUPIED = 0;
	int                                 OCCUPANCY_NO_ROAD       = 255;

	struct PointXYZIRT
	{
		pcl::PointXYZI  point;

		float           radius;       //cylindric coords on XY Plane
		float           theta;        //angle deg on XY plane

		size_t          radial_div;  //index of the radial divsion to which this point belongs to
		size_t          concentric_div;//index of the concentric division to which this points belongs to

		size_t          original_index; //index of this point in the source pointcloud
	};
	typedef std::vector<PointXYZIRT> PointCloudXYZIRTColor;

	/*!
	 *
	 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
	 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
	 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
	 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
	 */
	void ConvertXYZIToRTZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
	                      PointCloudXYZIRTColor& out_organized_points,
	                      std::vector<pcl::PointIndices>& out_radial_divided_indices,
	                      std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);

	/*!
	 * Resets road layer with in_grid_image
	 * @param in_grid_map Map to replace
	 * @param in_grid_image Image containing the desired data in the layer
	 * @return True if replacement was possible. False otherwise
	 */
	bool LoadRoadLayerFromMat(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image);

	/*!
	 * Publishes the GridMap object contained in the current instance
	 * @param[in] GridMap to Publish
	 * @param[in] Name of the layer to publish as OccupancyGrid
	 */
	void PublishGridMap(grid_map::GridMap& in_grid_map, const std::string& in_layer_publish);

	void Convert3dPointToOccupancy(grid_map::GridMap& in_grid_map,
	                               const geometry_msgs::Point& in_point,
	                               cv::Point& out_point);

	/*!
	 * Draws a line between two points in the bitmap
	 * @param in_grid_map GridMap object to modify
	 * @param in_grid_image Bitmap on which to draw the line
	 * @param in_start_point Initial point of the line
	 * @param in_end_point Final point of the line
	 * @param in_value Valid valid to assign to the points in the line
	 */
	void DrawLineInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_start_point,
	                       const geometry_msgs::Point& in_end_point,
	                       uchar in_value);

	/*!
	 * Draws a point in the occupancy grid
	 * @param in_grid_map GridMap object to modify
	 * @param in_grid_image Bitmap on which to draw the point
	 * @param in_point Point where to draw the point
	 * @param in_value Value to set in the whole point
	 */
	void SetPointInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_point,
	                       uchar in_value);

	/*!
	 * Receives the GridMap message and extract its geometry, occupancy bitmap
	 * @param in_message Received message
	 */
	void GridMapCallback(const grid_map_msgs::GridMap& in_message);
	/*!
	 * Receives 2 synchronized point cloud messages. in_ground_cloud_msg contains the points classified externally as
	 * ground, while in_no_ground_cloud_msg contains the points classified beloging to obstacle above the ground.
	 * @param[in] in_ground_cloud_msg Message containing pointcloud classified as ground.
	 * @param[in] in_no_ground_cloud_msg Message containing pointcloud classified as obstacle.
	 * @param[in] in_gridmap_msg Message containing the GridMap object with the road areas defined.
	 */
	void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_ground_cloud_msg,
	                    const sensor_msgs::PointCloud2::ConstPtr& in_no_ground_cloud_msg);

	/*!
	 * Obtains parameters from the command line, initializes subscribers and publishers.
	 * @param in_private_handle Ros private handle to get parameters for this node.
	 */
	void InitializeRosIo(ros::NodeHandle& in_private_handle);

	/*!
	 * Searches for the transformation between in_source_frame and in_target_frame in the TF tree
	 * @param in_target_frame Target Frame name
	 * @param in_source_frame Source Frame name
	 * @return The Transform, if any, to convert from in_source_frame to in_target_frame
	 */
	tf::StampedTransform FindTransform(const std::string& in_target_frame, const std::string& in_source_frame);

	/*!
	 * Transforms a pointcloud if the target frame is different
	 * @param in_pointcloud PointCloud to convert
	 * @param in_targetframe Target frame
	 * @param out_pointcloud Output pointcloud, if frame is the same, no transformation will be performed.
	 */
	void ConvertPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_pointcloud,
	                       const std::string& in_targetframe,
	                       pcl::PointCloud<pcl::PointXYZI>& out_pointcloud);
	/*!
	 * Transforms a Point using the specified transform
	 * @param in_point Point to transform
	 * @param in_transform Transformation data
	 * @return The transformed point
	 */
	geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_transform);

public:
	void Run();

	RosRoadOccupancyProcessorApp();
};
