#ifndef OCCLUSION_CALC_H
#define OCCLUSION_CALC_H
// 入出力機能に関する基本的な型や関数を使用する目的でヘッダーをインクルード
#include <iostream>
// ベクトル
#include <vector>
// 文字列
#include <string>
// 時間
#include <chrono>
// ROSに必要なヘッダー
#include <ros/ros.h>
// 物体検出された物体の位置と大きさ
#include <jsk_recognition_msgs/BoundingBox.h>
// BoundingBoxが検出された分だけ入っている
#include <jsk_recognition_msgs/BoundingBoxArray.h>
// 
#include <geometry_msgs/TwistStamped.h>
//

//

//
#include <tf/transform_listener.h>
// 
#include <vector_map/vector_map.h>
// GridMapクラスのROS版
#include <grid_map_ros/grid_map_ros.hpp>
// GridMapのヘッダー
#include <grid_map_msgs/GridMap.h>

#include <grid_map_cv/grid_map_cv.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "object_map_utils.hpp"

namespace object_map
{

	class WayareaToGrid
	{
	public:
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




double pos_y = obj_msg->objects.at(i).pose.position.y;