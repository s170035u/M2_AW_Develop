#include "occlusion.h"
// Publishのための関数
Occlusion::Occlusion()
{
	ROS_INFO(" Constructer is up. ");	
}
void Occlusion::PublishGridMap(grid_map::GridMap &input_grid_map, const std::string &input_layer_for_publish)
{
	// 配信用のレイヤーがデータに含まれている場合
	if (input_grid_map.exists(input_layer_for_publish))
	{
		// grid_map_msg/GridMap.msg型の変数
		grid_map_msgs::GridMap ros_gridmap_message;
		// nav_msgs/OccupancyGrid型の変数
		nav_msgs::OccupancyGrid ros_occupancygrid_message;		
		// GridMap型のROSトピックを生成する：引数（グリッドマップ，メッセージ）
		grid_map::GridMapRosConverter::toMessage(input_grid_map, ros_gridmap_message);
		// OccupancyGrid型のROSトピックを生成する：引数(グリッドマップ，レイヤー名，最小値，最大値，メッセージ)
		grid_map::GridMapRosConverter::toOccupancyGrid(input_grid_map,
		                                               input_layer_for_publish,
		                                               grid_min_value_,
		                                               grid_max_value_,
		                                               ros_occupancygrid_message);
		// パブリッシュする 
		publisher_grid_map_.publish(ros_gridmap_message);
		// パブリッシュする
		publisher_occupancy_grid_.publish(ros_occupancygrid_message);
	}
	// 配信用のレイヤーがデータに含まれていない場合
	else
	{
		// デバック用
		ROS_INFO(" Empty GridMap. It might still be loading or it does not contain valid data.");
	}
}
// 絶対値比較のための関数
static bool abs_compare(double a, double b)
{
    return (std::abs(a) < std::abs(b));
}
// Road Occupancy Processorからメッセージを受け取った時に呼び出される：callback関数
void Occlusion::OccupancyCallback(const grid_map_msgs::GridMap& input_grid_message)
{
	// メッセージを受け取るための変数
	grid_map::GridMap input_grid;
	// GridMap形式のROSメッセージをGridMapクラスで受け取る
	grid_map::GridMapRosConverter::fromMessage(input_grid_message, input_grid);
	// Road Occupancy Processorのグリッドマップデータをそのまま引用
	gridmap_.add("occupancy_road_status", input_grid.get(occupancy_layer_name_));
	// フレームid
	input_gridmap_frame_        = input_grid.getFrameId();
	// グリッドマップの長さ（X,Y方向）
	input_gridmap_length_       = input_grid.getLength();
	// Road Occupancy Processorで使用されている解像度を選択する
	input_gridmap_resolution_   = input_grid.getResolution();
	// グリッドマップの位置
	input_gridmap_position_     = input_grid.getPosition();
	// レイヤーとして追加する
}
// Detected Object メッセージを受け取った時に呼び出される：callback関数
void Occlusion::ObjectCallback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg)
{
	// gridmap_ に ”occupancy_road_status” レイヤーができるまで待つ
	if(!gridmap_.exists("occupancy_road_status")){
		return;
	}
	if (!set_map)
	{
	ROS_INFO("Created Map");
	// フレームIDセット（デフォルト：velodyne　←　WayAreaノードから引き継がれている）
	gridmap_.setFrameId(input_gridmap_frame_);
	// ジオメトリ情報セット（） 
    gridmap_.setGeometry(input_gridmap_length_,
	                     input_gridmap_resolution_,
	                     input_gridmap_position_);
	// デバック用					 
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
   	gridmap_.getLength().x(), gridmap_.getLength().y(),
   	gridmap_.getSize()(0), gridmap_.getSize()(1));	
	// 次のループでは処理しない
	set_map = true;
	}
	// データをGridMapに格納していく
	ros::Time time = ros::Time::now();
	// occlusionという名前のレイヤーで初期化
	gridmap_.add("occlusion", 0.0);
	// 検出した物体の数すべてに対して反復しオクルージョン領域を計算していく
	for (int i(0); i < (int)obj_msg->objects.size(); ++i) {
	// 物体の位置Xを求める
	double pos_x = obj_msg->objects.at(i).pose.position.x;
	// 自車進行方向に向かって前にある物体のみを考える→次の検出物体を見つける
	if (pos_x < 0)
	{
		continue;
	}
	// 物体の位置（高さ）Zを求める
	double pos_z = obj_msg->objects.at(i).pose.position.z;
	// 物体のZ方向長さを求める
	double len_z = obj_msg->objects.at(i).dimensions.z;
	// 物体がカメラより下の位置にある場合オクルージョン領域は生まれないとする
	if (pos_z + (len_z/2) < camera_height_)
	{
		continue;
	}
	// 物体のX方向長さを求める
    double len_x = obj_msg->objects.at(i).dimensions.x;
    // 物体の位置Yを求める  
	double pos_y = obj_msg->objects.at(i).pose.position.y;
	// 検出物体をEigen::Vector2d形式で利用するGridMapクラス"Position"に代入する
	grid_map::Position object_position(pos_x, pos_y);
	// "Position"の場所にある"occupancy_road_status"レイヤーの値を取得
	float gridcell_value = gridmap_.atPosition("occupancy_road_status",object_position);
	// std::out_of_range：指定した名前のレイヤーがない場合の例外処理がないと厳しいかも
	/* try {
                v.at(100) = 100;
        } catch (std::out_of_range& oor) {
                std::cerr << "Out of Range: " << oor.what() << std::endl;
        }
	*/
	// 検出した物体が専有領域にいない場合（＝道路でないところにある物体もしくはよくわからない物体）
	if (!gridcell_value == 0)
	{
		// ループを抜けだし，次の物体のループへ
		continue;
	}
	// 検出した物体が専有領域にいる場合
	if (gridcell_value == 0)
	{
		// 物体のY方向長さを求める
    	double len_y = obj_msg->objects.at(i).dimensions.y;
    	// 検出された物体のクオータニオンからRoll,Yaw,Pitchへの変換を求める
    	double r, p, y;
		// DetectedObjectメッセージからクオータニオン形式の回転表現を取得し，クオータニオンを定義する
		tf::Quaternion quat(obj_msg->objects.at(i).pose.orientation.x,
                        	obj_msg->objects.at(i).pose.orientation.y,
                        	obj_msg->objects.at(i).pose.orientation.z,
                        	obj_msg->objects.at(i).pose.orientation.w);
    	// 2次元で大きさを考えるため，地面に垂直方向軸による回転（yaw方向）を取得 
   		tf::Matrix3x3(quat).getRPY(r, p, y);
		// 物体のXY平面におけるX：頂点情報を配列にする
		std::vector<double> vertex_position_x(4);
		// 物体のXY平面におけるY：頂点情報を配列にする
		std::vector<double> vertex_position_y(4);
		// 物体の頂点と原点を通る直線の傾きを配列にする
		std::vector<double> slope_center_to_vertex(4);
		// 傾きが最大となるイテレータオブジェクトを定義する
		std::vector<double>::iterator max_slope_ite;
		// yaw方向の回転を使って物体の２次元上での位置(進行方向：左：上の位置)を取得する
    	vertex_position_x[0] = std::cos(y) * (len_x/2) - std::sin(y) * (len_y/2) + pos_x;
        vertex_position_y[0] = std::sin(y) * (len_x/2) + std::cos(y) * (len_y/2) + pos_y;
		// 原点と物体の角を通る直線の傾き
		slope_center_to_vertex[0] = vertex_position_x[0]/vertex_position_y[0];
		// yaw方向の回転を使って物体の２次元上での位置(進行方向：右：上の位置)を取得する
		vertex_position_x[1] = std::cos(y) * (len_x/2) - std::sin(y) * (-len_y/2) + pos_x;
        vertex_position_y[1] = std::sin(y) * (len_x/2) + std::cos(y) * (-len_y/2) + pos_y;
		// 原点と物体の角を通る直線の傾き
		slope_center_to_vertex[1] = vertex_position_x[1]/vertex_position_y[1];
		// yaw方向の回転を使って物体の２次元上での位置(進行方向：右：下の位置)を取得する
    	vertex_position_x[2] = std::cos(y) * (-len_x/2) - std::sin(y) * (-len_y/2) + pos_x;
        vertex_position_y[2] = std::sin(y) * (-len_x/2) + std::cos(y) * (-len_y/2) + pos_y;
		// 原点と物体の角を通る直線の傾き
		slope_center_to_vertex[2] = vertex_position_x[2]/vertex_position_y[2];
		// yaw方向の回転を使って物体の２次元上での位置(進行方向：左：下の位置)を取得する
    	vertex_position_x[3] = std::cos(y) * (-len_x/2) - std::sin(y) * (len_y/2) + pos_x;
        vertex_position_y[3] = std::sin(y) * (-len_x/2) + std::cos(y) * (len_y/2) + pos_y;
		// 原点と物体の角を通る直線の傾き
		slope_center_to_vertex[3] = vertex_position_x[3]/vertex_position_y[3];


/* ******************************************************************************************
* 物体がグリッドマップ領域にすべて入っているか確認する
* ------------------------------------------------------------------------------------------
* 物体がグリッドマップの外に要る場合は無視する
* 
* **************************************************************************************** */


		/* ******************************************************************************************
		 * 傾きの絶対値が一番大きい頂点を通る直線が，「オクルージョン領域」の境界を作る
		 * ------------------------------------------------------------------------------------------
		 * ここではまだ物体が自車の右側にいるか，左側にいるかに関わらず，計算できる
		 * 
		 * **************************************************************************************** */
		// 傾きの最大値が一番大きい頂点のiteratorオブジェクトを取得する
		max_slope_ite = std::max_element(slope_center_to_vertex.begin(), slope_center_to_vertex.end(), abs_compare);
		// iteratorオブジェクトから配列のインデックスを求める
		int max_index_vertex = std::distance(slope_center_to_vertex.begin(), max_slope_ite);
		// オクルージョン領域を作る物体の頂点の座標を求めるX
		double max_vertex_position_x = vertex_position_x[max_index_vertex];
		// オクルージョン領域を作る物体の頂点の座標を求めるY
		double max_vertex_position_y = vertex_position_y[max_index_vertex];
		// 傾きの最大値を求める
		double max_slope = *std::max_element(slope_center_to_vertex.begin(), slope_center_to_vertex.end(), abs_compare);
		// グリッドマップのセンター
		// Position Center_Grid 	= (0.0,0.0);
		// 原点のインデックスを定義
		// Index Index_origin;
		// 原点のインデックスを求める
		// gridmap_.getIndex(Center_Grid, Index_origin)

		// ポリゴンを定義
		grid_map::Polygon occlusion_polygon;
		// グリッドマップの縦横比を求める
		double aspect_ratio_gridmap = gridmap_.getLength().x()/gridmap_.getLength().y();
		// 物体が車両の右側にいる場合：傾きが負になっているはず
		if(-center_width_ > pos_y) {
			// ポリゴンのフレームIDをセット
			occlusion_polygon.setFrameId(gridmap_.getFrameId());
			// ポリゴン頂点を追加する（オクルージョン領域を作る頂点）
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			// 原点と物体の角を結ぶ直線がたどり着くGridMap境界を求めてポリゴンに追加する
			if (-max_slope < aspect_ratio_gridmap) {
			/* ******************************************************************************************
			 * 原点とオクルージョン領域を作り出す頂点を結ぶ直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * オクルージョン直線の傾き < グリッドマップの縦横比　→　ポリゴン頂点が３つの三角形
			 * オクルージョン直線の傾き > グリッドマップの縦横比　→　ポリゴン頂点が４つの四角形
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (-gridmap_.getLength().y() / 2 * max_slope) , (-gridmap_.getLength().y() / 2) ));
			/* ******************************************************************************************
			 * オクルージョン領域を作り出す頂点から引いたY軸に平行な直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * Xの値：オクルージョン領域を作り出す頂点と同じ
			 * Yの値：ー（グリッドマップのY軸の長さ÷２）と同じ
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (max_vertex_position_x) , (-gridmap_.getLength().y() /2 ) ));
			/* ******************************************************************************************
			 * 最後にオクルージョン領域を作り出す頂点をポリゴンに追加すればポリゴンが完成
			 * ------------------------------------------------------------------------------------------
			 * ３つの頂点を持つポリゴンとなる
			 * 最低４つのaddVertex関数が必要
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			} else {
			/* ******************************************************************************************
			 * 原点とオクルージョン領域を作り出す頂点を結ぶ直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * オクルージョン直線の傾き < グリッドマップの縦横比　→　ポリゴン頂点が３つの三角形
			 * オクルージョン直線の傾き > グリッドマップの縦横比　→　ポリゴン頂点が４つの四角形
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , (gridmap_.getLength().x() / 2 /max_slope) ));
			/* ******************************************************************************************
			 * グリッドマップの端を追加する
			 * ------------------------------------------------------------------------------------------
			 * X：X方向のグリッドマップの大きさ÷２
			 * Y：Y方向のグリッドマップの大きさ÷２×−１
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , (-gridmap_.getLength().y() / 2 ) ));
			/* ******************************************************************************************
			 * オクルージョン領域を作り出す頂点から引いたY軸に平行な直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * Xの値：オクルージョン領域を作り出す頂点と同じ
			 * Yの値：ー（グリッドマップのY軸の長さ÷２）と同じ
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (max_vertex_position_x) , (-gridmap_.getLength().y() /2 ) ));
			/* ******************************************************************************************
			 * 最後にオクルージョン領域を作り出す頂点をポリゴンに追加すればポリゴンが完成
			 * ------------------------------------------------------------------------------------------
			 * ３つの頂点を持つポリゴンとなる
			 * 最低４つのaddVertex関数が必要
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			}
		// 物体が車両の右側にいる場合：傾きが正になっているはず
		} else if ((center_width_ < pos_y)) {
			// ポリゴンのフレームIDをセット
			occlusion_polygon.setFrameId(gridmap_.getFrameId());
			// ポリゴン頂点を追加する（オクルージョン領域を作る頂点）
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			// 原点と物体の角を結ぶ直線がたどり着くGridMap境界を求めてポリゴンに追加する
			if (max_slope < aspect_ratio_gridmap) {
			/* ******************************************************************************************
			 * 原点とオクルージョン領域を作り出す頂点を結ぶ直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * オクルージョン直線の傾き < グリッドマップの縦横比　→　ポリゴン頂点が３つの三角形
			 * オクルージョン直線の傾き > グリッドマップの縦横比　→　ポリゴン頂点が４つの四角形
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().y() / 2 * max_slope) , (gridmap_.getLength().y() / 2) ));
			/* ******************************************************************************************
			 * オクルージョン領域を作り出す頂点から引いたY軸に平行な直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * Xの値：オクルージョン領域を作り出す頂点と同じ
			 * Yの値：ー（グリッドマップのY軸の長さ÷２）と同じ
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (max_vertex_position_x) , (gridmap_.getLength().y() /2 ) ));
			/* ******************************************************************************************
			 * 最後にオクルージョン領域を作り出す頂点をポリゴンに追加すればポリゴンが完成
			 * ------------------------------------------------------------------------------------------
			 * ３つの頂点を持つポリゴンとなる
			 * 最低４つのaddVertex関数が必要
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			} else {
			/* ******************************************************************************************
			 * 原点とオクルージョン領域を作り出す頂点を結ぶ直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * オクルージョン直線の傾き < グリッドマップの縦横比　→　ポリゴン頂点が３つの三角形
			 * オクルージョン直線の傾き > グリッドマップの縦横比　→　ポリゴン頂点が４つの四角形
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , (gridmap_.getLength().x() / 2 /max_slope) ));
			/* ******************************************************************************************
			 * グリッドマップの端を追加する
			 * ------------------------------------------------------------------------------------------
			 * X：X方向のグリッドマップの大きさ÷２
			 * Y：Y方向のグリッドマップの大きさ÷２×−１
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , (gridmap_.getLength().y() / 2 ) ));
			/* ******************************************************************************************
			 * オクルージョン領域を作り出す頂点から引いたY軸に平行な直線とグリッドマップ境界が交わる点を求めていく
			 * ------------------------------------------------------------------------------------------
			 * Xの値：オクルージョン領域を作り出す頂点と同じ
			 * Yの値：ー（グリッドマップのY軸の長さ÷２）と同じ
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( (max_vertex_position_x) , (gridmap_.getLength().y() /2 ) ));
			/* ******************************************************************************************
			 * 最後にオクルージョン領域を作り出す頂点をポリゴンに追加すればポリゴンが完成
			 * ------------------------------------------------------------------------------------------
			 * ３つの頂点を持つポリゴンとなる
			 * 最低４つのaddVertex関数が必要
			 * **************************************************************************************** */
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x , max_vertex_position_y ));
			}
		// 物体が車両の前方にいる場合：傾きは正と負でどちらも現れる
		} else {
			// ポリゴンのフレームIDをセット
			occlusion_polygon.setFrameId(gridmap_.getFrameId());
			// 傾きの最大値が一番大きい（正の大きさ）頂点のiteratorオブジェクトを取得する
			std::vector<double>::iterator max_slope_ite_positive = (slope_center_to_vertex.begin(), slope_center_to_vertex.end());
			// iteratorオブジェクトから配列のインデックスを求める
			int max_index_vertex_positive = std::distance(slope_center_to_vertex.begin(), max_slope_ite_positive);
			// オクルージョン領域を作る物体の頂点の座標を求めるX
			double max_vertex_position_x_positive = vertex_position_x[max_index_vertex_positive];
			// オクルージョン領域を作る物体の頂点の座標を求めるY
			double max_vertex_position_y_positive = vertex_position_y[max_index_vertex_positive];
			// 傾きの最大値が一番大きい（負の大きさ）頂点のiteratorオブジェクトを取得する
			std::vector<double>::iterator max_slope_ite_negative = std::min_element(slope_center_to_vertex.begin(), slope_center_to_vertex.end());
			// iteratorオブジェクトから配列のインデックスを求める
			int max_index_vertex_negative = std::distance(slope_center_to_vertex.begin(), max_slope_ite_negative);
			// オクルージョン領域を作る物体の頂点の座標を求めるX
			double max_vertex_position_x_negative = vertex_position_x[max_index_vertex_negative];
			// オクルージョン領域を作る物体の頂点の座標を求めるY
			double max_vertex_position_y_negative = vertex_position_y[max_index_vertex_negative];
			// まず車両から見えている物体の左側をポリゴンに追加する
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x_positive , max_vertex_position_y_positive ));
			// 次に車両から見えている物体の右側をポリゴンに追加する
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x_negative , max_vertex_position_y_negative ));
			// 右側から物体の奥（マップ境界）をポリゴンに追加する
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , max_vertex_position_y_negative ));
			// 左側から物体の奥（マップ境界）をポリゴンに追加する
			occlusion_polygon.addVertex(grid_map::Position( (gridmap_.getLength().x() / 2) , max_vertex_position_y_positive ));
			// 車両から見えている物体の左側をポリゴンに追加して終了
			occlusion_polygon.addVertex(grid_map::Position( max_vertex_position_x_positive , max_vertex_position_y_positive ));
		}
		bool isOcclusionOriginSet = false;
		for (grid_map::PolygonIterator iterator(gridmap_, occlusion_polygon); 
			!iterator.isPastEnd(); ++iterator) {
		  if (!isOcclusionOriginSet) {
		  gridmap_.at("occlusion", *iterator) = 77;
		  isOcclusionOriginSet = true;
		  }
		  // オクルージョン領域に値111を代入していく
		  gridmap_.at("occlusion", *iterator) = 111;
		}
		
	}// 検出した物体が専有領域にいる場合
	}// for (int i(0); i < (int)obj_msg->objects.size(); ++i) {
	/* ******************************************************************************************
     * ポリゴン
	 * ------------------------------------------------------------------------------------------
	 * 
	 * 
	 * **************************************************************************************** */
	// polygonをメッセージとして出力
	//geometry_msgs::PolygonStamped message;
	// 
  	//grid_map::PolygonRosConverter::toMessage(occlusion_polygon, polygon_message);
	// 
  	//polygonPublisher_.publish(polygon_message); 
	
	// occlusion計算後のGridMapをパブリッシュする：引数（配信用GridMap，配信用GridMapに）
	// タイムスタンプ：GridMapのタイムスタンプを
	gridmap_.setTimestamp(time.toNSec());

	// occlusion計算後のGridMapをパブリッシュする：引数（配信用GridMap，配信用GridMapに）
	PublishGridMap(gridmap_, output_layer_name_);
}


	/* ******************************************************************************************
	 * setGeometry
	 * ------------------------------------------------------------------------------------------
	 * void grid_map::GridMap::setGeometry	(	const Length & 	length,
	 * const double 	resolution,
	 * const Position & 	position = Position::Zero() 
	 * )		
	 * Set the geometry of the grid map. Clears all the data.
	 * Parameters
	 * length	the side lengths in x, and y-direction of the grid map [m].
	 * resolution	the cell size in [m/cell].
	 * position	the 2d position of the grid map in the grid map frame [m].
	 * Definition at line 51 of file GridMap.cpp.
	 * **************************************************************************************** */

	/* ******************************************************************************************
	 * isInside
	 * ------------------------------------------------------------------------------------------
	 * bool grid_map::GridMap::isInside	(	const Position & 	position	)	const
	 * Check if position is within the map boundaries.
	 * PointSta
	 * Parameters
	 * position	the position to be checked.
	 * Returns
	 * true if position is within map, false otherwise.
	 * Definition at line 237 of file GridMap.cpp.
	 * **************************************************************************************** */
	
	/* ******************************************************************************************
	 * atPosiinittion
	 * ------------------------------------------------------------------------------------------
	 * float & grid_map::GridMap::atPosition	(	const std::string & 	layer,
	 * const Position & 	position 
	 * )		
	 * Get cell data at requested position.
	 * Parameters
	 * layer	the name of the layer to be accessed.
	 * position	the requested position.
	 * Returns
	 * the data of the cell.
	 * Exceptions
	 * std::out_of_range	if no map layer with name layer is present.
	 * Definition at line 174 of file GridMap.cpp.
	 * 
	 * **************************************************************************************** */
  
	/* ******************************************************************************************
	 * add
	 * ------------------------------------------------------------------------------------------
	 * float & grid_map::GridMap::atPosition	(	const std::string & 	layer,
	 * const Position & 	position 
	 * )		
	 * Get cell data at requested position.
	 * Parameters
	 * layer	the name of the layer to be accessed.
	 * position	the requested position.
	 * Returns
	 * the data of the cell.
	 * Exceptions
	 * std::out_of_range	if no map layer with name layer is present.
	 * Definition at line 174 of file GridMap.cpp.
	 * 
	 * **************************************************************************************** */

	/* ******************************************************************************************
	 * bool grid_map::GridMap::getIndex	(	const Position & 	position,
	 * Index & 	index 
	 * )const
	 * Gets the corresponding cell index for a position.
	 * Parameters
	 * [in]	position	the requested position.
	 * [out]	index	the corresponding index.
	 * Returns
	 * true if successful, false if position outside of map.
	 * Definition at line 227 of file GridMap.cpp.
	 * **************************************************************************************** */

// mainプログラム：init()
void Occlusion::InitRosIo(ros::NodeHandle &in_private_handle)
{
	// road occupancy processor のトピック名
	std::string occupancy_topic_str;
	/* ******************************************************************************************
	 * パラメータ取得
	 * ------------------------------------------------------------------------------------------
	 * param（）はgetParam（）に似ていますが、パラメータを取得できなかった場合,
	 * デフォルト値を指定できます。
	 * コンパイラが文字列型のヒントを要求することがあります。
	 * **************************************************************************************** */
	// パラメータ取得：<arg name="occupancy_topic_src" defーault="gridmap_road_status" />
	// <!-- Road Occupancy Processor が配信しているトピック名 -->
	in_private_handle.param<std::string>("occupancy_topic_src", occupancy_topic_str, "gridmap_road_status");
	ROS_INFO("[%s] occupancy_topic_src: %s","occlusion", occupancy_topic_str.c_str());
	// パラメータ取得：<arg name="occupancy_layer_name" default="road_status" />
	// <!-- Road Occupancy Processor の GridMap クラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("occupancy_layer_name", occupancy_layer_name_, "road_status");
	ROS_INFO("[%s] occupancy_layer_name: %s","occlusion", occupancy_layer_name_.c_str());
    // パラメータ取得；<arg name="output_layer_name" default="road_occlusion" />
	// <!-- Occlusion が配信する GridMapクラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("output_layer_name", output_layer_name_, "occlusion");
	ROS_INFO("[%s] output_layer_name: %s","occlusion", output_layer_name_.c_str());
	// パラメータ取得：	<arg name="road_unknown_value" default="128" />   
	in_private_handle.param<int>("road_unknown_value", OCCUPANCY_ROAD_UNKNOWN, 128);
	ROS_INFO("[%s] road_unknown_value: %d","occlusion", OCCUPANCY_ROAD_UNKNOWN);
	// パラメータ取得：<arg name="road_free_value" default="75" /> 
	in_private_handle.param<int>("road_free_value", OCCUPANCY_ROAD_FREE, 75);
	ROS_INFO("[%s] road_free_value: %d","occlusion", OCCUPANCY_ROAD_FREE);
	// パラメータ取得：<arg name="road_occupied_value" default="0" />      
	in_private_handle.param<int>("road_occupied_value", OCCUPANCY_ROAD_OCCUPIED, 0);
	ROS_INFO("[%s] road_occupied_value: %d","occlusion", OCCUPANCY_ROAD_OCCUPIED);
	// パラメータ取得：<arg name="no_road_value" default="255" />  
	in_private_handle.param<int>("no_road_value", OCCUPANCY_NO_ROAD, 255);
	ROS_INFO("[%s] no_road_value: %d","occlusion", OCCUPANCY_NO_ROAD);
	// パラメータ取得：<arg name="camera_height" default="1.5" />	
	in_private_handle.param<float>("camera_height", camera_height_, 1.5);
	ROS_INFO("[%s] no_road_value: %d","occlusion", camera_height_);
	/* ******************************************************************************************
	 * サブスクライバー宣言
	 * ------------------------------------------------------------------------------------------
	 * １．検出され追跡されている物体情報をサブスクライブ
	 * ２．道路領域の区分けを示したGridMap情報をサブスクライブ
	 * **************************************************************************************** */
	// サブスクライバー宣言：Road Occupancy Grid ノードが配信するトピックを購読
	// トピックを受け取った場合→&Occlusion::OccupancyCallback関数を呼び出す
	gridmap_subscriber_ = node_handle_.subscribe(occupancy_topic_str, 1, &Occlusion::OccupancyCallback, this);
	ROS_INFO("[%s] Subscribing to... %s","occlusion", occupancy_topic_str.c_str());
	// サブスクライバー宣言：トピック "DetectedObjects" を購読
	// トピックを受け取った場合→&Occlusion::OccupancyCallback関数を呼び出す
	objects_subscriber_ = node_handle_.subscribe("/detected_objects", 1, &Occlusion::ObjectCallback, this);
	ROS_INFO("[%s] Subscribing to... /detected_objects","occlusion");
	/* ******************************************************************************************
	 * パブリッシャー宣言
	 * ------------------------------------------------------------------------------------------
	 * OccupancyGrid 形式
	 * GridMap 形式
	 * **************************************************************************************** */
	// パブリッシャー宣言：Occlusion ノードとして配信するトピックを登録
	publisher_grid_map_= node_handle_.advertise<grid_map_msgs::GridMap>("occlusion", 1);
	ROS_INFO("[%s] Publishing GridMap in gridmap_road_status","occlusion");
	// パブリッシャー宣言：Occlusion ノードとして配信するトピックを登録
	publisher_occupancy_grid_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_occlusion", 1);
	ROS_INFO("[%s] Publishing Occupancy grid in occupancy_road_status","occlusion");
	// デバック用
}

// ノード本体
void Occlusion::run()
{
	// 引数を呼び出すためのノードハンドラ
	ros::NodeHandle private_node_handle("~");
	// 
	tf::TransformListener transform_listener;
	// 
	transform_listener_ = &transform_listener;
	// 
	InitRosIo(private_node_handle);
	// 
	ROS_INFO("[%s] Ready. Waiting for data...","occlusion");
	// 
	ros::spin();
	// 
	ROS_INFO("[%s] END","occlusion");
}
//------------------------------------------------------------------------------------------------------------
/*
void RosRoadOccupancyProcessorApp::ConvertXYZIToRTZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                                    RosRoadOccupancyProcessorApp::PointCloudXYZIRTColor &out_organized_points,
                                                    std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                                    std::vector<RosRoadOccupancyProcessorApp::PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
	out_organized_points.resize(in_cloud->points.size());
	out_radial_divided_indices.clear();
	out_radial_divided_indices.resize(radial_dividers_num_);
	out_radial_ordered_clouds.resize(radial_dividers_num_);

	for(size_t i=0; i< in_cloud->points.size(); i++)
	{
		PointXYZIRT new_point;
		auto radius         = (float) sqrt(
				in_cloud->points[i].x*in_cloud->points[i].x
				+ in_cloud->points[i].y*in_cloud->points[i].y
		);
		auto theta          = (float) atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
		if (theta < 0){ theta+=360; }

		auto radial_div     = (size_t) floor(theta/radial_divider_angle_);
		auto concentric_div = (size_t) floor(fabs(radius/concentric_divider_distance_));

		new_point.point    = in_cloud->points[i];
		new_point.radius   = radius;
		new_point.theta    = theta;
		new_point.radial_div = radial_div;
		new_point.concentric_div = concentric_div;
		new_point.original_index = i;

		out_organized_points[i] = new_point;

		//radial divisions
		out_radial_divided_indices[radial_div].indices.push_back(i);

		out_radial_ordered_clouds[radial_div].push_back(new_point);

	}//end for

	//order radial points on each division
#pragma omp for
	for(size_t i=0; i< radial_dividers_num_; i++)
	{
		std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
		          [](const PointXYZIRT& a, const PointXYZIRT& b){ return a.radius < b.radius; });
	}
}
*/
//------------------------------------------------------------------------------------------------------------
/*
bool RosRoadOccupancyProcessorApp::LoadRoadLayerFromMat(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image)
{
	if (!in_grid_image.empty())
	{
		grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(in_grid_image,
		                                                                  output_layer_name_,
		                                                                  in_grid_map,
		                                                                  grid_min_value_,
		                                                                  grid_max_value_);

		return true;
	}
}
*/
//------------------------------------------------------------------------------------------------------------
/*
void Occlusion::Convert3dPointToOccupancy(grid_map::GridMap &input_grid_map, const geometry_msgs::Point &in_point, cv::Point &out_point)
{
	// calculate position
	grid_map::Position map_pos = input_grid_map.getPosition();
	double origin_x_offset = input_grid_map.getLength().x() / 2.0 - map_pos.x();
	double origin_y_offset = input_grid_map.getLength().y() / 2.0 - map_pos.y();
	// coordinate conversion for cv image
	out_point.x = (input_grid_map.getLength().y() - origin_y_offset - in_point.y) / input_grid_map.getResolution();
	out_point.y = (input_grid_map.getLength().x() - origin_x_offset - in_point.x) / input_grid_map.getResolution();
}
*/
//------------------------------------------------------------------------------------------------------------
/*
void RosRoadOccupancyProcessorApp::DrawLineInGridMap(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image,
                                                     const geometry_msgs::Point &in_start_point,
                                                     const geometry_msgs::Point &in_end_point, uchar in_value)
{
	cv::Point cv_start_point, cv_end_point;
	Convert3dPointToOccupancy(in_grid_map, in_start_point, cv_start_point);
	Convert3dPointToOccupancy(in_grid_map, in_end_point, cv_end_point);

	cv::Rect rect(cv::Point(), in_grid_image.size());

	if(!rect.contains(cv_start_point) || !rect.contains(cv_end_point))
	{
		return;
	}
	if (in_grid_image.at<uchar>(cv_start_point.y, cv_start_point.x) != OCCUPANCY_NO_ROAD
	    && in_grid_image.at<uchar>(cv_end_point.y, cv_end_point.x) != OCCUPANCY_NO_ROAD)
	{
		const int line_width = 3;
		cv::line(in_grid_image, cv_start_point, cv_end_point, cv::Scalar(in_value), line_width);
	}
}
*/
//------------------------------------------------------------------------------------------------------------
/*
void RosRoadOccupancyProcessorApp::SetPointInGridMap(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image,
                                                     const geometry_msgs::Point &in_point, uchar in_value)
{
	// calculate position
	cv::Point cv_point;
	Convert3dPointToOccupancy(in_grid_map, in_point, cv_point);

	cv::Rect rect(cv::Point(), in_grid_image.size());

	if(!rect.contains(cv_point))
		return;

	if (in_grid_image.at<uchar>(cv_point.y, cv_point.x) != OCCUPANCY_NO_ROAD)
	{
		const int radius = 2;
		const int fill = -1;
		cv::circle(in_grid_image, cv::Point(cv_point.x, cv_point.y), radius, cv::Scalar(in_value), fill);
	}
}
*/
//------------------------------------------------------------------------------------------------------------
/*
void RosRoadOccupancyProcessorApp::ConvertPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_pointcloud,
                                                     const std::string& in_targetframe,
                                                     pcl::PointCloud<pcl::PointXYZI>& out_pointcloud)
{
	//check that both pointcloud and grid are in the same frame, otherwise transform
	if (in_pointcloud.header.frame_id != in_targetframe)
	{
		ROS_INFO("transformPointCloud");
		tf::Transform map2sensor_transform = FindTransform(in_targetframe, in_pointcloud.header.frame_id);
		pcl_ros::transformPointCloud(in_pointcloud, out_pointcloud, map2sensor_transform);
	}
	else
velodyne
velodynein_pointcloud;
velodyne
};velodyne
*/
