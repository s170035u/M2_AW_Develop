#include "occlusion_potential_field.h"
// コンストラクタ
OcclusionPotentialField::OcclusionPotentialField()
{
	ROS_INFO(" Constructer is up. ");	
}
// GridMap ＆ OccupancyGrid をパブリッシュ
void OcclusionPotentialField::PublishGridMap(grid_map::GridMap &input_grid_map, const std::string& input_layer_for_publish)
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
		publisher_grid_.publish(ros_gridmap_message);
		// パブリッシュする
		publisher_occupancy_.publish(ros_occupancygrid_message);
	}
	// 配信用のレイヤーがデータに含まれていない場合
	else
	{
		// デバック用
		ROS_INFO(" Empty GridMap. It might still be loading or it does not contain valid data.");
	}
}
// Road Occupancy Processorからメッセージを受け取った時に呼び出される：callback関数
// チェック完了
void OcclusionPotentialField::GridmapCallback(const grid_map_msgs::GridMap& input_grid_message)
{
	// メッセージを受け取るためのGridMap型変数
	grid_map::GridMap input_grid;
	// GridMap形式のROSメッセージをGridMapクラスで受け取る
	grid_map::GridMapRosConverter::fromMessage(input_grid_message, input_grid);
	// フレームid
	input_gridmap_frame_        = input_grid.getFrameId();
	// グリッドマップの長さ（X,Y方向）
	input_gridmap_length_       = input_grid.getLength();
	// Road Occupancy Processorで使用されている解像度を選択する
	input_gridmap_resolution_   = input_grid.getResolution();
	// グリッドマップの位置
	input_gridmap_position_     = input_grid.getPosition();
	// マップ設定
	if (!set_gridmap)
	{
	ROS_INFO("Created Map");
	// フレームIDセット（デフォルト：velodyne　←　WayAreaノードから引き継がれている）
	map_.setFrameId(input_gridmap_frame_);
	// ジオメトリ情報セット（） 
    map_.setGeometry(input_gridmap_length_,
	                     input_gridmap_resolution_,
	                     input_gridmap_position_);
	// デバック用					 
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
   	map_.getLength().x(), map_.getLength().y(),
   	map_.getSize()(0), map_.getSize()(1));	
	// 次のループでは処理しない
	// フラグ処理
	// occlusionという名前のレイヤーで初期化
	map_.add(output_layer_name_, 0.0);
	set_gridmap = true;
	ROS_INFO(" Map Set Done ");
	}
	// データをGridMapに格納していく
	ros::Time time = ros::Time::now();
	// OcclusionPotentialField用
	map_.add(output_layer_name_, 0.0);
	// オクルージョン領域を生み出しているポリゴン頂点の位置X
	std::vector<double> x_position_polygon_origin;
	// オクルージョン領域を生み出しているポリゴン頂点の位置Y
	std::vector<double> y_position_polygon_origin;
	// イテレータ用変数：現在のイテレータに対応する位置を格納する
	grid_map::Position pos_it;
	// オクルージョン領域の面積
	double occlusion_square_measure = 0.0;
	double max_square_measure = 0.0;
	double map_square_measure = map_.getLength().x() * map_.getLength().y() / 2 ;
	/* ******************************************************************************************
	 * ループ処理で・オクルージョン領域を生み出しているポリゴン頂点を割り出す
	 * 　　　　　　・オクルージョン領域の面積を計算する
	 * ------------------------------------------------------------------------------------------
	 * 改善案：自車位置より前にしかポリゴンは存在していないためSubmapIteratorで計算不可軽減
	 * **************************************************************************************** */
	for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
		// イテレータが指し示すセルの位置を格納する変数
		grid_map::Position position;
	 	// 変数positionにOcclusionPotentialFieldにおける現在のループでの位置を格納
      	map_.getPosition(*it, position);
	 	// サブスクライブした"/Occlusion"からオクルージョン領域を生み出しているポリゴン頂点を求める
	 	int cell_value_it = input_grid.atPosition(occlusion_layer_name_, position);
	  	// 値が77の時：オクルージョン領域を生み出しているポリゴン頂点
	  	if (cell_value_it == 77){
			  // ポリゴン頂点（X座標）を行列に追加していく
			  x_position_polygon_origin.push_back(position(0));
			  // ポリゴン頂点（Y座標）を行列に追加していく
			  y_position_polygon_origin.push_back(position(1));
			  // デバック用
			  ROS_INFO("OcclusionPotentialField Detected Occlusion Polygon (%lf , %lf)",position(0),position(1));
	  	}
		// 値が111の時，オクルージョン領域
	  	if (cell_value_it == 111){
			  // オクルージョン領域の面積を求める
			  occlusion_square_measure += std::pow(input_gridmap_resolution_ , 2.0);
		}
    }// GridMap Iterator
	/* ******************************************************************************************
	* Occlusion領域が存在している場合のみ、ポテンシャルフィールドが生成される
	* ------------------------------------------------------------------------------------------
	* 
	* **************************************************************************************** */
	if( !x_position_polygon_origin.empty() ) {
		/* ******************************************************************************************
	 	* ポテンシャルフィールド生成
	 	* ------------------------------------------------------------------------------------------
	 	* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 	* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 	* **************************************************************************************** */
		// オクルージョン領域の数だけループを回していく
		for(int i = 0; i != x_position_polygon_origin.size(); ++i){
			// GridMap Length で大きさを決定する
			grid_map::Length submap_length_double(fabs(x_position_polygon_origin[i])*2 , fabs(y_position_polygon_origin[i])*2 );
			// サブマップの大きさ
			grid_map::Size submap_size_int = (submap_length_double / map_.getResolution()).cast<int>();
			// オクルージョン領域を生み出すポリゴン頂点の位置
			grid_map::Position occlusion_polygon(x_position_polygon_origin[i] , y_position_polygon_origin[i]);
			// オクルージョン領域を生み出すポリゴン頂点のインデックス
			grid_map::Index occlusion_start_index;
			// インデックスを取得する
			map_.getIndex(occlusion_polygon , occlusion_start_index);
			/* ******************************************************************************************
	 		* サブマップ作成
	 		* ------------------------------------------------------------------------------------------
			* 物体が車両進行方向左側に存在する場合：イテレータは左から右に流れるため原点インデックスは"occlusion_start_index"
			* 物体が車両進行方向右側に存在する場合：イテレータは左から右に流れるため原点インデックスは"
			* 
			* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 		* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 		* **************************************************************************************** */
		 	// 物体が車両進行方向左側に存在する場合
			if (x_position_polygon_origin[i] > center_width_){

				// サブマップの開始点となるグリッドマップ上の位置
				grid_map::Position submap_start_position(x_position_polygon_origin[i] , -y_position_polygon_origin[i]);
				// サブマップの開始点となるグリッドマップ上のインデックス
				grid_map::Index submap_start_index;
				// インデックスを取得する
				map_.getIndex(submap_start_position , submap_start_index);
				// サブマップイテレータでオクルージョン領域から自車までの計算のみ行う
				for (grid_map::SubmapIterator iterator(map_, submap_start_index, submap_size_int);
     				!iterator.isPastEnd(); ++iterator) {
						// 現在ループにおける位置情報を取得
						map_.getPosition(*iterator, pos_it);
	 	 				/* ******************************************************************************************
	 					* 座標変換を行う
	 					* ------------------------------------------------------------------------------------------
						* Occlusion領域を検出したGridMap上の座標系から
						* ポテンシャルフィールド計算用のオクルージョン領域を生み出すポリゴン頂点をゼロ点とし
						* GridMap座標系を１８０度回転させた座標系へ変換する
						* 
						* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 					* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 					* **************************************************************************************** */
						// ポテンシャルフィールド座標系におけるX
						double x_it_occlusion_axis = x_position_polygon_origin[i] - pos_it(0);
						// ポテンシャルフィールド座標系におけるY
						double y_it_occlusion_axis = pos_it(1) - y_position_polygon_origin[i];
						// ポテンシャルフィールド計算
						map_.at(output_layer_name_, *iterator) = grid_max_value_ * occlusion_square_measure / std::exp(std::hypot(x_it_occlusion_axis,y_it_occlusion_axis));
						// 面積の
						
				}// grid_map::SubmapIterator 
			}// 物体が車両進行方向左側に存在する場合
			/* ******************************************************************************************
	 		* サブマップ作成
	 		* ------------------------------------------------------------------------------------------
			* 物体が車両進行方向左側に存在する場合：イテレータは左から右に流れるため原点インデックスは"occlusion_start_index"
			* 物体が車両進行方向右側に存在する場合：イテレータは左から右に流れるため原点インデックスは"
			* 
			* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 		* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 		* **************************************************************************************** */
			// 物体が車両進行方向右側に存在する場合
			if (x_position_polygon_origin[i] < -center_width_){

				// サブマップの開始点となるグリッドマップ上の位置
				grid_map::Position submap_start_position(-x_position_polygon_origin[i] , -y_position_polygon_origin[i]);
				// サブマップの開始点となるグリッドマップ上のインデックス
				grid_map::Index submap_start_index;
				// インデックスを取得する
				map_.getIndex(submap_start_position , submap_start_index);
				// サブマップイテレータでオクルージョン領域から自車までの計算のみ行う
				for (grid_map::SubmapIterator iterator(map_, submap_start_index, submap_size_int);
     				!iterator.isPastEnd(); ++iterator) {
						// 現在ループにおける位置情報を取得
						map_.getPosition(*iterator, pos_it);
	 	 				/* ******************************************************************************************
	 					* 座標変換を行う
	 					* ------------------------------------------------------------------------------------------
						* Occlusion領域を検出したGridMap上の座標系から
						* ポテンシャルフィールド計算用のオクルージョン領域を生み出すポリゴン頂点をゼロ点とし
						* GridMap座標系を１８０度回転させた座標系へ変換する
						* 
						* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 					* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 					* **************************************************************************************** */
						// ポテンシャルフィールド座標系におけるX
						double x_it_occlusion_axis = x_position_polygon_origin[i] - pos_it(0);
						// ポテンシャルフィールド座標系におけるY
						double y_it_occlusion_axis = pos_it(1) - y_position_polygon_origin[i] ;
						// ポテンシャルフィールド計算
						map_.at(output_layer_name_, *iterator) = grid_max_value_ * occlusion_square_measure / std::exp(std::hypot(x_it_occlusion_axis,y_it_occlusion_axis));
				}// grid_map::SubmapIterator 
			}// 物体が車両進行方向右側に存在する場合
			/* ******************************************************************************************
	 		* サブマップ作成
	 		* ------------------------------------------------------------------------------------------
			* 物体が車両前方にある場合：検出したオクルージョン領域を生み出すポリゴン頂点は物体の前方にある
		  	* 　　　　　　　　　　　　　
			* 前方にいると決めたY方向の幅だけサブマップを用意し，ポリゴン頂点からの距離に反比例して
			* ポテンシャルフィールドの大きさが変わるようにする
			* 
			* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 		* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 		* **************************************************************************************** */
			// 物体が車両進行方向前方に存在する場合
			if ( center_width_ > x_position_polygon_origin[i] && x_position_polygon_origin[i] > -center_width_){
				continue;
				// // サブマップの開始点となる位置
				// grid_map::Position position_submap_origin(x_position_polygon_origin[i] , center_width_/2);
				// // サブマップの開始点となるインデックス
				// grid_map::Index submap_start_index;
				// // インデックスを取得する
				// map_.getIndex(position_submap_origin , submap_start_index);
				// // サブマップの大きさ（Y方向のグリッドの数）を指定する
				// buffersize_y = static_cast<int>(center_width_ / input_gridmap_resolution_); 
				// // サブマップの大きさを設定する
				// grid_map::Index submap_buffer_size(buffersize_x , buffersize_y);
				// // サブマップイテレータでオクルージョン領域から自車までの計算のみ行う
				// for (grid_map::SubmapIterator iterator(map_, occlusion_start_index, submap_buffer_size);
     			// 	!iterator.isPastEnd(); ++iterator) {
				// 		// 現在ループにおける位置情報を取得
				// 		map_.getPosition(*iterator, pos_it);
	 	 		// 		/* ******************************************************************************************
	 			// 		* 座標変換を行う
	 			// 		* ------------------------------------------------------------------------------------------
				// 		* Occlusion領域を検出したGridMap上の座標系から
				// 		* ポテンシャルフィールド計算用のオクルージョン領域を生み出すポリゴン頂点をゼロ点とし
				// 		* GridMap座標系を１８０度回転させた座標系へ変換する
				// 		* 
				// 		* 改善案：ROSにはtfがあり使えば多くの機能を使えばコードを短くできるかもしれない
	 			// 		* →GridMapは２次元なので簡単に座標変換できるためここでは使用しない
	 			// 		* **************************************************************************************** */
				// 		// ポテンシャルフィールド座標系におけるX
				// 		double x_it_occlusion_axis = x_position_polygon_origin[i] - pos_it(0);
				// 		// ポテンシャルフィールド座標系におけるY
				// 		double y_it_occlusion_axis = y_position_polygon_origin[i] - pos_it(1);
				// 		// ポテンシャルフィールド計算
				// 		map_.at("occlusion_potential_field", *iterator) = occlusion_square_measure /  std::hypot(x_it_occlusion_axis,y_it_occlusion_axis);
			}// if 
		}// オクルージョン領域の数だけループを回していく
	}// オクルージョン領域が存在している場合if( !x_position_polygon_origin.empty() )
	// GridMapのタイムスタンプ
	map_.setTimestamp(time.toNSec());
	// occlusion計算後のGridMapをパブリッシュする：引数（配信用GridMap，配信用GridMapに）
	PublishGridMap(map_, output_layer_name_);
}//void OcclusionPotentialField::GridmapCallback

// mainプログラム：init()
void OcclusionPotentialField::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	// ！！！要チェック！！！
	std::string occlusion_topic_str;
	/* ******************************************************************************************
	 * パラメータ取得
	 * ------------------------------------------------------------------------------------------
	 * param（）はgetParam（）に似ていますが、パラメータを取得できなかった場合,
	 * デフォルト値を指定できます。
	 * コンパイラが文字列型のヒントを要求することがあります。
	 * **************************************************************************************** */
	// パラメータ取得：<arg name="occupancy_topic_src" default="occlusion" />
	// <!-- Road Occupancy Processor が配信しているトピック名 -->
	in_private_handle.param<std::string>("occlusion_topic_src", occlusion_topic_str, "occlusion");
	ROS_INFO("[%s] occlusion_topic_src: %s","occlusion_potential_field", occlusion_topic_str.c_str());
	// パラメータ取得：<arg name="occupancy_layer_name" default="occlusion" />
	// <!-- Road Occupancy Processor の GridMap クラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("occupancy_layer_name", occlusion_layer_name_, "occlusion");
	ROS_INFO("[%s] occupancy_layer_name: %s","occlusion_potential_field", occlusion_layer_name_.c_str());
    // パラメータ取得；<arg name="output_layer_name" default="road_occlusion" />
	// <!-- Occlusion が配信する GridMapクラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("output_layer_name", output_layer_name_, "occlusion_potential_field");
	ROS_INFO("[%s] output_layer_name: %s","occlusion_potential_field", output_layer_name_.c_str());
	// パラメータ取得：	<arg name="road_unknown_value" default="128" />   
	in_private_handle.param<int>("road_unknown_value", OCCUPANCY_ROAD_UNKNOWN, 128);
	ROS_INFO("[%s] road_unknown_value: %d","occlusion_potential_field", OCCUPANCY_ROAD_UNKNOWN);
	// パラメータ取得：<arg name="road_free_value" default="75" /> 
	in_private_handle.param<int>("road_free_value", OCCUPANCY_ROAD_FREE, 75);
	ROS_INFO("[%s] road_free_value: %d","occlusion_potential_field", OCCUPANCY_ROAD_FREE);
	// パラメータ取得：<arg name="road_occupied_value" default="0" />      
	in_private_handle.param<int>("road_occupied_value", OCCUPANCY_ROAD_OCCUPIED, 0);
	ROS_INFO("[%s] road_occupied_value: %d","occlusion_potential_field", OCCUPANCY_ROAD_OCCUPIED);
	// パラメータ取得：<arg name="no_road_value" default="255" />  
	in_private_handle.param<int>("no_road_value", OCCUPANCY_NO_ROAD, 255);
	ROS_INFO("[%s] no_road_value: %d","occlusion_potential_field", OCCUPANCY_NO_ROAD);
	/* ******************************************************************************************
	 * サブスクライバー宣言
	 * ------------------------------------------------------------------------------------------
	 * ① /occlusion ：オクルージョン領域が計算されたGridMap形式のメッセージトピック
	 * ② /occlusion_polygon :オクルージョン領域 
	 * **************************************************************************************** */
	// トピックを受け取った場合→&Occlusion::OccupancyCallback関数を呼び出す
	gridmap_subscriber_ = nh_.subscribe(occlusion_topic_str, 1, &OcclusionPotentialField::GridmapCallback, this);
	ROS_INFO("[%s] Subscribing to... /occlusion_potential_field","OcclusionPotentialField");
	// サブスクライバー宣言：トピック "DetectedObjects" を購読
	/* ******************************************************************************************
	 * パブリッシャー宣言
	 * ------------------------------------------------------------------------------------------
	 * ① /occlusion_potential_field ：オクルージョン領域が計算されたGridMap形式のメッセージトピック
	 * ② /occlusion_potential_polygon :オクルージョン領域 
	 * **************************************************************************************** */
	// オクルージョン領域を"occlusion_potential_field"という名前のトピックにgrid_map_msgs::GridMap形式のノードを送ることを伝える
	publisher_grid_ = nh_.advertise<grid_map_msgs::GridMap>("/occlusion_potential_field", 1, true);
	ROS_INFO("[%s] Publishing GridMap in gridmap_road_status","OcclusionPotentialFIeld");
	// オクルージョンではない領域を"occupancy_potential_field"という名前のトピックにros nav_msgs :: OccupancyGrid形式のノードを送ることを伝える
	publisher_occupancy_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_occlusion_potential_field", 1, true);
	ROS_INFO("[%s] Publishing Occupancy grid in occupancy_road_status","OccupancyPotentialField");	
}
// ノード実行関数
void OcclusionPotentialField::run()
{
	// 引数を呼び出すためのノードハンドラ
	ros::NodeHandle private_node_handle("~");
	// 初期化
	InitializeRosIo(private_node_handle);
	// ROS情報
	ROS_INFO("[%s] Ready. Waiting for data...","occlusion_potential_field");
	// ROS::spin関数
	ros::spin();
	// ROS 通知
	ROS_INFO("[%s] END","occlusion_potential_field");
}
