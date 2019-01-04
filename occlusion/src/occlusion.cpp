#include "wayarea2grid.h"

namespace object_map
{

	WayareaToGrid::WayareaToGrid() :
			private_node_handle_("~")
	{
		InitializeRosIo();
		LoadRoadAreasFromVectorMap(private_node_handle_, area_points_);
	}


	void WayareaToGrid::InitializeRosIo()
	{
		// private_node_handleはプライベートパラメータを呼び出すためのノードハンドル
		// .launch で引数とった，もしくはdefaultの値が用いられているので使用
		private_node_handle_.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
		private_node_handle_.param<std::string>("map_frame", map_frame_, "map");
		private_node_handle_.param<double>("grid_resolution", grid_resolution_, 0.2);
		private_node_handle_.param<double>("grid_length_x", grid_length_x_, 80);
		private_node_handle_.param<double>("grid_length_y", grid_length_y_, 30);
		private_node_handle_.param<double>("grid_position_x", grid_position_x_, 20);
		private_node_handle_.param<double>("grid_position_x", grid_position_y_, 0);
		
		// オクルージョン領域を"occlusion_area"という名前のトピックにgrid_map_msgs::GridMap形式のノードを送ることを伝える
		publisher_grid_map_ = node_handle_.advertise<grid_map_msgs::GridMap>("occlusion_area", 1, true);
		// オクルージョンではない領域を"openning_area"という名前のトピックにros nav_msgs :: OccupancyGrid形式のノードを送ることを伝える
		publisher_occupancy_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("openning_area", 1, true);
		
		obj_subscriber_ = nh_.subscribe("/detected_objects", 1,&PotentialField::obj_callback, this);
	}

	void WayareaToGrid::Run()
	{
		bool set_map = false;
		ros::Rate loop_rate(10);







// ==========================================================================
		// 見つけたすべての障害物すべてに対する処理
		for (int i(0); i < (int)obj_msg->objects.size(); ++i) {

		}
		
		// 見つけた物体の位置 X [m]
		double pos_x = obj_msg->objects.at(i).pose.position.x;
		// 見つけた物体の位置 Y [m]
		double pos_y = obj_msg->objects.at(i).pose.position.y;
		// 見つけた物体のx方向長さ L_x [m]
        double len_x = obj_msg->objects.at(i).dimensions.x / 2.0;
        // 見つけた物体のy方向長さ L_y [m]
		double len_y = obj_msg->objects.at(i).dimensions.y / 2.0;
		// 
      double r, p, y;
      tf::Quaternion quat(obj_msg->objects.at(i).pose.orientation.x,
                          obj_msg->objects.at(i).pose.orientation.y,
                          obj_msg->objects.at(i).pose.orientation.z,
                          obj_msg->objects.at(i).pose.orientation.w);
      tf::Matrix3x3(quat).getRPY(r, p, y);
 // ==========================================================================





		while (ros::ok())
		{
			if (!set_map)
			{
				gridmap_.add(grid_layer_name_);
				gridmap_.setFrameId(sensor_frame_);
				gridmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_),
				                grid_resolution_,
				                grid_map::Position(grid_position_x_, grid_position_y_));
				set_map = true;
			}

			// timer start
			//auto start = std::chrono::system_clock::now();

			if (!area_points_.empty())
			{
				FillPolygonAreas(gridmap_, area_points_, grid_layer_name_, OCCUPANCY_NO_ROAD, OCCUPANCY_ROAD, grid_min_value_,
				                 grid_max_value_, sensor_frame_, map_frame_,
				                 tf_listener_);
				PublishGridMap(gridmap_, publisher_grid_map_);
				PublishOccupancyGrid(gridmap_, publisher_occupancy_, grid_layer_name_, grid_min_value_, grid_max_value_);
			}

			// timer end
			//auto end = std::chrono::system_clock::now();
			//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
			//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

			loop_rate.sleep();
		}
	}

}  // namespace object_map



#include "road_occupancy_processor.h"

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

void RosRoadOccupancyProcessorApp::PublishGridMap(grid_map::GridMap &in_grid_map, const std::string& in_layer_publish)
{
	// gridmapにパブリッシュするレイヤー"occlusion"を設定する
	gridmap_setFrameId("occlusion");
	// 
	gridmap_()


	// 配信用のレイヤーがデータに含まれている場合
	if (in_grid_map.exists(in_layer_publish))
	{
		// grid_map_msg/GridMap.msg型の変数：ros_gridmap_message
		grid_map_msgs::GridMap ros_gridmap_message;
		// 
		nav_msgs::OccupancyGrid ros_occupancygrid_message;
		// GridMap型のROSトピックを生成する：引数（グリッドマップ，メッセージ）
		grid_map::GridMapRosConverter::toMessage(in_grid_map, ros_gridmap_message);
		// 
		grid_map::GridMapRosConverter::toOccupancyGrid(in_grid_map,
		                                               in_layer_publish,
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
		ROS_INFO("[%s] Empty GridMap. It might still be loading or it does not contain valid data.", occlusion);
	}
}

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

	ROS_INFO("[%s] Empty Image received.", occlusion);
	return false;
}

void RosRoadOccupancyProcessorApp::Convert3dPointToOccupancy(grid_map::GridMap &in_grid_map,
                                                             const geometry_msgs::Point &in_point, cv::Point &out_point)
{
	// calculate position
	grid_map::Position map_pos = in_grid_map.getPosition();
	double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
	double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
	// coordinate conversion for cv image
	out_point.x = (in_grid_map.getLength().y() - origin_y_offset - in_point.y) / in_grid_map.getResolution();
	out_point.y = (in_grid_map.getLength().x() - origin_x_offset - in_point.x) / in_grid_map.getResolution();
}

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
// 
void Occlusion::GridMapCallback(const grid_map_msgs::GridMap& in_message)
{
	grid_map::GridMap input_grid;
	grid_map::GridMapRosConverter::fromMessage(in_message, input_grid);

	grid_map::GridMapCvConverter::toImage<unsigned char, 1>(input_grid,
	                                                        occupancy_layer_name_,
	                                                        CV_8UC1,
	                                                        grid_min_value_,
	                                                        grid_max_value_,
	                                                        road_wayarea_original_mat_);

	input_gridmap_frame_        = input_grid.getFrameId();
	input_gridmap_length_       = input_grid.getLength();
	// Get the resolution of the grid map.
	// Returns : resolution of the grid map in the xy plane [m/cell].
	// Road Occupancy Processorで使用されている解像度を選択する
	input_gridmap_resolution_   = input_grid.getResolution();
	// 
	input_gridmap_position_     = input_grid.getPosition();
}
// Detected Object メッセージを受け取った時：callback関数
void Occlusion::GridMapCallback(const grid_map_msgs::GridMap& in_message)
{

	grid_map::GridMap input_grid;
	grid_map::GridMapRosConverter::fromMessage(in_message, input_grid);

	grid_map::GridMapCvConverter::toImage<unsigned char, 1>(input_grid,
	                                                        occupancy_layer_name_,
	                                                        CV_8UC1,
	                                                        grid_min_value_,
	                                                        grid_max_value_,
	                                                        road_wayarea_original_mat_);

	input_gridmap_frame_        = input_grid.getFrameId();
	input_gridmap_length_       = input_grid.getLength();
	input_gridmap_resolution_   = input_grid.getResolution();



	input_gridmap_position_     = input_grid.getPosition();
}
void Occlusion::ObjectCallback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg)
{
	// データをGridMapに格納していく
	ros::Time time = ros::Time::now();
	// itにはIndexの集まりが保存される
	for (GridMapIterator it(gridmap_); !it.isPastEnd(); ++it) {
		// あるインデックスに対応するグリッドマップフレーム上での位置：宣言（初期化）
		Position grid_position;
		// あるインデックスに対応するグリッドマップフレーム上での位置：代入
		gridmap_.getPosition(*it, grid_position);
		// レイヤー"occlusion"の中のインデックスitにデータを格納
		gridmap_.at("occlusion", *it) = 1;// ここに計算式を組み込む
		//

	}
	// 検出した物体の数すべてに対して反復
	for (int i(0); i < (int)obj_msg->objects.size(); ++i) {
	// 位置X
	double pos_x = obj_msg->objects.at(i).pose.position.x;
    // 位置Y  
	double pos_y = obj_msg->objects.at(i).pose.position.y;
    // 物体のX方向長さ
    double len_x = obj_msg->objects.at(i).dimensions.x;
    // 物体のY方向長さ
    double len_y = obj_msg->objects.at(i).dimensions.y;
    // 物体のZ方向長さ
    double len_z = obj_msg->objects.at(i).dimensions.z;
    // クオータニオンからRoll,Yaw,Pitchへの変換
    double r, p, y;

	}
}





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
	{
		out_pointcloud = in_pointcloud;
	}
};
// 
void Occlusion::OccupancyCallback(const grid_map_msgs::GridMap& gridmap_ros_in_message)
{

	grid_map::GridMap input_grid;

	grid_map::GridMapRosConverter::fromMessage(gridmap_ros_in_message, input_grid);

	grid_map::GridMapCvConverter::toImage<unsigned char, 1>(input_grid,
	                                                        occupancy_layer_name_,
	                                                        CV_8UC1,
	                                                        grid_min_value_,
	                                                        grid_max_value_,
	                                                        road_wayarea_original_mat_);

	input_gridmap_frame_        = input_grid.getFrameId();
	input_gridmap_length_       = input_grid.getLength();
	input_gridmap_resolution_   = input_grid.getResolution();
	input_gridmap_position_     = input_grid.getPosition();

	gridmap_.add("Occupancy", input_grid.get(occupancy_layer_name_));
	// Index(Eigen::Array2i)→[A,B]という形
	gridmap::Index 


}

void RosRoadOccupancyProcessorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_ground_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_no_ground_cloud_msg)
{
	if(road_wayarea_original_mat_.empty())
		return;

	// timer start
	//auto start = std::chrono::system_clock::now();

	cv::Mat current_road_mat = road_wayarea_original_mat_.clone();
	cv::Mat original_road_mat = current_road_mat.clone();

	grid_map::GridMap output_gridmap;
	output_gridmap.setFrameId(input_gridmap_frame_);
	
	
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
	output_gridmap.setGeometry(input_gridmap_length_,
	                           input_gridmap_resolution_,
	                           input_gridmap_position_);
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
    output_gridmap.getLength().x(), output_gridmap.getLength().y(),
    output_gridmap.getSize()(0), output_gridmap.getSize()(1),
    output_gridmap.getPosition().x(), output_gridmap.getPosition().y(), output_gridmap.getFrameId().c_str());
	/* ******************************************************************************************
	 * isInside
	 * ------------------------------------------------------------------------------------------
	 * bool grid_map::GridMap::isInside	(	const Position & 	position	)	const
	 * Check if position is within the map boundaries.
	 * 
	 * Parameters
	 * position	the position to be checked.
	 * Returns
	 * true if position is within map, false otherwise.
	 * Definition at line 237 of file GridMap.cpp.
	 * **************************************************************************************** */
	//// Adding outliers (accessing cell by position).
    //for (unsigned int i = 0; i < 500; ++i) {
    //  Position randomPosition = Position::Random();
    //  if (map.isInside(randomPosition))
	/* ******************************************************************************************
	 * atPosition
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
    //   map.atPosition("elevation_noisy", randomPosition) = std::numeric_limits<float>::infinity();
	// }
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


	//output_gridmap[output_layer_name_].setConstant(OCCUPANCY_NO_ROAD);

	pcl::PointCloud<pcl::PointXYZI>::Ptr in_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr in_no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr final_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr final_no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(*in_ground_cloud_msg, *in_ground_cloud);
	pcl::fromROSMsg(*in_no_ground_cloud_msg, *in_no_ground_cloud);

	//check that both pointcloud and grid are in the same frame, otherwise transform
	ConvertPointCloud(*in_ground_cloud, output_gridmap.getFrameId(), *final_ground_cloud);
	ConvertPointCloud(*in_no_ground_cloud, output_gridmap.getFrameId(), *final_no_ground_cloud);

	//organize pointcloud in cylindrical coords
	PointCloudXYZIRTColor organized_points;
	std::vector<pcl::PointIndices> radial_division_indices;
	std::vector<pcl::PointIndices> closest_indices;
	std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

	ConvertXYZIToRTZ(final_ground_cloud,
	                 organized_points,
	                 radial_division_indices,
	                 radial_ordered_clouds);

	//draw lines between initial and last point of each ray
	for (size_t i = 0; i < radial_ordered_clouds.size(); i++)//sweep through each radial division
	{
		geometry_msgs::Point prev_point;
		for (size_t j = 0; j < radial_ordered_clouds[i].size(); j++)//loop through each point
		{
			geometry_msgs::Point current_point;
			current_point.x = radial_ordered_clouds[i][j].point.x;
			current_point.y = radial_ordered_clouds[i][j].point.y;
			current_point.z = radial_ordered_clouds[i][j].point.z;

			DrawLineInGridMap(output_gridmap, current_road_mat, prev_point, current_point, OCCUPANCY_ROAD_FREE);
		}
	}

	//process obstacle points
	for (const auto &point:final_no_ground_cloud->points)
	{
		geometry_msgs::Point sensor_point;
		sensor_point.x = point.x;
		sensor_point.y = point.y;
		sensor_point.z = point.z;
		SetPointInGridMap(output_gridmap, current_road_mat, sensor_point, OCCUPANCY_ROAD_OCCUPIED);
	}

#pragma omp for
	for(int row = 0; row < current_road_mat.rows; row++)
	{
		for(int col = 0; col < current_road_mat.cols; col++)
		{
			if(original_road_mat.at<uchar>(row,col) == OCCUPANCY_NO_ROAD)
			{
				current_road_mat.at<uchar>(row,col) = OCCUPANCY_NO_ROAD;
			}
		}
	}
	//cv::imshow("result", current_road_mat);
	//cv::waitKey(10);
	LoadRoadLayerFromMat(output_gridmap, current_road_mat);

	// タイムスタンプ：GridMapのタイムスタンプを
	gridmap_.setTimestamp(time.toNSec());

	// occlusion計算後のGridMapをパブリッシュする：引数（配信用GridMap，配信用GridMapに）
	PublishGridMap(output_gridmap, output_layer_name_);

	// timer end
	//auto end = std::chrono::system_clock::now();
	//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}
// mainプログラム：init()
void Occlusion::init(ros::NodeHandle &in_private_handle)
{
	
	// ！！！要チェック！！！
	std::string occupancy_topic_str;

	// Set the frame id of the grid map
	map.setFrameId("map");
	// 
    map.setGeometry(Length(1.2, 2.0), 0.03);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));




	/* ******************************************************************************************
	 * パラメータ取得
	 * ------------------------------------------------------------------------------------------
	 * param（）はgetParam（）に似ていますが、パラメータを取得できなかった場合,
	 * デフォルト値を指定できます。
	 * コンパイラが文字列型のヒントを要求することがあります。
	 * **************************************************************************************** */

	// パラメータ取得：<arg name="occupancy_topic_src" default="gridmap_road_status" />
	// <!-- Road Occupancy Processor が配信しているトピック名 -->
	in_private_handle.param<std::string>("occupancy_topic_src", occupancy_topic_str, "gridmap_road_status");
	ROS_INFO("[%s] occupancy_topic_src: %s",occlusion, occupancy_topic_str.c_str());
	// パラメータ取得：<arg name="occupancy_layer_name" default="road_status" />
	// <!-- Road Occupancy Processor の GridMap クラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("occupancy_layer_name", occupancy_layer_name_, "road_status");
	ROS_INFO("[%s] occupancy_layer_name: %s",occlusion, occupancy_layer_name_.c_str());
    // パラメータ取得；<arg name="output_layer_name" default="road_occlusion" />
	// <!-- Occlusion が配信する GridMapクラスメッセージにおけるレイヤー名 -->
	in_private_handle.param<std::string>("output_layer_name", output_layer_name_, "road_occlusion");
	ROS_INFO("[%s] output_layer_name: %s",occlusion, output_layer_name_.c_str());
	// パラメータ取得：	<arg name="road_unknown_value" default="128" />   
	in_private_handle.param<int>("road_unknown_value", OCCUPANCY_ROAD_UNKNOWN, 128);
	ROS_INFO("[%s] road_unknown_value: %d",occlusion, OCCUPANCY_ROAD_UNKNOWN);
	// パラメータ取得：<arg name="road_free_value" default="75" /> 
	in_private_handle.param<int>("road_free_value", OCCUPANCY_ROAD_FREE, 75);
	ROS_INFO("[%s] road_free_value: %d",occlusion, OCCUPANCY_ROAD_FREE);
	// パラメータ取得：<arg name="road_occupied_value" default="0" />      
	in_private_handle.param<int>("road_occupied_value", OCCUPANCY_ROAD_OCCUPIED, 0);
	ROS_INFO("[%s] road_occupied_value: %d",occlusion, OCCUPANCY_ROAD_OCCUPIED);
	// パラメータ取得：<arg name="no_road_value" default="255" />  
	in_private_handle.param<int>("no_road_value", OCCUPANCY_NO_ROAD, 255);
	ROS_INFO("[%s] no_road_value: %d",occlusion, OCCUPANCY_NO_ROAD);

	/* ******************************************************************************************
	 * サブスクライバー宣言
	 * ------------------------------------------------------------------------------------------
	 * 
	 * 
	 * 
	 * **************************************************************************************** */

	// サブスクライバー宣言：Road Occupancy Grid ノードが配信するトピックを購読
	// トピックを受け取った場合→&Occlusion::OccupancyCallback関数を呼び出す
	gridmap_subscriber_ = node_handle_.subscribe(occupancy_topic_str, 1, &Occlusion::OccupancyCallback, this);
	ROS_INFO("[%s] Subscribing to... %s",occlusion, occupancy_topic_str.c_str());
	// サブスクライバー宣言：トピック "DetectedObjects" を購読
	// トピックを受け取った場合→&Occlusion::OccupancyCallback関数を呼び出す
	objects_subscriber_ = node_handle_.subscribe("/detected_objects", 1, &Occlusion::ObjectCallback, this);
	ROS_INFO("[%s] Subscribing to... /detected_objects",occlusion);

	// パブリッシャー宣言：Occlusion ノードとして配信するトピックを登録
	publisher_grid_map_= node_handle_.advertise<grid_map_msgs::GridMap>("gridmap_occlusion", 1);
	ROS_INFO("[%s] Publishing GridMap in gridmap_road_status",occlusion);
	// パブリッシャー宣言：Occlusion ノードとして配信するトピックを登録
	publisher_occupancy_grid_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_occlusion", 1);
	ROS_INFO("[%s] Publishing Occupancy grid in occupancy_road_status",occlusion);
	// デバック用
	ROS_INFO("Created Map");




}

tf::StampedTransform
RosRoadOccupancyProcessorApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
	tf::StampedTransform transform;

	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}

	return transform;
}

geometry_msgs::Point
RosRoadOccupancyProcessorApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_transform)
{
	tf::Point tf_point;
	tf::pointMsgToTF(in_point, tf_point);

	tf_point = in_transform * tf_point;

	geometry_msgs::Point geometry_point;
	tf::pointTFToMsg(tf_point, geometry_point);

	return geometry_point;
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
	InitializeRosIo(private_node_handle);
	//
	
	// 
	ROS_INFO("[%s] Ready. Waiting for data...",occlusion);
	// 
	ros::spin();
	// 
	ROS_INFO("[%s] END",occlusion);
}

RosRoadOccupancyProcessorApp::RosRoadOccupancyProcessorApp()
{
	radial_dividers_num_ = ceil(360 / radial_divider_angle_);
}
// 検出された物体情報を取り出す
Occlusion::Occlusion()
{
	
}