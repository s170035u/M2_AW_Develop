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
