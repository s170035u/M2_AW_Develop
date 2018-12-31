#ifndef OCCLUSION_POTENTIAL_H
#define OCCLUSION_POTENTIAL_H
#include <ros/ros.h>


class PotentialField {
private:
  //
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  ros::Subscriber waypoint_subscriber_;
  ros::Subscriber vscan_subscriber_;
  ros::Subscriber obj_subscriber_;
  bool use_target_waypoint_;
  bool use_obstacle_box_;
  bool use_vscan_points_;
  double map_x_size_;
  double map_y_size_;
  double map_resolution_;
  double tf_x_;
  double tf_z_;
  double map_x_offset_;
  GridMap map_;
  class ObstacleFieldParameter {
  public:
    ObstacleFieldParameter() : ver_x_p(0.9), ver_y_p(0.9) {}
    double ver_x_p;
    double ver_y_p;
  };
  class TargetWaypointFieldParamater {
  public:
    TargetWaypointFieldParamater() : ver_x_p(1.0), ver_y_p(1.0) {}
    double ver_x_p;
    double ver_y_p;
  };
  class VscanPointsFieldParamater {
  public:
    VscanPointsFieldParamater() : around_x(0.5), around_y(0.5) {}
    double around_x;
    double around_y;
  };

  void obj_callback(autoware_msgs::DetectedObjectArray::ConstPtr obj_msg);
  void target_waypoint_callback(
      visualization_msgs::Marker::ConstPtr target_point_msgs);
  void vscan_points_callback(sensor_msgs::PointCloud2::ConstPtr vscan_msg);
  void publish_potential_field();

public:
  PotentialField();
  void run();
  void init();
};

PotentialField::PotentialField()
    : tf_x_(1.2), tf_z_(2.0),
      map_({"potential_field", "obstacle_field", "target_waypoint_field",
            "vscan_points_field"}) {
  ros::NodeHandle private_nh("~");
  if (!private_nh.getParam("use_obstacle_box", use_obstacle_box_)) {
    ROS_INFO("use obstacle_box");
    use_obstacle_box_ = true;
  }
  if (!private_nh.getParam("use_vscan_points", use_vscan_points_)) {
    ROS_INFO("use vscan points");
    use_vscan_points_ = true;
  }
  if (!private_nh.getParam("use_target_waypoint", use_target_waypoint_)) {
    ROS_INFO("don't use target_waypoint");
    use_target_waypoint_ = false;
  }
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
  publisher_ =
      nh_.advertise<grid_map_msgs::GridMap>("/potential_field", 1, true);

  if (use_obstacle_box_)
    obj_subscriber_ = nh_.subscribe("/detected_objects", 1,
                                    &PotentialField::obj_callback, this);
  if (use_vscan_points_)
    vscan_subscriber_ = nh_.subscribe(
        "/vscan_points", 1, &PotentialField::vscan_points_callback, this);
  if (use_target_waypoint_)
    waypoint_subscriber_ =
        nh_.subscribe("/next_target_mark", 1,
                      &PotentialField::target_waypoint_callback, this);
}