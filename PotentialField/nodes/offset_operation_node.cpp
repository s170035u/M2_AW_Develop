/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
*/

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>


#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>


#include <autoware_msgs/LaneArray.h>
#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/ConfigVelocitySet.h"


//Subscribing data
autoware_msgs::lane subscribed_waypoints_;
int closest_waypoint_index_ = -1;
geometry_msgs::PoseStamped current_pose_;
double current_velocity_mps_;
pcl::PointCloud<pcl::PointXYZ> points_;


//Subscribing parameters 
double detection_height_top_    = -0.75;
double detection_height_bottom_ = -2.30 ;
double stop_distance_obstacle_  = 2.75;
double stop_range_              = 1.0;
double remove_radius_max_2 = 30.0*30.0;
double remove_radius_min_2 = 1.5*1.5;

double max_offset_ = 1.0;


//Do generate offset path? 
bool DoOffset = false;

//Dp change speed depend on the free space?
bool DoSlowdown = false;

// Free space data pf a waypoint
struct LaneRiskMap
{
	tf::Vector3 BasePosition;
	
	double Length;
	tf::Vector3 Direction;
	
	double OffsetValue;
	
	double LeftDist;
	double RightDist;
};

std::vector<LaneRiskMap> LocalWaypoints;


//ROS Publishers
ros::Publisher waypoints_pub;
ros::Publisher mark_pub;

//---------------
// Callback functions

void configCallback(const autoware_msgs::ConfigVelocitySetConstPtr &config)
{
  //stop_distance_obstacle_ = config->others_distance;
  //stop_range_ = config->detection_range;
  detection_height_top_ = config->detection_height_top;
  detection_height_bottom_ = config->detection_height_bottom;
}


void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
 	current_pose_ = *msg;
}

void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  	current_velocity_mps_ = msg->twist.linear.x;
}


void waypointsCallback(const autoware_msgs::laneConstPtr &msg)
{
  subscribed_waypoints_ = *msg;
	
  auto& waypoints = subscribed_waypoints_.waypoints;
  
  if(LocalWaypoints.size() == waypoints.size()-1)
   return;
   
  LocalWaypoints.clear();
	
  for (size_t WPID = 0; WPID < waypoints.size()-1; WPID++)
  {

    LaneRiskMap NewWaypoint;
    
    NewWaypoint.Length   = 1.0;
	NewWaypoint.OffsetValue = 0.0;
	
    LocalWaypoints.push_back(NewWaypoint);
  }
  
}

void closestWaypointCallback(const std_msgs::Int32ConstPtr &msg)
{
	closest_waypoint_index_ = msg->data;
}

void stateCallback(const std_msgs::StringConstPtr &msg)
{
	std::string state_ = msg->data;
	

}


void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  //ROS_INFO("Point:%d",sub_points.size());
  	
  	
  points_.clear();
  for (const auto &v : sub_points)
  {
    if (v.x == 0 && v.y == 0)
      continue;

    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;

	double PointDist2 = v.x * v.x + v.y * v.y;
	
	
    if (   PointDist2 < remove_radius_min_2)
      continue;
      

    points_.push_back(v);
  }
  
}

//---------------


//---------------
//visualization helper
visualization_msgs::Marker GenRVIZMaker()
{
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 1.0;
  marker.frame_locked = true;
  
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.lifetime = ros::Duration(0);
  
  marker.ns = "path";
  marker.id = 0;
  marker.frame_locked = true;
  
  return marker;
}
//---------------


//---------------
//Path generation main

void ObstacleAvoidance()
{
  //This should be a parameter.
  const int SearchDistance = 50;


  auto& waypoints = subscribed_waypoints_.waypoints;
  
  // when no waypoints (init. stage)
  if(waypoints.size() < 2  || LocalWaypoints.size() < 2)return;
  
  if( closest_waypoint_index_ < 0  )
  {
    double weight = 1.0;
    for(auto &wp : subscribed_waypoints_.waypoints)
    {
	wp.twist.twist.linear.x *= weight;
	weight*= 0.8;
    }
    waypoints_pub.publish(subscribed_waypoints_);
    return;
  }
  
  //prepare: calc. waypoint position and direction in the relative coordinate of the ego-vehicle.
  autoware_msgs::lane offset_waypoints;
  
  offset_waypoints.header    = subscribed_waypoints_.header;
  offset_waypoints.increment = subscribed_waypoints_.increment;
  offset_waypoints.lane_id   = subscribed_waypoints_.lane_id;
  offset_waypoints.waypoints.clear();
  

  const int EndIndex =  closest_waypoint_index_ + SearchDistance < (int)waypoints.size()?
  						closest_waypoint_index_ + SearchDistance-1:
  						(int)waypoints.size()-1;

  for (int id = closest_waypoint_index_; id<EndIndex;id++)
  {
  	// waypoint seen by car
    geometry_msgs::Point waypoint1 = calcRelativeCoordinate(waypoints[id].pose.pose.position, current_pose_.pose);
    tf::Vector3 tf_waypoint1 = point2vector(waypoint1);
    
    geometry_msgs::Point waypoint2 = calcRelativeCoordinate(waypoints[id+1].pose.pose.position, current_pose_.pose);
    tf::Vector3 tf_waypoint2 = point2vector(waypoint2);
    
    tf_waypoint1.setZ(0);
    tf_waypoint2.setZ(0);
    
    LocalWaypoints[id].BasePosition = tf_waypoint1;
    LocalWaypoints[id].Direction = tf_waypoint2 - tf_waypoint1;
	
	LocalWaypoints[id].Length = LocalWaypoints[id].Direction.length();
	LocalWaypoints[id].Direction /= LocalWaypoints[id].Length;
	
  }

  //for visualize
  visualization_msgs::MarkerArray marker_arrey;
  
  {
	  visualization_msgs::Marker marker_org = GenRVIZMaker();
	  
	  marker_org.color.g = 1;
	  marker_org.ns = "org_path";
	  
	  //Add original waypoints
	  for (int id = closest_waypoint_index_; id<EndIndex;id++)
	  {
	    marker_org.points.push_back(waypoints[ id ].pose.pose.position);
	  }
	  
	  marker_arrey.markers.push_back(marker_org);
  }
  
  
  
  //check free space around the waypoints
  for (int id = closest_waypoint_index_; id<EndIndex;id++)
  {
	auto& ThisWP = LocalWaypoints[id];
	autoware_msgs::waypoint OffsetWaypoint = waypoints[ id ];
	
	double RangeMin = -ThisWP.Length*2.0;
	double RangeMax =  ThisWP.Length*2.0;
	
	std::vector<double> MaxList = {-100.0,-100.0,-100.0,-100.0,-100.0};
	std::vector<double> MinList = { 100.0, 100.0, 100.0, 100.0, 100.0};
	
    for (const auto& p : points_)
    {
	  tf::Vector3 DiffVec = tf::Vector3(p.x, p.y, 0) - ThisWP.BasePosition;
	  
	  
	  double DistLng = ThisWP.Direction.dot(DiffVec);

	  if( DistLng < RangeMin || DistLng > RangeMax)
	  {
		continue;
	  }
	  
	  
	  double DistLat =  DiffVec.cross(ThisWP.Direction).getZ();
	  
	  for(size_t t=0;t<MaxList.size();t++)
	  {
	  	if(DistLat < 0 && MaxList[t] < DistLat)
	  	{
	  		double LastMax = MaxList[t];
	  		MaxList[t] = DistLat;
	  		DistLat = LastMax;
	  	}
	  }
	  
	  for(size_t t=0;t<MinList.size();t++)
	  {
	  	if(DistLat > 0 && MinList[t] > DistLat)
	  	{
	  		double LastMin = MinList[t];
	  		MinList[t] = DistLat;
	  		DistLat = LastMin;
	  	}
	  }
    }
    
    double AverageMax = 0.0;
    for(auto val : MaxList) AverageMax += val;
	AverageMax /= MaxList.size();
	
	double AverageMin = 0.0;
    for(auto val : MinList) AverageMin += val;
	AverageMin /= MinList.size();
	  
	//decide offset value (m)
	double ThisOffset = max_offset_*exp(- AverageMin*AverageMin / 2.5/2.5 ) - max_offset_*exp(- AverageMax*AverageMax / 2.5/2.5 );
	
	ThisWP.OffsetValue =  0.7*ThisWP.OffsetValue + 0.3*ThisOffset;
	
	
	ThisWP.LeftDist   = AverageMin;
	ThisWP.RightDist  = AverageMax;
	
	
  }
  
  
  //path smoothing
  int window_size = 3;
  std::vector<double> FinalOffset;
  
  
  for(int t=0;t<(int)LocalWaypoints.size();t++)
  {
  	if(t<window_size || t >= (int)LocalWaypoints.size() - window_size )
  	{
  		FinalOffset.push_back(LocalWaypoints[t].OffsetValue);
  	}
  	else
  	{
  		double Sum = 0.0;
  		for(int offset = -window_size;offset <= window_size;offset++)
  			Sum +=  LocalWaypoints[t+offset].OffsetValue;

  		FinalOffset.push_back(Sum/(window_size*2+1));
  	}
  }
  
  
  visualization_msgs::Marker marker_avoid = GenRVIZMaker();
  
  marker_avoid.ns = "new_path";
  marker_avoid.color.r = 1;

  //output avoiding path
  
  for (int id = closest_waypoint_index_; id<EndIndex;id++)
  {
	auto& ThisWP = LocalWaypoints[id];
	autoware_msgs::waypoint OffsetWaypoint = waypoints[ id ];
	
	if( DoOffset )
	{
		tf::Vector3 OffsetVector(  -ThisWP.Direction.getY()*FinalOffset[id]
    						 ,  ThisWP.Direction.getX()*FinalOffset[id]
    						 ,0.0);
    
		ThisWP.BasePosition += OffsetVector;
	
		OffsetWaypoint.pose.pose.position = calcAbsoluteCoordinate( vector2point( ThisWP.BasePosition ), current_pose_.pose);
	}
		
	if(DoSlowdown)
	{

		double DistL = fabs( ThisWP.LeftDist  + FinalOffset[id] );
		double DistR = fabs( ThisWP.RightDist + FinalOffset[id] );;
		double MinDist = DistL > DistR ? DistR: DistL;
	
		double slow_start =  1.0 - exp( -MinDist*MinDist/2.0 ); 
		
		OffsetWaypoint.twist.twist.linear.x = waypoints[id].twist.twist.linear.x * slow_start;
	}
		
	offset_waypoints.waypoints.push_back(OffsetWaypoint);
    marker_avoid.points.push_back(OffsetWaypoint.pose.pose.position);
    
  }
  
 
  waypoints_pub.publish(offset_waypoints);
 
  marker_arrey.markers.push_back(marker_avoid);
  
  mark_pub.publish(marker_arrey);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "offset_operation");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  
  // ROS subscribers
  ros::Subscriber config_sub = n.subscribe("config/velocity_set", 1, &configCallback);
  
  ros::Subscriber start_sub             = n.subscribe("current_pose", 1, currentPoseCallback);
  ros::Subscriber waypoints_sub         = n.subscribe("base_waypoints", 1, waypointsCallback);
  ros::Subscriber closest_waypoint_sub  = n.subscribe("closest_waypoint", 1, closestWaypointCallback);

  ros::Subscriber current_velocity_sub  = n.subscribe("current_velocity", 1, currentVelocityCallback);
  ros::Subscriber state_sub             = n.subscribe("state", 1, stateCallback);

  std::string points_topic;
  private_nh.param<std::string>("points_topic", points_topic, "points_no_ground");
  ros::Subscriber points_sub = n.subscribe(points_topic, 1, pointsCallback);

  private_nh.param<double>("remove_points_upto", remove_radius_min_2, 2.3);
  remove_radius_min_2 *= remove_radius_min_2;
  
  private_nh.param<double>("max_offset", max_offset_, 1.0);

  private_nh.param<bool>("do_offset", DoOffset, false);
  private_nh.param<bool>("do_slowdown", DoSlowdown, false);
  
  // ROS publishers
  waypoints_pub = n.advertise<autoware_msgs::lane>("safety_waypoints", 1, true);

  mark_pub = n.advertise<visualization_msgs::MarkerArray>("/new_waypoints_mark", 10, true);
    
  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    ros::spinOnce();

	ObstacleAvoidance();
    loop_rate.sleep();
  }

  return 0;
}
