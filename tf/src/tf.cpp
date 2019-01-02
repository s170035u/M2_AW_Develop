#include "tf/transform_broadcaster.h"
 
//TF Broadcasterの実体化
tf::TransformBroadcaster glabal_robot_broadcaster;
 
//Robot位置と姿勢(x,y,yaw)の取得
double x=GetRobotPositionX();
double y=GetRobotPositionY();
double yaw=GetRobotPositionYaw();
 
//yawのデータからクォータニオンを作成
geometry_msgs::Quaternion robot_quat=tf::createQuaternionMsgFromYaw(yaw);
 
//robot座標系の元となるロボットの位置姿勢情報格納用変数の作成
geometry_msgs::TransformStamped robotState;
 
//現在の時間の格納
robotState.header.stamp = ros::Time::now();
 
//座標系globalとrobotの指定
robotState.header.frame_id = "global";
robotState.child_frame_id  = "robot";
 
//global座標系からみたrobot座標系の原点位置と方向の格納
robotState.transform.translation.x = x;
robotState.transform.translation.y = y;
robotState.transform.translation.z = z;
robotState.transform.rotation = robot_quat;
 
//tf情報をbroadcast(座標系の設定)
glabal_robot_broadcaster.sendTransform(robotState);