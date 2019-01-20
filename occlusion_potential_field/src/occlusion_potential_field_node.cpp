// ROSに必要
#include <ros/ros.h>
// ヘッダーファイル読み込み
#include "occlusion_potential_field.h"
// メイン関数
int main(int argc, char **argv) {
  ros::init(argc, argv, "occlusion_potential_field");
  OcclusionPotentialField occlusion_potential_field;
  occlusion_potential_field.run();
  return 0;
}
