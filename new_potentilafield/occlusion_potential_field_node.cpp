// ROSに必要
#include <ros/ros.h>
// ヘッダーファイル読み込み
#include "occlusion_potential_field.h"
// 
int main(int argc, char **argv) {
  ros::init(argc, argv, "occlusion_potential_field");

  PotentialField potential_field;
  potential_field.init();
  potential_field.run();
  return 0;
}
