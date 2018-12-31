#include <ros/ros.h>
#include "occlusion.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "occlusion");
  occlusion_calc::Occlusion occlusion;
  occlusion.init();
  occlusion.run();
  return 0;
}
