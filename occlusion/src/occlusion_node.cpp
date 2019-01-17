// ROSに必要なヘッダーを読み込む
#include <ros/ros.h>
// includeファイルのocclusion.hを読み込む
#include "occlusion.h"
// C++の本体関数
int main(int argc, char **argv) {
  // ROSの初期化
  ros::init(argc, argv, "occlusion");
  // Occlusion クラスをインスタンス化
  Occlusion occlusion;
  // occlusionクラス：オクルージョン計算メソッド
  occlusion.run();
  // 終了
  return 0;
}
