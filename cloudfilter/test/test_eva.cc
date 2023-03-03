#include <eva.hh>// 包含评估函数eva的头文件
#include <gflags/gflags.h>// 包含gflags库的头文件，用于解析命令行参数

DEFINE_string(sp, "", "sam_positive"); //定义命令行参数sp，表示样本中的正样本点云文件路径
DEFINE_string(sn, "", "sam_negative"); //定义命令行参数sn，表示样本中的负样本点云文件路径
DEFINE_string(ep, "", "exp_positive"); //定义命令行参数ep，表示待评估的正样本点云文件路径
DEFINE_string(en, "", "exp_negative"); //定义命令行参数en，表示待评估的负样本点云文件路径

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true); //解析命令行参数
  // 创建四个点云指针，用于存储加载的点云数据
  PointCloudT::Ptr sp(new PointCloudT);
  PointCloudT::Ptr sn(new PointCloudT);
  PointCloudT::Ptr ep(new PointCloudT);
  PointCloudT::Ptr en(new PointCloudT);
   // 加载点云数据
  pcl::io::loadPCDFile(FLAGS_sp, *sp);
  pcl::io::loadPCDFile(FLAGS_sn, *sn);
  pcl::io::loadPCDFile(FLAGS_ep, *ep);
  pcl::io::loadPCDFile(FLAGS_en, *en);

  enum { precision, recall, accuracy, tp_rate, fp_rate, fn_rate };//定义枚举类型，表示评估指标
  std::vector<double> result = eva(sp, sn, ep, en);//调用评估函数eva计算指标值
  // 输出评估结果
  PCL_INFO("%.2f %.2f %lf %.2f %.2f %.2f\n", result[precision] * 1e2, result[recall] * 1e2,
           result[accuracy], result[tp_rate] * 1e2, result[fp_rate] * 1e2, result[fn_rate] * 1e2);

  return 0;
}