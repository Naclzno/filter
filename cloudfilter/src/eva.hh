#ifndef INCLUDE_EVA_HH
#define INCLUDE_EVA_HH

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
//这段代码实现了计算两个点云之间的交集数量。
int intersaction(const PointCloudT::Ptr &c1, PointCloudT::Ptr c2) {
//初始化一个变量 correct 用于记录交集数量，并初始化一个整型变量 k，用于指定搜索邻居的数量。
  int correct = 0;
  int k = 1;
//这段代码创建了一个指向pcl::KdTreeFLANN类型的智能指针kdtree
//并通过new关键字在堆内存中动态分配了一个pcl::KdTreeFLANN对象
//该对象用于在点云中执行k近邻搜索操作，它的模板参数PointT表示点云中点的数据类型。
  auto kdtree(new pcl::KdTreeFLANN<PointT>);

  kdtree->setInputCloud(c2);
//定义两个 std::vector 用于存储最近邻搜索的结果，
//其中 pointIdxNKNSearch 用于存储搜索到的最近邻点的索引，
//pointNKNSquareDistance 用于存储搜索到的最近邻点和查询点之间的距离平方。
  std::vector<int> pointIdxNKNSearch(k);
  std::vector<float> pointNKNSquareDistance(k);
//这段代码是对点云进行最近邻搜索，并统计两个点云之间的相交点的数量。
//循环遍历第一个点云c1中的所有点，其中searchPoint是一个迭代器，用于指向当前正在处理的点。

  for (auto searchPoint = c1->begin(); searchPoint != c1->end(); searchPoint++)
  //对于当前的searchPoint，使用kdtree的nearestKSearch方法查找在第二个点云c2中离该点最近的k个点，
  //pointIdxNKNSearch和pointNKNSquareDistance分别用于存储找到的点的索引和距离平方。
  //如果成功找到至少一个最近邻，则进入下一层if语句，否则直接跳过这个点的处理。
    if (kdtree->nearestKSearch(*searchPoint, k, pointIdxNKNSearch,
                               pointNKNSquareDistance) > 0)
  //如果距离最近的点的距离平方小于1e-6，则说明这两个点在空间中相交，因为它们之间的距离平方非常小。
  //这里的1e-6是一个阈值，用于判断点是否在一个平面上或者非常靠近一个表面上，具体值的设定需要根据实际情况调整。
      if (pointNKNSquareDistance[0] < 1e-6)
      //如果满足上述两个条件，则认为这两个点云有一个相交点，将correct计数器加1。
        correct++;
  return correct;
}
//这段代码定义了一个名为eva的函数，
//该函数接受四个参数sp、sn、ep和en，这四个参数都是指向PointCloudT的智能指针。
std::vector<double> eva(const PointCloudT::Ptr &sp, const PointCloudT::Ptr &sn,
                        const PointCloudT::Ptr &ep,
                        const PointCloudT::Ptr &en) {
//函数中调用了名为intersaction的函数，用于计算两个点云之间的交集。
//具体来说，该函数分别计算了真阳性（True Positive, TP）、假阳性（False Positive, FP）、真阴性（True Negative, TN）和假阴性（False Negative, FN）的数量。
  int tn = intersaction(sn, en);
  int fn = intersaction(sp, en);
  int fp = intersaction(sn, ep);
  int tp = intersaction(sp, ep);
//这些计算结果被用于计算精度（precision）、召回率（recall）、准确率（accuracy）、真阳性率（True Positive Rate, TPR）、假阳性率（False Positive Rate, FPR）和假阴性率（False Negative Rate, FNR）。
//最后，这些结果被存储在一个vector中，并被返回。
  double accuracy = static_cast<double>(tp + tn) / (tp + tn + fp + fn);
  double precision = static_cast<double>(tp) / (tp + fp);
  double recall = static_cast<double>(tp) / (tp + fn);
  double tp_rate = static_cast<double>(tp) / sp->size();
  double fp_rate = static_cast<double>(fp) / sn->size();
  double fn_rate = static_cast<double>(fn) / sp->size();

  std::vector<double> result{precision, recall,  accuracy,
                             tp_rate,   fp_rate, fn_rate};
  return result;
}

#endif // INCLUDE_EVA_HH