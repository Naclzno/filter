#ifndef INCLUDE_DSOR_HH
#define INCLUDE_DSOR_HH

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class DSOR {
public:
//在构造函数中，使用了 = default 的语法，表示使用默认的构造函数实现。
//在 C++11 中，为了提高代码的可读性和简洁性，引入了默认函数的概念。默认函数指的是使用编译器自动生成的函数实现，其功能和默认情况下编译器生成的函数相同。
//在类的定义中使用 = default 可以显式地声明使用默认函数实现某个函数，这样可以避免手动编写函数实现，同时也方便代码的阅读和维护。
//  DSOR() 和 ~DSOR() 函数都被声明为使用默认实现，因此编译器会自动生成相应的代码。这些函数的默认实现通常与不使用 = default 的情况下相同。
//使用默认函数的好处是可以使代码更加简洁，同时也可以避免因手动编写函数实现导致的错误。
  DSOR() = default;
  ~DSOR() = default;
//设置DSORFilter类中的mean_k_成员变量的值，而参数mean_k的值就是用来更新mean_k_的值。
//this->mean_k_表示当前对象中的mean_k_成员变量。
//this->的作用是访问当前对象的成员，表示更新当前对象中的mean_k_成员变量的值为传入的参数mean_k。
  void SetMeanK(const int mean_k) { this->mean_k_ = mean_k; }
  void SetStandardDeviationMultiplier(const double std_mul) {
    this->standard_deviation_multiplier_ = std_mul;
  }
  void SetRangeMultiplier(const double range_multiplier) {
    this->range_multiplier_ = range_multiplier;
  }

  int GetMeanK() { return this->mean_k_; }
  double GetStandardDeviationMultiplier() {
    return this->standard_deviation_multiplier_;
  }
  double GetRangeMultiplier() { return this->range_multiplier_; }

  template <typename T>
  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud,
              typename pcl::PointCloud<T> &noise_cloud) {
    // initialize kd search tree
    //定义一个pcl::KdTreeFLANN<T>类型的kd_tree对象，该对象是一个KdTree，用于在点云中进行最近邻搜索。
    typename pcl::KdTreeFLANN<T> kd_tree;
    //调用setInputCloud()函数将待搜索的点云input_cloud作为输入传递给kd_tree对象。
    kd_tree.setInputCloud(input_cloud);

    // Allocate enough space to hold the results
    //定义三个vector类型的变量,这些变量将在最近邻搜索中存储结果。
    std::vector<int> pointIdxNKNSearch(this->mean_k_);
    std::vector<float> pointNKNSquaredDistance(this->mean_k_);
    std::vector<float> mean_distances;

    // Go over all the points and check which doesn't have enough neighbors
    // perform filtering
    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      // k nearest search
      //这里使用了KdTreeFLANN类的nearestKSearch函数，对当前点（*it）进行最近的K个邻居搜索，K的值是this->mean_k_，搜索结果存储在pointIdxNKNSearch和pointNKNSquaredDistance中。
      kd_tree.nearestKSearch(*it, this->mean_k_, pointIdxNKNSearch,
                             pointNKNSquaredDistance);
      //这里计算了当前点与其K个最近邻点的欧式距离，将其加起来存储在dist_sum中。
      // calculate mean distance
      double dist_sum = 0;
      for (int j = 1; j < this->mean_k_; ++j) {
        dist_sum += sqrt(pointNKNSquaredDistance[j]);
      }
      //这里将当前点与其K个最近邻点的平均距离存储在mean_distances向量中，作为该点的密度估计值。
      //注意要减1，因为点本身也算在K个邻居中，但是计算平均距离时应该排除该点。
      mean_distances.push_back(
          static_cast<float>(dist_sum / (this->mean_k_ - 1)));
    }
    //这段代码是用来计算一组数据的均值(mean)、方差(variance)和标准差(stddev)的。
    // Estimate the mean and the standard deviation of the distance vector
    double sum = 0, sq_sum = 0;
    //从0循环到mean_distances的大小，即对mean_distances中的每个元素进行处理。
    for (size_t i = 0; i < mean_distances.size(); ++i) {
      sum += mean_distances[i];//累加每个元素的值到sum中。
      sq_sum += mean_distances[i] * mean_distances[i];//累加每个元素的平方到sq_sum中。
    }
    //计算均值，即将sum除以mean_distances的大小，并将结果赋给mean。
    //使用static_cast将mean_distances的大小转换成double类型。
    double mean = sum / static_cast<double>(mean_distances.size());
    //计算方差，即先计算平方和的平均值，然后减去均值的平方，
    //再除以n-1。使用static_cast将mean_distances的大小转换成double类型。
    double variance =
        (sq_sum - sum * sum / static_cast<double>(mean_distances.size())) /
        (static_cast<double>(mean_distances.size()) - 1);
    //计算标准差，即将方差的平方根赋给stddev。
    double stddev = sqrt(variance);

    // PCL_INFO("mean: %lf var: %lf\n", mean, variance);
    // calculate distance threshold (PCL sor implementation)

    //这行代码定义了一个double类型的变量distance_threshold(全局阈值)，它的值由三部分组成：
    //mean是mean_distances中所有元素的平均值；
    //this->standard_deviation_multiplier_是一个用户设置的标准差倍数，表示标准差的倍数，通常为2或3；
    //stddev是mean_distances中所有元素的标准差。

    double distance_threshold =
        (mean + this->standard_deviation_multiplier_ * stddev);


    // iterate through vector
    int i = 0;

    for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
         it != input_cloud->end(); ++it) {
      // calculate distance of every point from the sensor
      //计算当前点到传感器的距离，即点的(x, y, z)坐标的欧几里得范数。
      float range = sqrt(pow(it->x, 2) + pow(it->y, 2) + pow(it->z, 2));
      // dynamic threshold: as a point is farther away from the sensor,
      // the threshold increases
      //根据全局阈值(distance_threshold)、距离乘法因子(this->range_multiplier_)和点到传感器的距离(range)计算动态阈值。
      double dynamic_threshold =
          distance_threshold * this->range_multiplier_ * range;
      // PCL_INFO("dynamic threshold: %lf\n", dynamic_threshold);

      // a distance lower than the threshold is an inlier
      //push_back()是向其尾部添加一个元素。
      if (mean_distances[i] < dynamic_threshold) {
        filtered_cloud.push_back(*it);
      } else {
        noise_cloud.push_back(*it);
      }
      // update iterator
      i++;
    }
  }

private:
  int mean_k_;//最近邻点的最小值
  double standard_deviation_multiplier_;//标准差乘法因子
  double range_multiplier_;//距离乘法因子
};

#endif // INCLUDE_DSOR_HH