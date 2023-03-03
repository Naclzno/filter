#ifndef INCLUDE_IOR_HH
#define INCLUDE_IOR_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloudT = pcl::PointCloud<pcl::PointXYZI>;

class IOR {
public:
  IOR() = default;
  ~IOR() = default;

  void SetIntensityThreshold(const int32_t intensity_threshold) {
    this->intensity_threshold_ = intensity_threshold;
  }

  int8_t GetIntensityThreshold() { return this->intensity_threshold_; }

  void Filter(PointCloudT::Ptr &input_cloud, PointCloudT &filtered_cloud, PointCloudT &noise_cloud) {
    for (auto point = input_cloud->begin(); point != input_cloud->end(); point++) {
      if (point->intensity > this->intensity_threshold_)
        filtered_cloud.push_back(*point);
      else
        noise_cloud.push_back(*point);
    }
  }

private:
  uint8_t intensity_threshold_;
};

#endif // INCLUDE_IOR_HH