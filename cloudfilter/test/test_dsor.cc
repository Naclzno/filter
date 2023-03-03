#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <dsor.hh>

using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;

DEFINE_string(f, "", "origin_cloud");
DEFINE_string(ep, "./ep.pcd", "ep path");
DEFINE_string(en, "./en.pcd", "en path");
DEFINE_int32(k, 50, "min_k_neighbours");
DEFINE_double(m, 0.15, "standard_deviation_multiplier");
DEFINE_double(r, 0.05, "range_multiplier");

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PointCloudT::Ptr origin(new PointCloudT);
  PointCloudT::Ptr en(new PointCloudT);
  PointCloudT::Ptr ep(new PointCloudT);
  pcl::io::loadPCDFile(FLAGS_f, *origin);

  DSOR outrem;
  outrem.SetMeanK(FLAGS_k);
  outrem.SetStandardDeviationMultiplier(FLAGS_m);
  outrem.SetRangeMultiplier(FLAGS_r);
  outrem.Filter(origin, *en, *ep);

  pcl::io::savePCDFileBinary(FLAGS_ep, *ep);
  pcl::io::savePCDFileBinary(FLAGS_en, *en);
  return 0;
}