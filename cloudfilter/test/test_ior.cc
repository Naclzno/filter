#include <gflags/gflags.h>
#include <ior.hh>
#include <pcl/io/pcd_io.h>

DEFINE_string(f, "", "origin_cloud");
DEFINE_string(ep, "./ep.pcd", "ep path");
DEFINE_string(en, "./en.pcd", "en path");
DEFINE_uint32(it, 5, "intensity_threshold");

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PointCloudT::Ptr origin(new PointCloudT);
  PointCloudT::Ptr en(new PointCloudT);
  PointCloudT::Ptr ep(new PointCloudT);
  pcl::io::loadPCDFile(FLAGS_f, *origin);

  IOR outrem;
  outrem.SetIntensityThreshold(FLAGS_it);
  outrem.Filter(origin, *en, *ep);

  pcl::io::savePCDFileBinary(FLAGS_ep, *ep);
  pcl::io::savePCDFileBinary(FLAGS_en, *en);
  return 0;
}
