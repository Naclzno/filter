//此文件是DROR类的定义

#include <gflags/gflags.h>
#include <pcl/io/pcd_io.h>
#include <dror.hh> //引用了DROR类的头文件
//该行代码使用了别名定义（alias declaration），将 pcl::PointXYZI 类型命名为 PointT，以便在后续代码中使用更为简洁的名称。
using PointT = pcl::PointXYZI;
using PointCloudT = pcl::PointCloud<PointT>;
//使用了gflags库中的the DEFINE_ macros定义了几个变量，这些变量允许为程序设置命令行arguments
//该行代码使用了 gflags 库中的 DEFINE_string 宏定义了一个名为 f 的字符串变量，用于指定原始点云文件的路径。
//第二个参数 "" 表示该变量的默认值为空字符串，第三个参数是一个字符串，用于在程序使用 --help 命令时显示该变量的用途。
DEFINE_string(f, "", "origin_cloud");

DEFINE_string(ep, "./ep.pcd", "ep path");
DEFINE_string(en, "./en.pcd", "en path");
DEFINE_int32(n, 3, "min_neighbours");
DEFINE_double(m, 3, "radius_multiplier");
DEFINE_double(a, 0.2, "azimuth_angle");
DEFINE_double(r, 0.04, "min_search_radius");

int main(int argc, char* argv[]) {
//该行代码使用了 gflags 库中的 ParseCommandLineFlags 函数，将程序运行时的命令行参数进行解析，并将解析结果存储到之前定义的变量中。
//第一个参数 &argc 是指向 argc 的指针，表示命令行参数的个数；第二个参数 &argv 是指向 argv 的指针，表示命令行参数的具体内容；
//第三个参数为 true，表示在解析过程中移除所有的标志参数。
  gflags::ParseCommandLineFlags(&argc, &argv, true);
//程序定义了三个PointCloudT指针，分别命名为origin、en和ep
//该行代码定义了一个名为 origin 的 PointCloudT 指针，并将其初始化为一个新的 PointCloudT 对象。PointCloudT::Ptr 表示该指针是指向 PointCloudT 类型的指针，
//new PointCloudT 表示在堆上动态分配一个新的 PointCloudT 对象，并将其地址赋值给 origin。
  PointCloudT::Ptr origin(new PointCloudT);
  PointCloudT::Ptr en(new PointCloudT);
  PointCloudT::Ptr ep(new PointCloudT);
//并使用pcl::io::loadPCDFile函数从命令行参数FLAGS_f指定的文件中加载点云到origin点云。
//这行就是读取数据集的数据。
  pcl::io::loadPCDFile(FLAGS_f, *origin);
//然后创建一个DROR类的对象，并使用其各种Set函数对命令行参数进行初始化。
//这部分其实就是设置DROR滤波器的参数。
  DROR outrem;
  outrem.SetMinNeighbors(FLAGS_n);
  outrem.SetRadiusMultiplier(FLAGS_m);
  outrem.SetAzimuthAngle(FLAGS_a);
  outrem.SetMinSearchRadius(FLAGS_r);
//然后用原点云调用DROR类的滤波函数，并将得到的滤波点云和噪声点云分别存储在en和ep中。
  outrem.Filter(origin, *en, *ep);
//最后，使用pcl::io::savePCDFileBinary函数将得到的点云保存到由FLAGS_en和FLAGS_ep命令行参数指定的文件中。
  pcl::io::savePCDFileBinary(FLAGS_ep, *ep);
  pcl::io::savePCDFileBinary(FLAGS_en, *en);
  return 0;
}