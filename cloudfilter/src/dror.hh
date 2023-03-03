//此文件是DROR类的头文件
//下面两行定义了include guards，以防止文件被多次包含
#ifndef INCLUDE_DROR_HH
#define INCLUDE_DROR_HH
//引用pcl库需要的头文件，这些行包含了点云库( PCL )中使用Kd - tree搜索算法和处理点云所需的headers 。
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DROR {
public:
//是一个默认的构造函数，用于创建一个没有任何参数的DROR类的实例
//这一行表示该构造函数是默认定义的，并且有一个空的实现。这是必要的，因为在实现Filter方法时，DROR类被实例化，没有任何参数。
  DROR() = default; 

//析构函数 作用：回收/撤销对象
//它是一个默认的析构器，用于清理在DROR类实例的生命周期中分配的资源。
//这一行表示这个析构器是默认定义的，并且有一个空的实现，意味着这个类不需要特定的清理。
  ~DROR() = default; 

//使用=default是为这些函数提供实现的一种选择。使用默认函数的好处是，它减少了模板代码，并有助于避免在自定义实现中可能发生的常见错误。

//这里不仅声明了类，而且直接定义了类，在main函数当中只要创建类对象就ok了
//将形参的数值传递给对应的私有变量
//这四个方法是DROR类的公共成员函数，用于设置私有成员变量的值：radius_multiplier_、azimuth_angle_、min_neighbors_和min_search_radius_。
  void SetRadiusMultiplier(double radius_multiplier) {
    radius_multiplier_ = radius_multiplier;//将等于号右边的值赋值给左边
  }
  void SetAzimuthAngle(double azimuth_angle) { azimuth_angle_ = azimuth_angle; }
  void SetMinNeighbors(int min_neighbors) { min_neighbors_ = min_neighbors; }
  void SetMinSearchRadius(double min_search_radius) {
    min_search_radius_ = min_search_radius;
  }
//这四个方法是DROR类的公共成员函数，用于获取私有成员变量的值。
  double GetRadiusMultiplier() { return radius_multiplier_; }
  double GetAzimuthAngle() { return azimuth_angle_; }
  int GetMinNeighbors() { return min_neighbors_; }
  double GetMinSearchRadius() { return min_search_radius_; }

//template <typename T> 是C++中用于定义模板的固定格式。
//模板是实现代码重用机制的一种工具，它可以实现数据类型参数化，即把数据类型定义为参数， 从而实现了真正的代码可重用性。
//这是一个DROR类的模板成员函数，它接受三个点云参数作为输入：输入点云、过滤点云和噪声点云。模板类型T代表点云的点类型。
  template <typename T>
//DROR类中比较重要的一部分，公共的滤波器函数
//input_cloud是一个指向正在被过滤的输入点云的指针，而filtered_cloud和noise_cloud则是分别包含过滤点和噪声点的输出点云。通过使用对这些对象的引用，Filter函数可以直接修改这些点云的内容，而不必为它们创建新的副本。
//pcl::PointCloud是一个模板类，定义了一个点云对象，T是一个模板参数，决定了点云的类型。 ::Ptr是pcl::PointCloud的一个成员，是一个共享指针的类型定义，用于指向点云类型。
//所以，typename pcl::PointCloud<T>::Ptr是一个指向T类型的pcl::PointCloud对象的共享指针。符号&表示input_cloud是这个共享指针的一个引用。换句话说，input_cloud是对指向T类型的pcl::PointCloud对象的一个指针的引用。
  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud, //& 将引用用作函数参数
              typename pcl::PointCloud<T> &noise_cloud) {
  
  // 这几行定义了一个KdTreePtr类型，它是指向点类型为T的Kd - tree搜索树的指针。
 
  
  // 通过using指定别名，eg： using T=int； 用T代替int
  // 用KdTreePtr代替typename pcl::KdTreeFLANN<T>::Ptr
  
  //using关键字定义了一个类型别名KdTreePtr，KdTreePtr类型被定义为一个指向pcl::KdTreeFLANN<T>对象的智能指针
  //typename关键字用来告诉编译器pcl::KdTreeFLANN<T>::Ptr是一个类型，而不是一个变量或一个函数
  using KdTreePtr = typename pcl::KdTreeFLANN<T>::Ptr;
  //这一行在执行过滤操作之前清除了filtered_cloud对象的内容。这是必要的，因为Filter方法将过滤后的点附加到这个云中，而不是替换其内容。在每次迭代开始时清除filtered_cloud，可以确保它只包含满足过滤条件的点。
  filtered_cloud.clear();
  
  //new的功能：Ⅰ分配空间 Ⅱ调用构造函数
  //这一行创建了一个新的pcl::KdTreeFLANN<T>对象，并用input_cloud对其进行初始化。new操作符为kd_tree_对象分配了内存，并用input_cloud来构造它。
  //KdTreePtr类型的别名被用来定义kd_tree_变量的类型。
  KdTreePtr kd_tree_(new pcl::KdTreeFLANN<T>());
  //该行将kd_tree_对象的输入云设置为input_cloud
  //setInputCloud方法是pcl::KdTreeFLANN类的一个成员函数，用于建立半径搜索操作中使用的搜索树。这一行是必要的，因为搜索树需要使用input_cloud来建立，以执行搜索操作。
  //kd_tree_ is a pointer to a KdTreeFLANN object
  //The setInputCloud() function is a member function of the KdTreeFLANN class，将一个指向点云的指针作为参数，并将其设置为KdTree的输入云。
  //->操作符被用来访问kd_tree_所指向的KdTreeFLANN对象的setInputCloud（）成员函数，并将input_cloud指针作为参数传递给该函数
  //This sets the input_cloud as the input cloud of the KdTree.
  kd_tree_->setInputCloud(input_cloud);

  // Go over all the points and check which doesn't have enough neighbors
  // perform filtering
  //typename pcl::PointCloud<T>::iterator是一个迭代器，可以用来迭代T类型的pcl::PointCloud对象的元素
  //在本例中，it是一个迭代器，用于迭代input_cloud中的点，它是一个指向点云的指针。
  for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    //这将提取循环中正在处理的当前点的x和y坐标。
    //一个点的x和y成员是用->操作符访问的。
    //it->x表示当前迭代器所指向的点的x成员被访问并分配给变量x_i。同样地，it->y意味着点的y成员被访问并分配给变量y_i。
    float x_i = it->x;
    float y_i = it->y;
    //pow(x_i, 2)代表的是前者的平方
    float range_i = sqrt(pow(x_i, 2) + pow(y_i, 2));
    //M_PI 是一个宏定义，圆周率的定义  #define M_PI 3.14159265358979323846
    //角度转弧度 π/180×角度；弧度变角度 180/π×弧度
    //此代码是角度转弧度
    float search_radius_dynamic =
        radius_multiplier_ * azimuth_angle_ * M_PI / 180 * pow(range_i / 5, 3);

    if (search_radius_dynamic < min_search_radius_) {
      search_radius_dynamic = min_search_radius_;
    }
//pointIdxRadiusSearch和pointRadiusSquaredDistance被用来分别存储邻居的数量和距离
    std::vector<int> pointIdxRadiusSearch;//该行代码定义了一个名为 pointIdxRadiusSearch 的整型向量，用于存储搜索半径内的所有点的索引。
    std::vector<float> pointRadiusSquaredDistance;//该行代码定义了一个名为 pointRadiusSquaredDistance 的浮点型向量，用于存储每个点与查询点之间的距离的平方。
//得到的neighbors数量被储存在neighbors变量中
    int neighbors =
        kd_tree_->radiusSearch(*it, search_radius_dynamic, pointIdxRadiusSearch,
                               pointRadiusSquaredDistance);
//该行代码调用了 Kd 树的 radiusSearch 函数，在点云中搜索距离查询点 *it 小于等于 search_radius_dynamic 半径内的所有点
//并将结果存储在 pointIdxRadiusSearch 和 pointRadiusSquaredDistance 向量中。
//neighbors 是搜索到的点的数量。其中 kd_tree_ 是一个指向 Kd 树的指针，具体实现可以参考 PCL 的文档说明。需要注意的是，由于 radiusSearch 函数是一个计算密集型任务，可能会占用较多的计算资源和时间。
    if (neighbors >= min_neighbors_) {
    //添加到过滤后的点云中
      filtered_cloud.push_back(*it);
    } else {
    //添加到噪声点云中
      noise_cloud.push_back(*it);
    }
  }
}

//DROR类中定义的私有变量，保密性好
private:
  double radius_multiplier_;  //乘法因数
  double azimuth_angle_; //激光雷达的水平角度分辨率
  int min_neighbors_; //最小邻点数
  double min_search_radius_; //最小搜索半径
};

#endif // INCLUDE_DROR_HH