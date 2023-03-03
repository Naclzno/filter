//这两行代码是头文件保护，防止头文件被重复引用，只有在该头文件未被引用时才会被编译。
#ifndef INCLUDE_TIMER_HH
#define INCLUDE_TIMER_HH
//C++11 引入了一个新的标准库 chrono 用于时间计算。
#include <chrono>

class Timer {
public:
//该类的构造函数，可以传入一个布尔型参数，表示是否自动开始计时。默认值为 true。
  Timer(bool flag = true) {
    this->autostart = flag;//代码首先将autostart成员变量设置为flag参数的值，表示是否需要自动开始计时。
//如果autostart的值为true，则调用std::chrono::steady_clock::now()获取当前时间
//并将其赋值给start_point成员变量，即记录开始计时的时间点。
    if (autostart)
      this->start_point = std::chrono::steady_clock::now();
  }
  
  ~Timer() = default;
//开始计时的函数，记录当前时间点。
  void Begin() { this->start_point = std::chrono::steady_clock::now(); }
//结束计时的函数，记录结束时间点。
  void End() {
    this->autostart = false;
    this->end_point = std::chrono::steady_clock::now(); }

//获取代码执行时间的函数，返回类型为 double 类型，表示纳秒数。
//这段代码是 Timer 类的 Get() 成员函数的实现，它用于获取经过的时间。

  double Get() {
//该函数首先检查 autostart 是否为真，如果为真则表示计时器之前处于自动启动模式，
//即构造函数中传递了 true 参数，因此 end_point 属性需要设置为当前时间。
    if(this->autostart)
      this->end_point = std::chrono::steady_clock::now();
//接下来，函数使用 std::chrono 库中的 duration_cast() 函数计算出 end_point 和 start_point 之间的时间间隔，并将其转换为纳秒数，最后返回纳秒数。
    auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
//duration_cast() 函数是用来将时间间隔转换为特定的时间单位的函数，如 std::chrono::nanoseconds 表示的就是纳秒。
        this->end_point - this->start_point);
    return time.count();
  }

private:
//记录开始时间点的私有成员变量，使用 std::chrono::steady_clock::time_point 类型。
  std::chrono::steady_clock::time_point start_point;
//记录结束时间点的私有成员变量，使用 std::chrono::steady_clock::time_point 类型。
  std::chrono::steady_clock::time_point end_point;
//表示计时器是否自动开始的私有成员变量。
  bool autostart;
};

#endif // INCLUDE_TIMER_HH