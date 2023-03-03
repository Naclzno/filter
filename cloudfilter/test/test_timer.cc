#include <timer.hh>
#include <cstdio>
#include <unistd.h>

int main(int argc, char *argv[]) {
//具体来说，程序中先创建了一个Timer对象timer，这时Timer类的构造函数就会自动启动计时器，
//并记录下开始时间start_point和自动启动标记autostart。
  Timer timer;
  printf("Timer Test.\n");
//接着，程序调用sleep(1)函数暂停1秒，模拟一些耗时的操作。
  sleep(1);
//然后，程序调用timer.Get()函数来获取从start_point到现在的时间间隔，即程序的执行时间。
//在Timer类的Get()函数中，如果autostart标记为true，则end_point会被更新为当前时间，否则直接使用end_point记录的时间。
//函数返回的是以纳秒为单位的时间间隔。
//最后，程序将获取到的时间间隔通过printf()函数以毫秒为单位输出。
//这样，我们就可以在程序中加入任意多的计时点来测量代码的执行时间，从而进行性能优化。
  printf("%.2f ms\n", timer.Get() * 1e-6);

  return 0;
}