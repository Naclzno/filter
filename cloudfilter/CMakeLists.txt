#这段 CMake 代码主要用于设置编译环境和参数。

cmake_minimum_required(VERSION 3.12.0) #设置项目需要CMake的最低版本，其实，该命令的作用就是通过设置版本号，设置编译器采用的相关编译策略，即调用cmake_policy(version)
project(LIDSOR) #设置项目/工程名称
#set()命令定义一个变量;set()命令可以设置普通变量、cmake缓存变量、环境变量
#格式：set(<变量名> <变量值>)
#引用一个变量  格式：${变量名}
set(CMAKE_CXX_STANDARD 17)#该行代码指定编译器使用 C++17 标准来编译代码，CMake 会在生成 Makefile 或其他构建系统时将该标准传递给编译器。
set(CMAKE_CXX_STANDARD_REQUIRED ON)#该行代码指定 C++17 标准是必须的，如果编译器不支持 C++17 标准，CMake 会生成错误。
set(CMAKE_BUILD_TYPE Release) #该行代码指定编译类型为 Release，这意味着生成的可执行文件将优化代码以提高执行速度
#该行代码指定生成可执行文件的输出目录为 ${CMAKE_SOURCE_DIR}/bin，即源代码目录下的 bin 子目录。
#这意味着在编译完成后，可执行文件将被放置在 ${CMAKE_SOURCE_DIR}/bin 目录下。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

#总之，这些 CMake 代码的作用是在编译时指定了 C++17 标准，强制使用该标准，指定了编译类型为 Release，并将生成的可执行文件输出到 ${CMAKE_SOURCE_DIR}/bin 目录下。

#这段 CMake 代码主要用于查找和配置 PCL 和 gflags 库，设置源代码目录和测试代码目录，以及添加编译器选项。

#该行代码使用 find_package 命令查找并配置 PCL 库，它会检查系统上是否已安装 PCL 库，并设置必要的编译器选项和链接选项。
#REQUIRED 关键字表示如果找不到 PCL 库，将发生错误。
find_package(PCL REQUIRED) 
#同理，只不过是 gflags库。
find_package(gflags REQUIRED)
#该行代码定义了源代码目录为 ${CMAKE_SOURCE_DIR}/src
#其中 ${CMAKE_SOURCE_DIR} 是 CMakeLists.txt 所在的根目录。
set(CODE_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
#同理，该行代码定义了测试代码目录
set(TEST_SOURCE_DIR ${CMAKE_SOURCE_DIR}/test)
#该行代码添加 PCL 定义选项到编译器选项中，这些选项通常包含 PCL 库的头文件路径和其他必要的宏定义。
add_definitions(${PCL_DEFINITIONS}) #在编译源文件时添加-D定义标志；add_definitions的功能和C/C++中的#define（预处理命令的一种，在预处理过程中进行简单字符串替换，不做正确性检查）是一样的
#该行代码将源代码目录添加到编译器的头文件搜索路径中，这意味着编译器将在 ${CMAKE_SOURCE_DIR}/src 目录下查找头文件。
include_directories(${CODE_SOURCE_DIR}) #添加头文件目录；将给定的目录添加到编译器用于搜索包含文件的目录中。相对路径解释为相对于当前源目录。

#总之，这些 CMake 代码的作用是查找和配置 PCL 和 gflags 库，设置源代码目录和测试代码目录，添加编译器选项，使编译器能够正确地编译项目。

#这段 CMake 代码主要用于创建一个可执行文件，并设置编译选项和链接选项。

# Unit Test
## IOR Test
set(test_IOR ior)#该行代码定义了一个名为 test_IOR 的变量，并将其设置为字符串 ior。
#该行代码使用 add_executable 命令创建一个可执行文件，文件名为 test_IOR，源文件为 ${TEST_SOURCE_DIR}/test_ior.cc。
#${TEST_SOURCE_DIR} 是前面设置的测试代码目录。
add_executable(${test_IOR} 
${TEST_SOURCE_DIR}/test_ior.cc
)
#该行代码使用 target_include_directories 命令将 ${PCL_INCLUDE_DIRS} 添加到 test_IOR 目标的包含目录中。
#${PCL_INCLUDE_DIRS} 是前面通过 find_package(PCL REQUIRED) 命令查找并配置的 PCL 库的头文件路径。
target_include_directories(${test_IOR} PRIVATE #向目标添加include directories
  ${PCL_INCLUDE_DIRS}
)
#该行代码使用 target_link_libraries 命令将 ${PCL_LIBRARIES} 和 ${GFLAGS_LIBRARIES} 添加到 test_IOR 目标的链接库列表中。
#${PCL_LIBRARIES} 和 ${GFLAGS_LIBRARIES} 分别是前面通过 find_package(PCL REQUIRED) 和 find_package(gflags REQUIRED) 命令查找并配置的 PCL 库和 gflags 库的链接库路径。
target_link_libraries(${test_IOR} #在连接一个给定的目标 和/或 其附属物时指定要使用的库或标志
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

#总之，这些 CMake 代码的作用是创建一个名为 test_IOR 的可执行文件，并将必要的编译器选项和链接选项添加到该目标中，使得编译器能够正确地编译和链接该文件。


## DROR Test
set(test_DROR dror)
add_executable(${test_DROR}
  ${TEST_SOURCE_DIR}/test_dror.cc
)
target_include_directories(${test_DROR} PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${test_DROR}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

## DSOR Test
set(test_DSOR dsor)
add_executable(${test_DSOR}
  ${TEST_SOURCE_DIR}/test_dsor.cc
)
target_include_directories(${test_DSOR} PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${test_DSOR}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

## EVA Test
set(test_EVA eva)
add_executable(${test_EVA}
  ${TEST_SOURCE_DIR}/test_eva.cc
)
target_include_directories(${test_EVA} PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${test_EVA}
  ${PCL_LIBRARIES}
  ${GFLAGS_LIBRARIES}
)

## Timer Test
set(test_Timer timer)
add_executable(${test_Timer}
  ${TEST_SOURCE_DIR}/test_timer.cc
)