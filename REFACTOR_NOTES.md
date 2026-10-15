# 目录重构说明

## 修改范围

- inter：PX4Interface、坐标变换的源文件和头文件迁入 inter。
- ctrl：控制器、轨迹管理器、算法和所有采样器迁入 ctrl；保留原有空采样器文件和 trajectory_sampler_base.hpp 名称。
- 三个入口迁入 src/main，分别为 drone_node_main.cpp、inter_main.cpp、ctrl_main.cpp。
- attitude_test.cpp 迁入 test/inter。
- types.hpp 保持为 include/drone_node/types.hpp，作为公共类型头文件。
- 更新受影响的 include 路径，目录结构以仓库中的实际目录为准。
- CMakeLists.txt 按参考文档组织 INTER_MSG_FILES、CTRL_MSG_FILES、INTER_SOURCES 和 CTRL_SOURCES，保留全部原依赖、typesupport、Eigen3 和 Threads 链接，以及四个原可执行文件名。
- config 安装位置仍为 share/drone_node/config；增加参考文档中的 launch 安装规则，现有空 launch 文件内容保持不变。
- 保留 drone_node.yaml、drone_node_test.yaml；inter.yaml 原样复制 drone_node.yaml，ctrl.yaml 提取 drone_node_test.yaml 的 flight_controller 配置。已有参数文件路径继续有效。
- package.xml、消息内容、原配置和运行逻辑不变。
- 预留 src/motion_executor/、include/drone_node/motion_executor/、action/、test/ctrl/ 和 test/motion_executor/。空目录不由 Git 跟踪，未添加占位类或 Action。

## 验证结果

- 已逐文件验证全部原源码，除映射后的 include 路径外内容不变；消息、原 YAML 和 launch 内容不变。
- CMake 列出的源码和消息路径全部存在；非生成的 drone_node 头文件引用全部可解析。
- 四个目标仍为 drone_node_exec、px4_interface_exec、flight_controller_exec、attitude_test。
- 联合入口仍创建 PX4Interface 和 FlightController，保留两个独立 Executor、线程和原 NodeOptions。
- 未进行功能性修改，未实现 motion_executor。
- 尚未完成编译验证：Windows 环境没有 colcon/ROS2，WSL 未安装；尝试 SSH 连接共享目录所在 Linux 主机时认证失败（Permission denied）。静态检查不等同于编译通过。

请在已有 ROS2 环境中加载工作区依赖后，从工作区根目录运行：

```bash
colcon build --packages-select drone_node --symlink-install
```

本次停止在目录重构阶段。
