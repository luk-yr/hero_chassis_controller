# hero_chassis_controller

基于 ROS Noetic 和 ros_control 的底盘控制器，用于 Gazebo 仿真。

## 开发环境

- Ubuntu 20.04、ROS Noetic（Docker 容器内）
- Gazebo 11
- CLion、C++14、catkin_tools

## 当前进度

已实现四个独立的 `control_toolbox::Pid`。每个周期读取各轮实际角速度，
用“目标角速度 − 实际角速度”计算误差，再将 PID 输出作为该轮的力矩指令。
目前四轮使用同一个目标角速度，单位为 rad/s；没有实现底盘逆运动学。

已在 Gazebo 中验证编译、插件加载、四轮速度跟踪和在线修改 PID 参数。
当前初始参数为 `p=0.5`、`i=0.1`、`d=0.0`，微分项暂未启用。
在当前模型的正、反向 2 rad/s 测试中，末段四轮平均速度均接近目标；
正向启动峰值约为 2.42 rad/s。实际轮速仍有周期波动，这组参数作为初步可用值，
仍需结合曲线学习和调整。

## 构建

在已加载 ROS 环境的 CLion 中打开本包的 `CMakeLists.txt`，构建
`hero_chassis_controller` 目标。

## 仿真与速度指令

以下命令在当前 Docker 容器内执行。当前 CLion 的构建输出位于包的私有 devel 目录，
运行前先加载该环境：

```bash
source /home/ros/ROS-ws/devel/.private/hero_chassis_controller/setup.bash
roslaunch hero_chassis_controller pid_test.launch
```

启动文件加载任务底盘模型、本包控制器和关节状态发布控制器。
另开一个已加载上述环境的终端，给四轮发送 2 rad/s 的目标：

```bash
rostopic pub -r 10 /hero_chassis_controller/command std_msgs/Float64 "data: 2.0"
```

停止发布不会清除最后一次目标。需要停车时，先按 Ctrl+C 结束上述命令，再发送零目标：

```bash
rostopic pub -1 /hero_chassis_controller/command std_msgs/Float64 "data: 0.0"
```

## 查看曲线与调整参数

```bash
rosrun rqt_plot rqt_plot /hero_chassis_controller/command/data \
  '/joint_states/velocity[0]' '/joint_states/velocity[1]' \
  '/joint_states/velocity[2]' '/joint_states/velocity[3]'
rosrun rqt_reconfigure rqt_reconfigure
```

两个图形工具需分别在终端中启动。当前模型的 `joint_states` 顺序为左后、左前、右后、右前；
若更换模型或发布方式，先核对消息中的 `name`，再使用对应下标。

参数窗口中展开 `/hero_chassis_controller`，每个轮子的 `pid` 都有独立的 `p`、`i`、`d` 等参数。
例如左前轮为 `/hero_chassis_controller/front_left/pid`。
在线调整只影响本次运行；要保存结果，需将调整后的值写回 `config/default.yaml`。
该文件也保存初始目标角速度 `target_velocity`，默认值为零。

## 实现参考

- [ros_control 官方轮速控制器源码](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/effort_controllers/src/joint_velocity_controller.cpp)：读取速度、计算误差、发送力矩的调用方式。
- [control_toolbox 官方 PID 源码](https://github.com/ros-controls/control_toolbox/blob/noetic-devel/src/pid.cpp)：PID 参数读取、状态发布和动态调参接口。

## 许可证

沿用模板的 [BSD 3-Clause 许可证](LICENSE)。
