# rm-controls 大观

rm-controls 是一套面向 RoboMaster 机器人的完整控制系统框架，构建在 ROS 1 Noetic 的 `ros_control` 之上。它不是零散的控制器集合，而是一个有明确架构哲学的分层系统。

## 1. 什么是无下位机

通常，RoboMaster 机器人的软件架构采用上位机（运算平台，如 NUC/Manifold）加下位机（MCU，如 STM32）的模式。下位机负责需要实时性的控制链路，上位机负责决策、导航、视觉等非实时、高算力的任务。

然而，下位机开发面临开发调试效率低、 RAM / Flash 有限等痛点。为了解决这些痛点，rm-controls 采用了**无下位机架构**：下位机不再执行控制算法，仅作为传感器与执行器数据的透明转发层（接收上位机的控制指令 → 驱动电机/舵机；读取传感器数据 → 发回上位机）；所有控制逻辑（PID、状态估计、运动规划）全部运行在上位机中。

这一架构还带来许多优势：

- **Sim2real 无缝衔接**：控制算法在上位机运行，可直接在 Gazebo、Unity 等仿真器中以完全相同的代码进行调试和验证，大幅降低场地与硬件成本
- **实时可视化与调参**：所有 ROS topic 可通过局域网实时转发，配合 PlotJuggler、rqt 等工具动态观察控制曲线、在线调整参数
- **迭代速度提升**：修改控制算法无需烧录固件，重启节点即可生效，调试周期缩短

## 2. 配置驱动

好比游戏中的按键设置可以在不修改源码的前提下自由映射功能，rm-controls 也采用同样的思想：将系统行为与代码分离。RoboMaster 机器人在机械结构、运动学模型、控制需求上有大量可复用的模式，因此 rm-controls 内置了一系列即插即用的控制器插件。每个机器人只需通过 YAML 配置文件声明所需控制器及其参数，即可组合出一套专有的控制系统，无需为不同底盘或平台重复编写嵌入式代码。

举实际开发中的例子，以下两个场景在传统下位机方案中需要改固件、重新烧录，但在 rm-controls 中只需编辑 URDF / YAML。

### 2.1 调拨盘 offset 解决双发问题

拨盘每次校准后顶到机械限位，但从限位到理想的发弹位置之间有一段固定的角度差。如果这个角度差没设对，就会出现双发（一次出两颗弹）或空发。

调这个 offset 不需要改任何 C++ 代码，只需要改 URDF 里的一个数值：

```xml
<!-- rm_description/urdf/hero/shooter.transmission.urdf.xacro -->
<transmission name="trans_trigger_joint">
  <type>transmission_interface/SimpleTransmission</type>
  <actuator name="trigger_joint_motor">
    <mechanicalReduction>-27.5</mechanicalReduction>
  </actuator>
  <joint name="trigger_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <offset>355</offset>   <!-- ← 调这个值，范围 0 ~ 7 rad -->
  </joint>
</transmission>
```

调法是每 1 rad 试一次，找到大概范围后再微调百分位，连续打 50 发验证。全程不需要动一行 C++。

### 2.2 调云台 pitch 的重力补偿

云台由于重心不在旋转轴上，启动控制器后如果补偿不对，pitch 会往下掉或往上抬。调重力补偿就是改 YAML 里的三个数字，重启控制器后看云台能不能定住：

```yaml
# rm_controllers/hero.yaml
gimbal_controller:
  feedforward:
    gravity: 6.725                    # ← 重力矩补偿值
    enable_gravity_compensation: false
    mass_origin: [0185, 0, 01]  # ← 质心位置
```

把 PID 临时置零，松手观察：云台往下掉 → 加大 `gravity`；往上抬 → 减小 `gravity`。几分钟就能调好，而不需要改下位机代码来加一个重力补偿项。

## 3 架构

rm-controls 的架构可以概括为：

- ROS control 的 hardware_interface 层
- 一套自定义的决策中间件（rm_manual）
- 9 个控制器插件
- 6 个硬件接口
- 一个标定框架

### 3.1 决策层

决策层解决的是 ROS control 上层的问题：

- 操作手交互：遥控器拨杆怎么映射到控制指令？按键按下 / 松开对应什么动作？
- 控制器编排：什么时候启动 `chassis_controller`？什么时候切到 `calibration_controller`？谁来仲裁 joint 冲突？
- 标定流水线：上电后先标定拨盘 → 再标定云台 → 再启动主控制器。这个顺序谁来编排？
- 兵种差异：英雄有云台 + 发射 + 自瞄，工程有机械臂 + 动作服务器，哨兵有导航 + 雷达，无人机有飞控——逻辑完全不同，但底层控制器可以复用

决策层运行在 100Hz 回调中，不参与实时控制循环。它与控制层通过 ROS 话题通信，与硬件层也通过 ROS 话题读取状态（/joint_states、/actuator_states）。

### 3.2 控制层

控制层解决的核心问题：ROS control 的标准 controller 只提供一个空的 `update()` 回调和几个 `JointHandle`，但 RoboMaster 需要复杂的控制算法和状态机。

控制层运行在 **1kHz 实时循环**中（与硬件层同线程），通过 `hardware_interface` 的 C++ 指针直通读写硬件数据，不走 ROS 话题序列化。

9 个控制器插件按用途分为三类：

#### 3.2.1 运动控制

真正参与 1kHz 控制环，向 joint 输出力矩 / 速度：

| 控制器 | 解决什么问题 |
| --- | --- |
| `chassis_controller` | 麦轮 / 舵轮 / 全向轮运动学解算 + 裁判系统功率限幅 + FOLLOW 模式下底盘跟随云台 |
| `gimbal_controller` | 云台串级 PID + 子弹弹道解算（RK4 积分）+ 重力补偿 + 底盘运动前馈 + 自瞄模式 |
| `shooter_controller` | 发射状态机（STOP → READY → PUSH → BLOCK）+ 摩擦轮转速控制 + 卡弹检测 |
| `mimic_joint_controller` | 从动关节跟随主关节位置（如摄像头 image_transmission 跟随 pitch） |

#### 3.2.2 标定与辅助控制

同样输出力矩，但仅在标定或特殊阶段运行：

| 控制器 | 解决什么问题 |
| --- | --- |
| `calibration_controllers` | 增量式编码器电机的自动找零：撞限位 → 检测堵转 → 设 offset → 标记完成 |
| `gpio_controller` | 读取 GPIO 输入（如霍尔传感器）触发标定，输出 GPIO 命令（如电磁铁） |

#### 3.2.3 状态发布

只读硬件数据并发布成 TF 或 topic，不输出力矩：

| 控制器 | 解决什么问题 |
| --- | --- |
| `robot_state_controller` | 把 URDF 关节树发布成 TF，操作 `RobotStateInterface` 供其他控制器实时查询 |
| `orientation_controller` | 把 IMU 数据发布成 TF，供底盘里程计和云台坐标系使用 |
| `tof_radar_controller` | 读取 ToF 测距传感器数据并发布 |

运动控制控制器和状态发布控制器之间的区别，对应 ROS control 中的两种角色：前者通过 `EffortJointInterface` **拥有某个 joint 的写入权**（一个 joint 同时只能被一个控制器写，否则两个控制器都发力矩指令就会打架），后者只通过 `JointStateInterface` **读 joint 状态**，再以 ROS 话题形式发布出去。

每个控制器通过 C++ 继承的方式声明自己需要哪几个硬件接口。以底盘控制器为例：

```cpp
class OmniController
    : public ChassisBase<rm_control::RobotStateInterface,          // 查 TF 树
                         hardware_interface::EffortJointInterface>  // 向关节发力矩指令
{
  // 实现略...
};
```

GPIO 控制器要的不是关节接口，而是 GPIO 的输入输出：

```cpp
class Controller
    : public MultiInterfaceController<rm_control::GpioStateInterface,      // 读 GPIO 引脚
                                       rm_control::GpioCommandInterface>   // 写 GPIO 引脚
{
  // 实现略...
};
```

标定控制器则需要标定专用接口加上力矩输出：

```cpp
class MechanicalCalibrationController
    : public CalibrationBase<rm_control::ActuatorExtraInterface,          // 读写标定状态
                             hardware_interface::EffortJointInterface>    // 向关节发力矩
{
  // 实现略...
};
```

不同的接口组合决定了每个控制器能做什么事、能读写哪些硬件资源。

### 3.3 硬件抽象层

硬件抽象层解决的核心问题：ROS control 的标准 `hardware_interface` 只认识 `JointState` 和 `EffortJoint`，但 RoboMaster 的硬件有大量特殊需求——电机有不同的通信协议（EtherCAT / CAN）、有不同的类型（RM6020 / M3508 / M2006 / 达妙）、有标定状态需要管理、有 GPIO 和 IMU 等非关节外设。

硬件抽象层由 `rm_hw` 和 `rm_ecat_hw` 两个包共同实现，继承 ROS control 的 `RobotHW` 基类，运行在 **1kHz** 循环中（与控制层同线程或独立线程）。

它做了三件事：

#### 3.3.1 封装通信协议

`rm_ecat_hw` 负责 EtherCAT 帧的收发——管理从站拓扑（哪块板、什么协议、挂在哪张网卡上），把 `RmMotor` / `MitMotor` 等不同协议电机封装成统一的 `ActuatorData` 结构体。`rm_hw` 负责 CAN 总线，管理 CAN ID、总线索引、电机类型映射。

#### 3.3.2 提供 6 个自定义 hardware_interface

标准 ROS control 不提供的功能都由自定义接口补上：

| 接口 | 解决的问题 |
| --- | --- |
| `RobotStateInterface` | 让控制器在 1kHz 实时线程中安全读写 TF 数据，而不需要走 ROS 话题（`tf2_ros::Buffer` 不是线程安全的） |
| `ActuatorExtraInterface` | 旁路存储标定状态（`needCalibration`、`calibrated`、`offset`）。标准 JointHandle 没有这些字段，且不应被标定逻辑污染 |
| `GpioStateInterface` / `GpioCommandInterface` | 扩展 ROS control 到 GPIO 设备，供标定控制器读霍尔传感器、供 manual 控制电磁铁 |
| `RmImuSensorInterface` | 提供滤波后的 IMU 姿态四元数（标准 `ImuSensorInterface` 只给原始加速度 / 角速度） |
| `TofRadarInterface` | 提供 ToF 测距数据，不属于 joint 体系 |

#### 3.3.3 管理 Transmission 与标定状态

硬件层负责通过 Transmission 换算：电机原始编码器读数 × 减速比 + offset → 关节弧度；反向把力矩指令 ÷ 减速比 → 电机电流。

标定状态的管理是硬件层和标定控制器协作完成的：

- 硬件层在初始化时读取 YAML 的 `need_calibration` 标志，存入 `ActuatorExtraHandle`
- `write()` 时如果电机未标定，跳过限幅，允许标定控制器自由转动电机
- 硬件层读编码器时自动加上当前 offset，上层 `getPosition()` 拿到的是修正后的值
- 标定控制器完成标定后通过 `setOffset()` 写入 offset，通过 `setCalibrated()` 标记完成

队内现在已经不走 Linux 到 CAN 的通信，因此我们只考虑 rm_ecat_hw。

### 3.4 架构总结

三个层次的分工可以归纳为一张表：

| 层次 | 频率 | 通信方式 | 管什么 |
| --- | --- | --- | --- |
| 决策层（rm_manual） | 100Hz | ROS 话题 ↔ 控制层 | 什么时候做、做什么 |
| 控制层（rm_controllers） | 1kHz | C++ 指针 ↔ 硬件层 | 算法怎么算 |
| 硬件抽象层（rm_ecat_hw） | 1kHz | EtherCAT 帧 ↔ 物理硬件 | 跟硬件怎么说话 |

数据流是单向的：**操作手操作遥控器 → 决策层解析为指令 → 控制层执行算法 → 硬件层驱动电机**。反馈走另一条路：**电机编码器 → 硬件层读回来 → 控制层拿来做闭环 → 决策层读到 joint_states 用于显示和判断**。

这套分层的核心意图是让每一层只关心自己的事，不越界。换一个底盘电机（硬件层），不影响决策层的遥控器映射逻辑；换一套 PID 参数（控制层），不影响上层的指令格式；新增一个兵种的行为逻辑（决策层），底层的控制器插件可以原样复用。
