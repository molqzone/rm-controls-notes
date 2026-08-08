# rm-controls 大观

rm-controls 是一套面向 RoboMaster 机器人的完整控制系统框架，构建在 ROS 1 Noetic 的 `ros_control` 之上。它不是零散的控制器集合，而是一个有明确架构哲学的分层系统。

## 1. 什么是无下位机

### 1.1 什么是实时性

电机要平稳转动，需要电机内部的控制器（RoboMaster 里叫“电调”，C620 / C610 这类）**定时**收到力矩或电流参考。rm-controls 的标称控制周期是 1ms（1kHz），每份新命令都应在下一截止时间前产生。偶发超期会增加闭环延迟，持续超期则可能让电调反复执行旧命令、使机构失稳；具体电调是否自带超时停机，取决于它的协议与固件，不能假定旧命令永远安全。

怎么尽量保证每个周期按时完成？关键不只在平均算得快，还在**最坏情况有界**：控制回路从“读编码器”到“发力矩”的延迟和抖动要有可验证上界，并且要为超期定义降级行为。平均 100μs 但偶尔卡 5ms 的循环，不如稳定在 400μs 的循环可控。

这才是机器人控制里说的**实时性**——不是“反应快”的同义词，而是关键任务能在截止时间内完成，超期能被检测并安全处理。允许的抖动取决于总线、执行器和闭环带宽，不是所有机构都固定为“几十微秒”。

### 1.2 传统方案怎么满足

STM32 这类 MCU 常被选作下位机，是因为任务少、内存和中断路径可控，更容易把最坏响应时间做小并测出来。它并非“没有抖动”：中断优先级、临界区、DMA 争用和缓存都会带来延迟，只是通常比通用 Linux 更容易给出上界。

所以典型的 RoboMaster 软件架构是**上位机 + 下位机**：

- **下位机**（STM32）：跑机构位置/速度环和状态机，经 CAN 给 C620/C610/GM6020 等电调发送电流或电压参考；换相、PWM 和内电流环仍由电调完成
- **上位机**（NUC）：跑 ROS、视觉、导航、决策——这些任务不在乎几十微秒的抖动，偶尔卡一帧也没关系

### 1.3 rm-controls 的做法

传统的 MCU 下位机有一个致命的开发痛点：**改一行代码就要烧录一次固件**。调个 PID 参数、改个标定逻辑、加个调试输出——都要编译、下载、重启，一次几分钟。而 RAM 和 Flash 有限，跑不了复杂算法，也存不了长时间的调试日志。

rm-controls 的解法很直接：**既然上位机算力富余，为什么不让上位机完成机构控制？** 它移除的是队伍自研 MCU 上的位置环、速度环和业务状态机，把这些逻辑搬到 NUC。嵌入式控制并没有从系统中消失：EtherCAT 从站仍做协议转换、收发缓存和链路保护，电调仍做换相、PWM 与内部电流环。“无下位机”更准确地说是**没有承载机构控制算法的自研 MCU 下位机**。

这显然有个矛盾：上位机跑的是 Linux，天然有调度抖动，怎么保证 1kHz 的确定性？

答案是引入一个关键设计思想——**异步解耦**。不是把实时和非实时硬塞进同一个线程里排队等（同步），而是让它们**各跑各的线程，中间用缓冲区异步交接**。这是 rm-controls 整个架构最核心的设计决策：

- **通信关键路径**（EtherCAT 线程）：只做 SOEM 收发与 PDO 数据搬运，不在路径中同步做 ROS 发布或日志 I/O，目标是稳定按 1kHz 运行。
- **控制路径**（controlWorker）：跑 ROS control 的 `controllerManager::update()` 和控制器计算。它可以比通信线程有更大抖动，但仍有自己的时限；卡住后通信线程只会重复旧命令，不会自动变安全。
- **降频发布路径**（publishWorker）：从快照发布诊断话题，把序列化和网络发送移出通信关键路径。
- **异步交接**：反馈与命令通过缓冲区跨线程传递，代价是约一个周期延迟。每份命令还应有时间戳或序号；超过新鲜度阈值时要清零/失能或受控保持，并清理相关积分，恢复时从当前状态平滑接回。

这就是 [communication](./communication.md) 里双缓冲和线程隔离的核心动机。异步解耦贯穿整个 rm-controls：决策层跑 ~100Hz，通过 ROS 话题通知控制层；控制层按 1kHz 目标周期计算，再通过缓冲区交接给 EtherCAT 线程。每一层不强求相邻层和自己同一个节拍，但都必须检查输入新鲜度。

除了实时性方面的收益，把所有控制逻辑集中到上位机还带来一些额外优势：

- **Sim2real 代码高度复用**：控制器可在 Gazebo、Unity 等仿真器中复用并提前验证；实车仍要重新面对通信延迟、量化、饱和、摩擦、固件模式和安全限制
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
    <offset>3.55</offset>  <!-- 示例：单位 rad，实际值必须按机构标定 -->
  </joint>
</transmission>
```

`SimpleTransmission` 的 offset 单位是 rad；若看到 `355` 这类明显超出单圈的值，应先核对是否漏了小数点或用错了自定义字段，不能照抄。调法可以先按较大步长寻找弹位，再缩小到百分位，并用连续发射验证。全程不需要动一行 C++。

### 2.2 调云台 pitch 的重力补偿

云台由于重心不在旋转轴上，启动控制器后如果补偿不对，pitch 会往下掉或往上抬。调重力补偿就是改 YAML 里的三个数字，重启控制器后看云台能不能定住：

```yaml
# rm_controllers/hero.yaml
gimbal_controller:
  feedforward:
    gravity: -1.944                         # 重力前馈系数
    enable_gravity_compensation: true
    mass_origin: [0.06662, -0.012383, 0.038901]  # 质心位置
```

调参时先用机械工装或手托住云台，保留低增益稳定环并设置保守的力矩、速度和关节限位；逐步增加补偿，观察稳定后 PID 还需要输出多少力矩。最好在多个 pitch 角采样并拟合质心参数，最后覆盖完整角域验证。不要把全部 PID 清零后直接松手，重云台会下坠撞限位。整个过程只改 YAML，不需要改下位机代码；完整方法见 [gimbal](./gimbal.md) §3.2。

## 3 架构

rm-controls 的架构可以概括为：

- ROS control 的 hardware_interface 层
- 一套自定义的决策中间件（决策核心 `rm_manual` + `rm_dbus`/`rm_vt`/`rm_referee` 三个 I/O 驱动）
- 9 个控制器插件
- 6 个硬件接口
- 一个标定框架

### 3.1 决策层

决策层解决的是 ROS control 上层的问题：

- 操作手交互：遥控器拨杆怎么映射到控制指令？按键按下 / 松开对应什么动作？
- 控制器编排：什么时候启动 `chassis_controller`？什么时候切到 `calibration_controller`？谁来仲裁 joint 冲突？
- 标定流水线：上电后先标定拨盘 → 再标定云台 → 再启动主控制器。这个顺序谁来编排？
- 兵种差异：英雄有云台 + 发射 + 自瞄，工程有机械臂 + 动作服务器，哨兵有导航 + 雷达，无人机有飞控——逻辑完全不同，但底层控制器可以复用

决策层运行在非实时 ROS 循环中（`rm_manual` ~100Hz `ros::Rate` 循环，`rm_referee` 80Hz 串口轮询），不参与 1kHz 实时控制环。它与控制层通过 ROS 话题通信，与硬件层也通过 ROS 话题读取状态（/joint_states、/actuator_states）。

决策层不止 `rm_manual` 一个节点。它由**决策核心**和一圈**输入/输出驱动**组成——后者负责把物理世界的操作手、裁判系统接口翻译成 ROS 话题，喂给决策核心，也把决策核心的反馈发回去：

```
决策层（都跑在非实时，各节点频率不同：`rm_manual` ~100Hz，`rm_referee` 80Hz 串口轮询，`rm_dbus` 60Hz，`rm_vt` 100Hz）
├── rm_manual        决策核心：事件解析、控制器编排、标定流水线、指令发布
└── I/O 驱动
    ├── rm_dbus      DT7 遥控器输入 → DbusData
    ├── rm_vt        图传链路键鼠输入 → VTKeyboardMouseData；客户端 UI 输出
    └── rm_referee   裁判系统输入（比赛状态/血量/功率/热量）；客户端/地图输出
```

这几个 I/O 驱动都是独立 ROS 节点，**不碰 1kHz 控制环、不直接操作电机**——它们的数据只流向 / 流出 `rm_manual`。这就是把它们归到决策层的理由：它们解决的是"操作手、比赛规则怎么和机器人的大脑对话"，属于决策关心的事，而不是"算法怎么算"（控制层）或"跟执行器硬件怎么说话"（硬件抽象层）。

### 3.2 控制层

TODO: joint+link only 的 robomaster 机器人
电控眼中目无全牛的 RoboMaster机器人

控制层解决的核心问题：ROS control 的标准 controller 只提供一个空的 `update()` 回调和几个 `JointHandle`，但 RoboMaster 需要复杂的控制算法和状态机。

当前 EtherCAT 实现中，控制器运行在独立的 `controlWorker`（目标周期 1ms）里：每拍执行 **`read() → controllerManager_->update() → write()`**。EtherCAT 收发位于另一个 `updateWorker`，两者通过从站的 staged command / reading 缓冲区交接数据；控制器仍通过 `hardware_interface` 的 C++ 句柄读写，不走 ROS 话题序列化。

#### 3.2.1 运动控制

运动控制控制器是真正参与 1kHz 控制环、向 joint 输出力矩/速度的控制器。它们的划分直接来源于机器人的机械结构——每个机构对应一个控制器，各自 claim 一组互不相交的 joint：

| 机构 | 对应控制器 | 控制的 joint |
| --- | --- | --- |
| 底盘 | `chassis_controller` | 4 个轮子 joint |
| 云台 | `gimbal_controller` | yaw_joint + pitch_joint |
| 发射机构 | `shooter_controller` | 2 个摩擦轮 + 拨弹盘 joint |
| 从动关节 | `mimic_joint_controller` | 跟随 pitch_joint 位置（只读） |

因为 joint set 互不相交，所以控制器可以独立启停——标定拨盘时只需停 `shooter_controller`，底盘和云台不受影响。这也是标定体系能工作的前提。

各控制器核心功能：

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
| `calibration_controllers` | 需要机械零点的关节自动找零：撞限位/Hall → 设运行时 offset → 标记完成 |
| `gpio_controller` | 读取 GPIO 输入（如霍尔传感器）触发标定，输出 GPIO 命令（如电磁铁） |

#### 3.2.3 状态发布

只读硬件数据并发布成 TF 或 topic，不输出力矩：

| 控制器 | 解决什么问题 |
| --- | --- |
| `robot_state_controller` | 把 URDF 关节树发布成 TF，并通过 `RobotStateInterface` 提供共享查询入口 |
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

硬件抽象层解决的核心问题：ROS control 的标准 `hardware_interface` 只认识 `JointState` 和 `EffortJoint`，但 RoboMaster 的硬件有大量特殊需求——电机有不同的类型（RM6020 / M3508 / M2006 / 达妙）、有标定状态需要管理、有 GPIO 和 IMU 等非关节外设。通信层面，所有电机使用 CAN 协议，`rm_ecat_hw` 通过 EtherCAT 转发 CAN 数据，不直接与电机通信。

当前 `rm_ecat_hw` 节点由 `rm_ecat_ros::RmEcatHardwareInterface` 实现，继承 ROS control 的 `RobotHW`。它启动两个目标周期为 **1ms** 的 worker：`updateWorker` 负责 EtherCAT 收发，`controlWorker` 负责 ROS control 的 read-update-write；另有约 100Hz 的 `publishWorker` 做 ROS 发布。

它做了三件事：

#### 3.3.1 封装通信协议

`rm_ecat_hw` 负责 EtherCAT 帧的收发——管理从站拓扑（哪块板、什么协议、挂在哪张网卡上），把 `RmMotor` / `MitMotor` 等不同协议电机封装成统一的 `ActuatorData` 结构体。

#### 3.3.2 提供扩展 hardware_interface

标准 ROS control 不提供的功能都由自定义接口补上：

| 接口 | 解决的问题 |
| --- | --- |
| `RobotStateInterface` | 把 `robot_state_controller` 创建的同一个 `tf2_ros::Buffer` 作为句柄交给控制器共享；控制器之间不用通过 `/tf` 话题往返，但这层封装**不保证**查表无锁、无分配或硬实时 |
| `ActuatorExtraInterface` | 旁路存储标定状态（`needCalibration`、`calibrated`、`offset`）。标准 JointHandle 没有这些字段，且不应被标定逻辑污染 |
| `GpioStateInterface` / `GpioCommandInterface` | 扩展 ROS control 到 GPIO 设备，供标定控制器读霍尔传感器、供 manual 控制电磁铁 |
| `RmImuSensorInterface` | 与标准 `ImuSensorInterface` 共享姿态、角速度和线加速度数组；扩展 handle 额外提供采样时间戳，方便 `orientation_controller` 判断是否有新数据 |
| `TofRadarInterface` | `rm_control` 中保留的 ToF 扩展接口；当前 `RmEcatHardwareInterface` 未注册它，属于遗留 CAN `rm_hw` 路径的能力 |

#### 3.3.3 管理 Transmission 与标定状态

硬件层先把线协议中的编码器码、电流码解成 SI 单位的执行器状态，再由 Transmission 在执行器空间与关节空间之间映射；控制器始终读写关节侧的 rad、rad/s 和 N·m。标定状态及运行时 offset 也由硬件层承载，详见 [hardware](./hardware.md) §2-§4。

本文以当前 EtherCAT 启动路径 `rm_ecat_hw.launch` 为主。工作区仍保留 `rm_hw` / `rm_can_hw.launch` 的 Linux-to-CAN 兼容路径及其配置；两套路径不要混作同一份运行时配置。

### 3.4 架构总结

这里有两个同名但职责不同的管理器。`RmEcatHardwareInterface` 内持有 ROS control 的 `controller_manager::ControllerManager`，由 `controlWorker` 在 1kHz 调用其 `update()`，实际执行活跃控制器。`rm_manual` 内的 `rm_common::ControllerManager` 则是约 100Hz 的 ROS 服务客户端：它加载控制器，并把 start/stop 请求发给前者。每个控制周期仍是 **read → 所有活跃 controller 的 update → write**。

三个层次的分工可以归纳为一张表：

| 层次 | 频率 | 通信方式 | 管什么 |
| --- | --- | --- | --- |
| 决策层（rm_manual 决策核心<br>+ rm_dbus/rm_vt/rm_referee I/O 驱动） | 非实时（~100Hz） | ROS 话题 ↔ 控制层<br>ROS 服务客户端 ↔ controller_manager<br>串口/DBus/图传 ↔ 操作手·裁判系统 | 什么时候做、做什么 |
| 控制层（rm_controllers） | controlWorker，目标 1kHz | C++ 句柄 ↔ 硬件抽象层 | 算法怎么算 |
| 硬件抽象层（rm_ecat_hw） | updateWorker，目标 1kHz | EtherCAT 帧 ↔ 物理硬件 | 跟硬件怎么说话 |

数据流是单向的：**操作手操作遥控器 → 决策层解析为指令 → 控制层执行算法 → 硬件层驱动电机**。反馈走另一条路：**电机编码器 → 硬件层读回来 → 控制层拿来做闭环 → 决策层读到 joint_states 用于显示和判断**。

这套分层的核心意图是让每一层只关心自己的事，不越界。换一个底盘电机（硬件层），不影响决策层的遥控器映射逻辑；换一套 PID 参数（控制层），不影响上层的指令格式；新增一个兵种的行为逻辑（决策层），底层的控制器插件可以原样复用。
