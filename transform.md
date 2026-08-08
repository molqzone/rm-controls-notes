# TF：机器人身上的坐标系树

> **前置知识**：[overview](./overview.md)（三层架构）、[hardware](./hardware.md)（理解 joint）

底盘要算里程计、云台要瞄准、视觉要把看到的目标换算到云台该转的角度——这些都在问同一类问题：**"A 相对 B 在哪、朝哪"**。回答这类问题的机制叫 **TF**。这篇讲清楚 TF 是什么、为什么它长成一棵树、rm-controls 怎么把关节和 IMU 变成 TF，以及它怎样把标准 TF Buffer 接入 ROS control 控制器。

> 本文会讲一点点必要的数学——刚好够解释"为什么是一棵树"和"变换是怎么算的"。但不深入旋转矩阵、四元数的具体运算规则（那可以另开一篇）。你只需要理解**每个变换就是一次"旋转 + 平移"**，以及它们怎么串起来。

---

## 1. TF 是什么

### 1.1 坐标系与坐标系树

机器人身上到处是**坐标系**（frame）：整车有个基准 `base_link`，云台有 `yaw`、`pitch`，摄像头有 `camera`，世界里还有个固定的 `odom`。每个坐标系就是一个"以某点为原点、某朝向为轴"的小十字架。

> **`odom` 和里程计是什么？** 里程计（odometry）就是机器人**自己估计"我从出发点走到哪了"**的方法——底盘靠轮子转了多少圈反推自己移动了多少（就像汽车的里程表，但同时记录方向）。`odom` 就是这个估计的**参考原点**：机器人一上电，当前位置被定为 `odom` 原点，之后它相对 `odom` 的位姿（`odom → base_link`）就表示"我现在离出发点多远、转了多少"。谁来算这个、怎么算、为什么会漂移，是 [chassis](./chassis.md) 里程计那节的事；这里只需知道 `odom` 是一个"固定在世界里、不随机器人动"的参考系。

每个坐标系不是孤立的。任意两个坐标系之间的关系，本质就是一次**刚体变换**——两样东西：

- **相对旋转**：B 的坐标轴相对 A 转了多少（一个旋转，可用旋转矩阵 $R$ 或四元数表示）
- **相对位置**：B 的原点在 A 里的位置（一个平移向量 $t$）

**我们真正关心的，从头到尾就是这两件事：旋转关系和相对位置关系。** 一个 TF 变换就是一对 $(R, t)$。给一个在 B 系里的点 $p_B$，换算到 A 系就是先转再挪：

$$p_A = R\, p_B + t$$

TF 里的每一条边、每一次 `lookupTransform`，返回的都是这么一个 $(R, t)$。

### 1.2 为什么是一棵树

现在解释"树"这个结构从哪来。把机器人所有坐标系画成一张图：每个坐标系是一个节点，两个直接相连、变换已知的坐标系之间连一条边。约束是——**每个坐标系只挂在唯一一个"父"坐标系下**（`pitch` 的父是 `yaw`，`yaw` 的父是 `base_link`，`base_link` 的父是 `odom`）。"每个节点只有一个父" 这个约束，数学上恰好定义了一棵**树**：

```
odom
 └── base_link          （底盘）
      └── yaw            （云台偏航）
           └── pitch     （云台俯仰）
                └── camera  （摄像头）
```

树结构带来一个关键性质：**任意两个坐标系之间，有且只有一条路径。** 这一点至关重要——它保证坐标变换是**无歧义**的。如果是一张有环的图，A 到 B 可能有多条路径，各自乘出来的结果还可能因为测量误差对不上，就乱套了。树砍掉了所有环，从任何坐标系到任何坐标系都只有唯一一条路可走。

而"沿路径把变换串起来",数学上就是**矩阵连乘**。想求 `camera` 相对 `base_link`，沿树把每条边的变换依次复合：

$$T_{\text{base\_link}\leftarrow\text{camera}} = T_{\text{base\_link}\leftarrow\text{yaw}} \cdot T_{\text{yaw}\leftarrow\text{pitch}} \cdot T_{\text{pitch}\leftarrow\text{camera}}$$

每个 $T$ 就是一个 $(R, t)$ 打包成的齐次变换矩阵，复合就是矩阵相乘；要反过来求（父→子换个方向），取逆矩阵 $T^{-1}$ 即可。**这就是为什么树 + 每条边一个变换，就足以回答任意两坐标系的关系**——框架沿唯一路径把 $(R, t)$ 连乘（或求逆再连乘）起来。

### 1.3 TF 帮你做什么

有了这棵树和每条边的变换，TF 系统能回答任意两个坐标系之间的关系——哪怕它们不直接相连。想知道"摄像头看到的东西在底盘坐标系里的位置"，TF 就自动沿树 `camera → pitch → yaw → base_link` 把这一串变换连乘起来，给你最终的 (R, t)。你只要问一句：

```
lookupTransform(目标坐标系, 源坐标系, 时间)
```

框架就把中间那些矩阵连乘（和必要的求逆）全办了，你不用碰任何矩阵。**这就是 TF 的全部价值：把散落在机器人各处的坐标系统一成一棵可查询的树，你只管问"A 相对 B 在哪、朝哪",它负责沿唯一路径算出旋转和平移。**

那么问题变成两个：这棵树是**谁建、谁维护**的？树上的变换又是**谁在实时更新**的？在 rm-controls 里，答案是两个控制器——`robot_state_controller` 和 `orientation_controller`。

---

## 2. robot_state_controller：把关节树变成 TF

### 2.1 职责

机器人的机械结构写在 **URDF** 文件里——哪个 link 连哪个 link、中间是什么关节、关节转轴在哪。这本身就是一棵树的静态描述。但 URDF 只说了"结构长这样"，没说"此刻 pitch 关节转到了 0.3 rad"。把静态结构 + 实时关节角度合成为**活的 TF 树**，就是 `robot_state_controller` 的活。

它每一拍做三件事：

1. **读关节角度**：通过 `JointStateInterface` 拿到每个运动关节当前的位置（就是 [hardware](./hardware.md) 里讲的、经过 Transmission 和 offset 换算好的关节角度）
2. **算每条边的变换**：结合 URDF 里的关节定义，算出每个子 link 相对父 link 的变换
3. **发布 TF**：把这些变换写进共享的 TF 树，同时广播到 ROS 的 `/tf` 话题

它声明的硬件接口正说明了这两头：

```cpp
class RobotStateController
  : public controller_interface::MultiInterfaceController<
      hardware_interface::JointStateInterface,   // 读关节角度（输入）
      rm_control::RobotStateInterface>           // 写 TF（输出）
```

一个细节优化：URDF 里的关节分两种——会动的（revolute/prismatic）每帧都要重算变换（存在 `segments_` 列表）；**固定关节**（fixed）永远不变，只需发布一次（存在 `segments_fixed_` 列表）。`init()` 里用 `addChildren()` 递归遍历关节树把两者分开，`update()` 每帧只重算 `segments_`，避免重复计算固定变换。发布使用 `TfRtBroadcaster`，其底层是 `realtime_tools::RealtimePublisher`；这能避免部分发布侧竞争，但 `sendTransform()` 本身仍会组装消息，不能据此宣称整条发布路径具有硬实时保证。

### 2.2 它还转发外部 TF

TF 树的数据源不止关节。底盘里程计（`chassis_controller`）、IMU 姿态（`orientation_controller`）、还有底盘外**独立 ROS 节点**（视觉 SLAM、定位 EKF、AMCL……）都会往树上贡献变换。前几个都在 1kHz 实时循环里，但外部节点是通过普通 ROS 话题 `/tf`、`/tf_static` 发布的，不在实时循环内。

`robot_state_controller` 顺带承担了一个"搬运工"角色：它订阅 `/tf` 话题，收到外部 TF 后，用线程安全的方式转发进实时循环，写入同一棵共享 TF 树。做法是一个 `realtime_tools::RealtimeBuffer`——非实时的订阅回调 `writeFromNonRT` 写进去，实时的 `update()` 里 `readFromRT` 取出来再 `setTransform` 汇入共享 Buffer：

```cpp
// 非实时线程：/tf 订阅回调
void tfSubCallback(const tf2_msgs::TFMessageConstPtr& msg) { tf_msg_.writeFromNonRT(*msg); }

// 实时循环：update() 里取出转发
for (auto& tf : tf_msg_.readFromRT()->transforms)
  robot_state_handle_.setTransform(tf, "external");   // authority 标明来源，便于溯源
```

这样无论变换来自实时循环还是外部话题，最终都汇入同一份树，其他控制器查询时看到的是完整的一棵。这套"`RealtimeBuffer` 跨线程搬运"的模式也用于控制器接收非实时命令（例如 [shooter](./shooter.md)），值得记住。

这个"为什么要费劲搬运"的问题，牵出了下一个更根本的设计——rm-controls 为什么要通过硬件接口共享标准 TF Buffer。先讲完 IMU，再回来说。

---

## 3. orientation_controller：把 IMU 姿态变成 TF

### 3.1 一个错位问题

底盘里程计、视觉定位都需要知道**底盘（`base_link`）在世界系（`odom`）里朝哪**。姿态信息来自 IMU，但 IMU 有个安装上的错位：

> 在 RoboMaster 机器人上，IMU 通常装在**云台**上（`gimbal_imu`），不是装在底盘上。所以 IMU 直接测到的是"云台相对世界"的姿态，而不是我们要的"底盘相对世界"的姿态。云台一直在相对底盘转动，两者对不上。

### 3.2 职责：一次坐标合成

`orientation_controller` 就是来消除这个错位的。它每拍做一次合成：

```
IMU 直接测量：      odom ──────────► gimbal_imu    （IMU 给的）
URDF/TF 已知：      gimbal_imu ────► base_link      （云台到底盘，查 TF 树得到）
合成、发布：        odom ──────────► base_link      （把上面两段接起来）
```

也就是——"世界到云台"（IMU 给）接上"云台到底盘"（TF 树给），得到"世界到底盘",发布出去。它声明的接口正对应这两个输入：

```cpp
class Controller : public controller_interface::MultiInterfaceController<
    rm_control::RmImuSensorInterface,   // 读 IMU 姿态四元数
    rm_control::RobotStateInterface>    // 查 TF（gimbal_imu→base_link）+ 发布 TF
```

`RmImuSensorInterface` 与标准 `ImuSensorInterface` 读取同一份姿态、角速度和线加速度数据；它额外提供采样时间戳，`orientation_controller` 用它判断是否出现新样本。姿态是否已滤波由下游硬件/IMU 管线决定，不是选择这个接口造成的。合成 `odom→base_link` 时，控制器还会查 `gimbal_imu→base_link`（来自 URDF，`robot_state_controller` 已经放进树里）再相乘。

这里能看到两个控制器的**依赖顺序**：`orientation_controller` 要用到 `gimbal_imu→base_link` 这条边，而这条边是 `robot_state_controller` 从 URDF 发布的。所以后者必须先就绪——这也是 overview 里"`robot_state_controller` 属于 `state_controllers`、必须先于其他控制器启动"的原因。

### 3.3 配置

```yaml
orientation_controller:
  type: rm_orientation_controller/Controller
  publish_rate: 100
  name: "gimbal_imu"        # IMU 名称，对应 RmImuSensorInterface 的 handle
  frame_source: "odom"      # 源坐标系
  frame_target: "base_link" # 目标坐标系
```

改 `name` 换 IMU，改 `frame_source/target` 换要发布的那条变换。

---

## 4. 为什么还要 RobotStateInterface

ROS 本身已有成熟的 TF 库 `tf2_ros`——`TransformBroadcaster` 发布、`Buffer` + `TransformListener` 查询。当前实现**仍然使用标准 `tf2_ros::Buffer`**；`RobotStateInterface` 的作用是把同一个 Buffer 作为硬件句柄交给控制器插件，而不是替换它。

### 4.1 需要解决的是共享与边界

`robot_state_controller` 初始化时创建一个 `tf2_ros::Buffer`，再把名为 `robot_state` 的 `RobotStateHandle` 注册到硬件接口。其他控制器据此查询或写入同一棵树：

- 避免每个控制器各自建 `Buffer + TransformListener`，从而避免重复订阅与相互独立的缓存；
- 控制器之间直接共享 Buffer，不必把内部 TF 先序列化成 `/tf` 再由另一个控制器订阅；
- `/tf` 和 `/tf_static` 的外部输入先写入 `RealtimeBuffer`，再由 `robot_state_controller::update()` 汇入该 Buffer；输出则用 `TfRtBroadcaster` 的 realtime publisher 发布。

这解决了数据所有权和话题交接边界，但不把 `tf2_ros::Buffer::lookupTransform()` / `setTransform()` 变成锁自由、无分配或有确定上界的操作。控制 worker 的 1kHz 只是目标周期；对硬实时路径仍应测量最坏耗时、限制查询次数，并处理查不到变换时抛出的异常。

### 4.2 RobotStateInterface：共享同一棵树

接口本身很薄，只转发同一个标准 Buffer：

```cpp
class RobotStateHandle {
  // 生产者：robot_state / chassis / orientation 调用，往树上写变换
  bool setTransform(transform, authority);
  // 消费者：任何需要 TF 的控制器调用，查变换
  geometry_msgs::TransformStamped lookupTransform(target, source, time);
};
```

`setTransform`（写）和 `lookupTransform`（读）通过 `RobotStateInterface` 在控制器之间**共享同一个实例**。配合 `RealtimeBuffer` 与 `TfRtBroadcaster`，当前数据路径是：

- 所有控制器读写的是**同一棵树**，不会各查各的、互相不一致
- 控制器之间的内部查询不走 ROS 话题序列化
- 外部话题来的 TF 由 `RealtimeBuffer` 跨入控制器 update，再写入同一棵树
- 出站 `/tf` 使用 realtime publisher 减少发布侧的非实时工作

所以它是一根贯穿控制器的 **TF 数据总线**，但不是实时安全性的证明：共享 Buffer、回调交接和发布优化是三件不同的事。

和标准方案对比：

| | 标准 `robot_state_publisher` + tf2 | rm-controls |
| --- | --- | --- |
| 形态 | 独立 ROS 节点 | ROS 控制器插件 |
| 运行位置 | 普通 ROS 节点 | ROS control 插件，在 controlWorker 目标 1kHz 周期调用 |
| 查询缓存 | 各节点可各自维护 Buffer | 一个标准 `tf2_ros::Buffer`，通过 `RobotStateInterface` 共享 |
| 广播器 | `tf2_ros::TransformBroadcaster` | `TfRtBroadcaster`（realtime publisher 封装） |
| 外部 TF 转发 | 订阅后直接写本地 Buffer | 订阅回调先入 `RealtimeBuffer`，再由 update 汇入 Buffer |

---

## 5. 其他控制器怎么消费 TF

生产端讲完了，看消费端。任何控制器只要在模板参数里声明了 `RobotStateInterface`，就能在自己的 `update()` 里查询：

```cpp
// 伪代码：某控制器查"云台相对底盘"的当前变换
auto tf = robot_state_handle_.lookupTransform("base_link", "pitch", time);
```

典型消费者：

- **`chassis_controller`**：FOLLOW 模式下要查云台朝向（底盘跟随云台），里程计要维护 `odom→base_link`。详见 [chassis](./chassis.md)
- **`gimbal_controller`**：把视觉给的目标位置、底盘的运动，换算到云台该转的 yaw/pitch。详见 [gimbal](./gimbal.md)
- **`orientation_controller`**：如上，查 `gimbal_imu→base_link` 做姿态合成
- **决策层 `rm_manual`**：也会查坐标系关系来构建指令。详见 [manual](./manual.md)

对消费者来说，整个 TF 系统就浓缩成一句 `lookupTransform`——谁在生产、变换怎么算、外部数据怎么汇进来，全被 `RobotStateInterface` 挡在后面了。这正是 overview 反复强调的分层意图：每一层只关心自己要问的那句话。

---

## 6. 小结

- **TF** 把机器人各处的坐标系连成一棵可查询的树，`lookupTransform(目标, 源, 时间)` 一句话就能得到任意两坐标系的相对位姿。
- 每个变换只关心两件事——**相对旋转 + 相对位置**，即一个 (R, t)。之所以是**树**（每个坐标系只有一个父），是为了让任意两系间的路径**唯一、无歧义**；沿唯一路径把各边的 (R, t) **矩阵连乘**，就得到任意两系的关系。
- **`odom`** 是固定在世界里的里程计参考原点，`odom → base_link` 表示机器人相对出发点走了多远、转了多少（里程计细节见 [chassis](./chassis.md)）。
- **`robot_state_controller`**：读关节角度 + URDF 结构 → 发布关节 TF 树；顺带把外部节点的 `/tf` 转发汇入。它属于 `state_controllers`，必须最先启动。
- **`orientation_controller`**：IMU 装在云台上，测的是"世界→云台",它合成"云台→底盘"得到"世界→底盘"并发布，纠正 IMU 的安装错位。
- rm-controls 仍使用标准 `tf2_ros::Buffer`；**`RobotStateInterface`** 只是把一个共享 Buffer 暴露给控制器。它减少重复 listener 和话题往返，但不保证 `lookupTransform` / `setTransform` 是硬实时操作。
- 消费者（chassis/gimbal/manual）只需声明接口、调 `lookupTransform`，无需关心 TF 从哪来。

下面进入三个业务领域文档：[chassis](./chassis.md)（底盘怎么动）、[gimbal](./gimbal.md)（云台怎么瞄）、[shooter](./shooter.md)（子弹怎么打）。
