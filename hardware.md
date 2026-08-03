# 硬件：电机、编码器、Transmission 与标定

> **前置知识**：[overview](./overview.md) 的"硬件抽象层"概念、[communication](./communication.md)（理解命令是怎么送到电机的）

[communication](./communication.md) 讲的是命令怎么从上位机送到电机；这篇讲的是**链路终点的硬件本身**——电机是什么、它怎么知道自己转到了哪、控制器眼里的"关节"和电机之间隔着什么，以及为什么每次上电都要做一件叫"标定"的事。这是硬件抽象层最需要新人理解的一块。

读之前先记住一个贯穿全文的区分：

- **关节（joint）**：控制器眼里的东西，单位是弧度、牛·米，符合机器人学直觉（"pitch 抬到 0.3 rad"）
- **执行器（actuator）**：真实的电机，单位是编码器计数、原始整数，符合硬件现实

两者之间的翻译层就是 **Transmission**。标定则是保证这个翻译在每次上电后都对得上。

---

## 1. RoboMaster 常用电机

RoboMaster 生态里，控制器最常打交道的是这么几款电机。它们的差别不只是大小，更关键的是**编码器类型**（决定要不要标定）和**内置电调支持的控制模式**。

| 型号 | 减速比 | 力矩常数 | 编码器 | 典型用途 |
| --- | --- | --- | --- | --- |
| **GM6020** | 1:1（直驱） | 0.741 N·m/A | **绝对值**（不需要标定） | 云台 yaw / pitch |
| **M3508** | ≈19.2:1（3591:187） | 0.3 N·m/A | 增量式（可能需要标定） | 底盘轮、摩擦轮 |
| **M2006** | 36:1 | 0.18 N·m/A | 增量式（可能需要标定） | 拨弹盘、小负载关节 |
| **达妙（DM）** | 视型号 | 视型号 | 绝对值 | 需要力控/位控的关节 |

### 1.1 三种控制模式

电机内置的电调（如 C620 对应 M3508）支持不同的控制"口味"，对应机器人学里三种最基本的量：

- **力矩控制（Effort）**：直接指定电机输出多大力矩（本质是控电流）。这是 RM 电机最底层、最通用的模式——rm-controls 里绝大多数控制器最终都是往关节写力矩。因为力矩控制最灵活，速度环、位置环都可以在上位机用软件 PID 叠在力矩之上实现
- **速度控制（Velocity）**：指定电机转多快。底盘轮、摩擦轮天然是速度需求
- **位置控制（Position）**：指定电机转到哪个角度。拨弹盘"每次精确转一格"、机械臂"转到某个姿态"是位置需求

这里有个 rm-controls 的关键设计选择：**它几乎不用电调自带的速度/位置模式，而是统一用力矩模式，速度环和位置环全在上位机的控制器里用 PID 软件实现。** 原因回到 overview 讲的无下位机哲学——把控制逻辑全放上位机，才能仿真、可视化、快速迭代。所以你在配置里看到的绝大多数电机接口都是 `EffortJointInterface`（力矩接口）。

> 达妙（DM）电机是个例外，它支持 MIT 协议，能直接接受"位置 + 速度 + 力矩 + 刚度 + 阻尼"的复合指令，对应 [communication](./communication.md) 里提到的 `RmEcatMitSlave`。需要底层力控的场合（如平衡底盘的腿）会用它。

### 1.2 力矩常数的意义

力矩常数（N·m/A）是"每安培电流产生多少力矩"。控制器算出的是关节力矩（N·m），最终要变成电调认识的电流值——这个换算就用到力矩常数和减速比。日常调车时你不用手算，但要知道：**换电机型号（比如 M3508 换成 M2006）后 PID 一定要重调**，因为力矩特性完全不同，老参数搬过去要么软绵绵要么直接震荡。

---

## 2. 编码器：电机怎么知道自己在哪

电机要做闭环控制，必须能回答"我现在转到哪个角度了"。回答这个问题的部件叫**编码器**。它分两种，这个区别是整个标定体系存在的根源。

### 2.1 绝对式编码器

绝对式编码器像一个刻好度数的表盘：任何时刻、包括**刚上电的瞬间**，它都能直接读出当前的绝对角度。GM6020、达妙用的就是这种。

**结果：用绝对式编码器的关节不需要标定。** 上电即知道自己在哪。这就是为什么英雄的云台 yaw/pitch（GM6020）从来不用标定。

### 2.2 增量式编码器

增量式编码器像一个只会"计步"的计步器：它只能数出"从某个起点转过了多少格"，但**它不知道起点在哪**。掉电后计数清零，重新上电时它只知道"我现在是 0",却不知道这个 0 对应机械结构上的哪个真实位置。

M3508、M2006 用的是增量式。这带来一个致命问题：

> 拨弹盘用 M2006。上电时编码器读数是 0，但这个 0 可能是拨盘转了半格的位置。如果控制器天真地以为"0 就是弹位对齐"，那每次发射的角度都是错的——要么双发，要么空发。

**解决办法就是标定**：上电后，让电机主动跑到一个机械上已知的参考点（比如撞到限位），把"编码器此刻的读数"和"这个真实位置"对应起来，记下差值。这个差值就是 **offset**（零点偏移）。之后每次读编码器，加上 offset，就得到正确的绝对角度。

一句话记住这一节：

| | 绝对式 | 增量式 |
| --- | --- | --- |
| 代表电机 | GM6020、达妙 | M3508、M2006 |
| 上电知道绝对位置吗？ | 知道 | **不知道** |
| 需要标定吗？ | 不需要 | **需要**（如果关心绝对角度） |

注意最后一栏的"如果关心绝对角度"：底盘轮用 M3508（增量式），但它只做**速度控制**，根本不在乎绝对位置转到哪，所以底盘轮**不标定**。真正需要标定的，是既用增量式编码器、又需要知道绝对角度的**位置控制关节**——拨弹盘、云台辅助关节、机械臂关节等。

---

## 3. Transmission：关节和电机之间的翻译层

### 3.1 为什么需要它

控制器想的是"关节"：给 pitch 关节一个 0.3 rad 的目标、往 trigger 关节输出 2 N·m。但电机是执行器，它认的是"电机轴转了多少圈、电机电流多大"。两者之间隔着减速箱、安装方向、零点偏移等一堆机械现实。

**Transmission（传动）就是把关节空间和执行器空间互相翻译的一层。** 它是双向的：

```
控制器写命令：  关节力矩/位置  ──Transmission──►  电机力矩/位置   （下行）
读回反馈：      关节角度/速度  ◄──Transmission──  电机编码器值   （上行）
```

这层翻译发生在硬件抽象层，控制器完全无感——控制器永远只跟"关节"打交道，Transmission 自动把它换算成电机认识的量。这正是 [communication](./communication.md) 全景图里那个"Transmission 换算"步骤。

Transmission 由 **URDF** 文件描述（`*.transmission.urdf.xacro`），硬件层启动时加载。rm-controls 用到三种。

### 3.2 SimpleTransmission：一个电机对一个关节

最常见的一种：一个电机通过减速箱驱动一个关节。它只需要两个参数——**减速比**和 **offset**：

```xml
<transmission name="trans_trigger_joint">
  <type>transmission_interface/SimpleTransmission</type>
  <actuator name="trigger_joint_motor">
    <mechanicalReduction>-27.5</mechanicalReduction>   <!-- 减速比 -->
  </actuator>
  <joint name="trigger_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <offset>355</offset>   <!-- 零点偏移 -->
  </joint>
</transmission>
```

- **减速比（mechanicalReduction）**：电机转多少圈，关节转一圈。换算关节角度和电机角度的比例因子。
- **offset**：关节零点相对电机零点的偏移。对增量式关节，这个值由标定动态算出并写入（见后文）；对某些场合（比如拨盘对齐弹位）也可以手动在 URDF 里写死。overview 里"调拨盘 offset 解决双发"那个例子调的就是它。
- **负减速比 = 反装**：减速比写成负数，表示电机是**反方向安装**的——电机正转对应关节反转。这是个非常实用的技巧：机械装反了不用拆下来重装，配置里把减速比取负即可。上面例子里的 `-27.5` 就是一个反装的拨盘电机。

### 3.3 DifferentialTransmission：差速器

有些机构用**差速器**：两个电机的转动通过差速齿轮耦合，共同决定两个关节的运动。典型场景是差动云台——两个电机一起转控制 pitch，反向转控制另一个自由度。

`DifferentialTransmission` 描述的就是这种"两个电机 ↔ 两个关节"的耦合关系。它的换算不再是简单的乘减速比，而是两个电机值的加减组合。因为两个电机是耦合的，它们的标定也必须一起做——这就是后面要讲的**差动标定**存在的原因。

### 3.4 DualActuatorTransmission：双电机带一个关节

还有一种是**两个电机共同驱动同一个关节**——比如一个大负载关节，单个电机力矩不够，用两个电机并联出力。这种"两个执行器 ↔ 一个关节"的关系由 `DualActuatorTransmission` 描述：关节的力矩指令被分配到两个电机上，两个电机的反馈被合并成一个关节状态。

### 3.5 三种 Transmission 小结

三种类型都是 ROS control `transmission_interface::Transmission` 抽象基类的实现：

```
transmission_interface::Transmission          （抽象基类）
  ├── SimpleTransmission            1 电机 : 1 关节 —— 绝大多数关节（轮子、拨盘、云台单轴）
  ├── DifferentialTransmission      2 电机 : 2 关节（耦合）—— 差动云台、差速机构
  └── DualActuatorTransmission      2 电机 : 1 关节 —— 大负载关节，双电机并联出力
```

每个 Transmission 对象持有指向执行器空间和关节空间的句柄，`propagate`（传播）就在这两块数据之间做换算——正是本节开头那张双向图的代码落点：

| 方向 | 方法 | 时机 |
| --- | --- | --- |
| 执行器 → 关节 | `actuatorToJoint{Position,Velocity,Effort}` | `read()` 里，把编码器反馈换成关节值 |
| 关节 → 执行器 | `jointToActuatorEffort` | `write()` 里，把关节命令换成电机命令 |

硬件层用 `TransmissionInterfaceLoader` 从 URDF 解析 `<transmission>` 标签、实例化对应类、把句柄接到 `ActData` 上。你在 URDF 里写 `<type>transmission_interface/SimpleTransmission</type>`，加载器就 new 出一个 `SimpleTransmission`——配置驱动，代码不动。

---

## 4. 标定：让每次上电后关节零点都对得上

> **自动标定是 rm-controls 的一大特色。** 别的框架（如 RMCS、XRobot）通常把零点位置硬编码在 YAML 配置文件里——装车时手动拨到零位、读编码器值、填进 `yaw_motor_zero_point` / `pitch_motor_zero_point` 这类字段，下次换机械结构就得重新量。这种做法的前提是"机械结构终身不变"，但 RoboMaster 的比赛环境是"频繁拆装、可能被撞歪"，硬编码零点一偏整个控制就偏了。
>
> rm-controls 的做法是**上电自动标定**：每次启动都让电机自己跑去找参考点，动态算 offset，写入内存。机械变了也不怕——下一次上电自动重新找零。这套机制的核心是下文的 `ActuatorExtraInterface` + `CalibrationQueue` + 标定控制器三层配合。

### 4.1 标定的本质

标定就四步，非常简单：

```
① 找零   —— 驱动电机运动到一个机械上已知的参考点
② 算 offset —— offset = -(此刻的编码器读数)，即把当前位置强制定义为 0
③ 存 offset —— 把 offset 写进 ActuatorExtraInterface
④ 标记完成 —— 设 calibrated = true，硬件层从此自动给读数加上 offset
```

第②步的核心代码就一行（以撞限位标定为例）：

```cpp
actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
actuator_.setCalibrated(true);
```

含义是：把"当前原始位置"修正掉，使得之后 `joint.getPosition() = 原始位置 + offset` 返回正确的绝对角度。找到参考点的那一刻，就把那一刻定义成零点。

### 4.2 标定状态存在哪：ActuatorExtraInterface

这里有个设计上的讲究。标准 ROS control 的 `JointHandle` 只有 `getPosition()` / `setCommand()`，**根本没有"标定状态"这个概念**——它是给工厂机器人设计的，出厂标定一次就终身不变。但 RoboMaster 每次上电都要标定，需要一个地方存 `needCalibration`、`calibrated`、`offset` 这些字段。

rm-controls 的解法是**不去污染标准 JointHandle，而是旁开一个专门的接口** `ActuatorExtraInterface`（overview 里 6 个自定义接口之一）：

```cpp
class ActuatorExtraHandle {
  // 读
  bool   getHalted();          // 电机是否堵转
  bool   getNeedCalibration(); // 是否需要标定（来自 YAML）
  bool   getCalibrated();      // 是否已完成标定
  double getPosition();        // 当前位置
  double getOffset();          // 零点偏移
  // 写（标定完成时调用）
  void   setOffset(double);    // 修正零点
  void   setCalibrated(bool);  // 标记标定完成
};
```

这样分离的好处：

1. **标准接口不被污染**：控制器读 `joint.getPosition()` 时完全不知道有标定这回事
2. **硬件层透明地应用 offset**：硬件层读编码器时自动 `pos = 原始值 + offset`，上层无感知
3. **标定逻辑只碰该碰的**：标定控制器只通过 `setOffset` / `setCalibrated` 改状态，不动别的

### 4.3 状态流转

一个需要标定的关节，其标定状态经历这样的流转：

```
needCalibration = true（YAML 里配的）
      │  上电，calibrated = false
      ▼
INITIALIZED ──驱动电机搜索参考点──► （运动中：MOVING_POSITIVE / FAST_FORWARD…）
      │
      │  检测到参考点（撞限位速度骤降 / GPIO 跳变）
      ▼
CALIBRATED ──setOffset + setCalibrated(true)──► 完成
```

配置里用 `need_calibration: true` 声明一个关节需要标定（在 `rm_hw` 和 `rm_ecat_hw` 配置中）：

```yaml
# rm_hw/hero.yaml
actuators:
  trigger_joint_motor:
    type: rm_3508
    need_calibration: true    # ← 需要标定
  yaw_joint_motor:
    type: rm_6020
    # 缺省 = false，绝对编码器不用标定
```

`calibrated` 每次控制器 `starting()` 时都被强制置回 false——也就是**每次启动都重新标定**，绝不信任上一次的零点。这是 RoboMaster"频繁重启、每次自动标定"现实的直接体现。

### 4.4 标定时硬件层的两个特殊行为

标定是个"鸡生蛋"的问题：还没标定完，关节的绝对位置就是错的，如果这时硬件层照常做关节限位保护，就会因为"位置不对"而误触发限幅，电机根本跑不到限位。所以硬件层在标定期间有两个特殊处理：

1. **跳过限幅**：当 `needCalibration && !calibrated` 时，硬件层的 `write()` 跳过关节限位保护，让标定控制器能自由驱动电机去撞限位。
2. **自动应用 offset**：标定完成、`calibrated = true` 后，硬件层读编码器时自动 `pos = 原始值 + offset`，上层控制器立刻拿到正确的绝对角度，无需任何额外代码。

### 4.5 三种标定方式

机械参考点怎么找？取决于关节的机械设计，有三种方式，对应三个标定控制器。

#### 撞限位（MechanicalCalibrationController）

最朴素：让电机朝一个方向匀速转，直到**撞到机械限位**转不动。撞上时速度会从搜索速度（比如 4 rad/s）**骤降到接近 0**。控制器检测到"速度持续低于阈值足够久"就确认到达限位，设 offset。

```
电机以 4 rad/s 正转 → 撞机械限位 → 速度骤降 < 0.001
   → countdown 计时确认（不是瞬间抖动）→ setOffset → 完成
```

判断堵转的核心逻辑（简化）：

```cpp
if (std::abs(velocity) < vel_threshold_ && !actuator_.getHalted())
  countdown_--;          // 速度够低，倒计时递减
else
  countdown_ = 100;      // 一旦超过阈值，倒计时复位
if (countdown_ < 0) {    // 持续低速足够久 → 确认撞限位
  setOffset(...); setCalibrated(true);
}
```

这里 `getHalted()` 用来区分"撞到限位"和"被人为/外力停住"——后者不算标定成功。

它还有两个变体：
- **中心标定（center）**：先正转找一侧限位，再反转找另一侧，取两侧**中点**做零点。适合需要对称运动范围的摆动关节。
- **回位标定（return）**：标定完不停在限位，而是用位置 PID 回到一个预设角度。适合机械臂——标定后要回到工作姿态才能干活。

适用关节：拨弹盘（trigger）、瞄准镜（scope）、图传（image_transmission）、机械臂各关节。

#### 读 Hall（GpioCalibrationController）

有些关节没有硬限位，或者需要更高精度。它们装了**霍尔开关/微动开关**（一种 GPIO），关节转到特定角度时电平翻转。这种标定用三步法保证精度：

```
① 快速正转 → 检测到 GPIO 跳变 → 记录大致位置
② 后退一个小角度 → 让 GPIO 恢复初始电平（脱离触发区）
③ 慢速正转 → 再次 GPIO 跳变 → 停下，设 offset
```

为什么要"快一次慢一次"？快速找大致区域效率高，但惯性会让跳变点有几度到十几度的误差；后退后慢速逼近能把过冲消掉，精度从约 0.02 rad 提到约 0.001 rad。快慢两个速度独立配置：

```yaml
velocity:
  search_velocity: -4.0        # 快速搜索
  slow_forward_velocity: -2.0  # 慢速逼近
```

适用关节：增量式编码器版本的云台 yaw/pitch、飞镖发射架的俯仰/偏航。

#### 差动（DifferentialCalibrationController）

针对 3.3 讲的**差动机构**——两个电机耦合控制自由度。标定时驱动一个电机去撞限位，另一个保持位置，撞到后**同时**给两个电机设 offset、设 calibrated：

```cpp
actuator_.setOffset(...);   actuator2_.setOffset(...);
actuator_.setCalibrated(true); actuator2_.setCalibrated(true);
```

因为差动机构撞限位不像单关节那么干脆，它加了个**超时保护**（`max_calibretion_time`，默认 5s），避免无限等待。

三种方式对比：

| | 撞限位 | 读 Hall | 差动 |
| --- | --- | --- | --- |
| 检测原理 | 速度骤降 | GPIO 电平跳变 | 速度骤降 + 双电机同步 |
| 额外硬件 | 无（靠机械限位） | 霍尔传感器 | 无 |
| 精度 | 中 | 高（慢速逼近） | 中 |
| 超时保护 | 无 | 无 | 有（默认 5s） |
| 控制电机数 | 1 | 1 | 2 |

### 4.6 calibration_controller 的实现

标定控制器本身也是一个标准的 ROS control 控制器，跑在 1kHz 实时环里。理解它的实现能把 overview 里"控制器怎么声明硬件接口"这件事落地。

**如何继承 MultiInterfaceController。** 三个标定控制器都继承自共享基类 `CalibrationBase<T...>`，而它继承自 `MultiInterfaceController<T...>`——一个用可变模板参数声明"我要哪几个硬件接口"的基类：

```cpp
class MechanicalCalibrationController
    : public CalibrationBase<rm_control::ActuatorExtraInterface,     // 读写标定状态
                             hardware_interface::EffortJointInterface> // 向关节发力矩
{ /* ... */ };
```

模板参数就是它的硬件依赖。GPIO 标定多要一个 `GpioStateInterface`（读霍尔电平），差动标定要两份执行器接口——需求不同，模板参数不同。三个子类挂在同一棵树上：

```
controller_interface::MultiInterfaceController<T...>
  └── CalibrationBase<T...>                     公共骨架（init/starting/stopping + is_calibrated 服务）
        ├── MechanicalCalibrationController     <ActuatorExtra, EffortJoint>
        ├── GpioCalibrationController           <ActuatorExtra, GpioState, EffortJoint>
        └── DifferentialCalibrationController   <ActuatorExtra, EffortJoint>
```

基类 `CalibrationBase` 备好三个子类共享的资源，子类**只覆写 `update()`**（各自那套状态机）——典型的模板方法：

```cpp
template <typename... T>
class CalibrationBase : public controller_interface::MultiInterfaceController<T...> {
protected:
  rm_control::ActuatorExtraHandle actuator_;               // 读写标定状态、offset（见 4.2）
  effort_controllers::JointVelocityController velocity_ctrl_;  // 速度 PID：按搜索速度驱动关节
  effort_controllers::JointPositionController position_ctrl_;  // 位置 PID：回位/后退用
  double velocity_search_;                                 // 搜索速度
  bool   calibration_success_;                             // 标定是否成功
  ros::ServiceServer is_calibrated_srv_;                   // 提供 is_calibrated 服务
  // starting() 每次强制 actuator_.setCalibrated(false)——每次启动都重标
};
```

**如何 claim / release joint。** 标定控制器内部持有一个 `effort_controllers::JointVelocityController`（速度 PID 子控制器）——它就是通过对关节施加力矩，让关节按搜索速度运动的手段。当标定控制器**启动（start）**时，它 claim（占用）目标 joint 的写入权；**停止（stop）**时释放。这里就是 overview 强调的关键约束：

> 一个 joint 同时只能被一个控制器 claim。所以标定拨盘时，必须先停 `shooter_controller`（它占着 trigger_joint），才能启动 `trigger_calibration_controller`。两者不能同时运行。

谁来保证"先停再启"这个顺序？是决策层的 `CalibrationQueue`，详见 [manual](./manual.md)。这里只需知道：joint 的独占性是硬约束，标定的整套编排都是为了在这个约束下安全地换手。

**如何检测堵转。** 就是 4.5 撞限位里那段 `countdown_` 逻辑——每周期读速度、够低就倒计时、复位或触发。它足够简单，能在一个 1kHz 周期内返回，不会阻塞控制环。这点很重要：标定控制器是控制环的一员，它每拍做的事（读速度、减计数、跑一次 PID）都必须快速返回。

### 4.7 为什么标定时其他机构还能动

回到 overview 里那个关键性质：**因为各控制器 claim 的 joint 集合互不相交，控制器可以独立启停。**

标定拨盘时，`CalibrationQueue` 只停了 `shooter_controller`、启了 `trigger_calibration_controller`——这俩都只碰 trigger/摩擦轮那几个 joint。底盘的 4 个轮子 joint、云台的 yaw/pitch joint 完全没被碰过，`chassis_controller` 和 `gimbal_controller` 照常在 1kHz 环里跑：

```
标定拨盘时的 1kHz 循环：
├── chassis_controller.update()  ← 照常，底盘能动
├── gimbal_controller.update()   ← 照常，云台能瞄
└── trigger_calibration_controller.update()  ← 只有拨盘在标定
```

这就是为什么比赛里可以一边标定拨盘、一边开着底盘跑位——标定不是"全车停摆"，而是精确地只冻结需要标定的那一个机构。这个独立启停的能力，正是整个标定体系能工作的前提。

---

## 5. 其他硬件

除了电机，硬件抽象层还统一管理了几类非关节外设——IMU、GPIO、ToF 雷达，它们各自对应一个自定义硬件接口（见 overview 的 6 接口表），跑在 1kHz 的 `rm_ecat_hw` 里。这里只做扫盲级介绍。

> **裁判系统是个例外，不在本节的硬件抽象层里**——它由独立的 `rm_referee` 节点读串口、跑在非实时的**决策层**，没有 `hardware_interface`。为避免误会，它挪到下面 §5.3 单独说明。

### 5.1 IMU（惯性测量单元）

IMU 提供机器人的姿态（朝向）和角速度，是云台稳定、底盘里程计的基础。RM 电机生态里 IMU 通常集成在云台板上（`gimbal_imu`）。

硬件层通过 `RmImuSensorInterface` 提供**滤波后的姿态四元数**（标准 ROS 的 `ImuSensorInterface` 只给原始加速度/角速度）。IMU 有个绕不开的问题是**零偏**——静止时读数也不为零。配置里给出零偏补偿值，或开启启动时的静止采样自动估算：

```yaml
imus:
  gimbal_imu:
    angular_vel_offset: [-0.003239, 0.002113, -0.001824]  # 陀螺零偏
    do_bias_estimation: false   # 是否启动时自动估算零偏
```

IMU 数据怎么变成坐标系（TF），是 [transform](./transform.md) 里 `orientation_controller` 的活。

### 5.2 GPIO

GPIO 就是通用输入输出引脚。硬件层用 `GpioStateInterface`（读）和 `GpioCommandInterface`（写）把它接进 ROS control：

- **读**：霍尔开关/微动开关的电平——正是 4.5 GPIO 标定要读的东西
- **写**：控制电磁铁、继电器等开关型执行器

对应的控制器是 `gpio_controller`。

### 5.3 裁判系统（属决策层，非硬件抽象层）

裁判系统（referee system）是 RoboMaster 官方的比赛裁判硬件，通过**串口**给机器人下发比赛状态、剩余血量、底盘功率上限、枪口热量等数据，也接收机器人上报的自定义数据（客户端 UI、地图等）。

它虽然也是一块物理接口板，但**和 IMU / GPIO / ToF 不是一回事**：那三者由 `rm_ecat_hw` 在 1kHz 里通过 `hardware_interface` 管理；裁判系统则由一个独立的 **`rm_referee` 节点**读串口、跑在非实时的**决策层**，没有任何 `hardware_interface`。它的数据只流向决策核心 `rm_manual`——底盘功率限制、发射热量限制都源于此。所以本系列把 `rm_referee` 归为**决策层的输入/输出驱动**（见 [overview](./overview.md) §3.1）；[chassis](./chassis.md)、[shooter](./shooter.md) 用到它的功率/热量数据，编排细节见 [manual](./manual.md)。放在本节只是顺带扫盲，别把它当成硬件抽象层的一部分。

### 5.4 ToF 雷达

ToF（Time of Flight）雷达是测距传感器，用光飞行时间测量到障碍物的距离，用于避障/防撞。硬件层用 `TofRadarInterface` 提供测距数据，对应 `tof_radar_controller`，只读并发布，不涉及关节。

---

## 6. 小结

- 控制器眼里是**关节**（弧度/牛·米），硬件是**执行器**（编码器计数/电流），中间靠 **Transmission** 翻译。
- 常用电机 GM6020（绝对式，免标定）、M3508 / M2006（增量式）、达妙（力控）。rm-controls 统一用**力矩模式**，速度/位置环在上位机软件实现。
- **增量式编码器上电不知道自己在哪**，这是标定存在的根源；只有既用增量式、又需要绝对角度的位置关节才要标定（底盘轮虽用增量式但只控速，不标定）。
- Transmission 三型：`SimpleTransmission`（1:1，负减速比=反装，带 offset）、`DifferentialTransmission`（差速耦合）、`DualActuatorTransmission`（双电机一关节）。
- 标定 = **找零 → 算 offset → 存 offset → 标记完成**；状态存在旁路的 `ActuatorExtraInterface`，硬件层标定时跳过限幅、标定后自动应用 offset。
- 三种方式：撞限位、读 Hall、差动。标定控制器继承 `MultiInterfaceController`，靠模板参数声明接口、claim/release joint、`countdown_` 检测堵转。
- 因为 joint 集合互不相交，**标定一个机构时其他机构照常运行**——这是标定体系的前提。

下一站 [transform](./transform.md)：关节和 IMU 的数据怎么变成一棵坐标系树（TF），供所有控制器查询。再往后是三个业务领域文档 [chassis](./chassis.md) / [gimbal](./gimbal.md) / [shooter](./shooter.md)。
