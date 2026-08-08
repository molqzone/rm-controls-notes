# 硬件：电机、编码器、Transmission 与标定

> **前置知识**：[overview](./overview.md) 的"硬件抽象层"概念、[communication](./communication.md)（理解命令是怎么送到电机的）

[communication](./communication.md) 讲的是命令怎么从上位机送到电机；这篇讲的是**链路终点的硬件本身**——电机是什么、它怎么知道自己转到了哪、控制器眼里的"关节"和电机之间隔着什么，以及为什么每次上电都要做一件叫"标定"的事。这是硬件抽象层最需要新人理解的一块。

读之前先把三层量分清。"原始整数"只应该活在线协议层，不能一路泄漏到控制器：

| 层次 | 典型表示 | 谁负责换算 |
| --- | --- | --- |
| **线协议** | `int16` 命令码、`0..8191` 单圈角度码、整数 rpm | EtherCAT Slave / CAN 电机协议驱动 |
| **执行器空间（actuator）** | 电机轴的 rad、rad/s、N·m | `ActData` / actuator interface |
| **关节空间（joint）** | 机构关节的 rad、rad/s、N·m | Transmission 之后交给控制器 |

协议驱动负责拆 CAN 字节、符号扩展和量纲缩放；**Transmission 只负责执行器空间与关节空间之间的机械传动关系**。标定再补上机械零点，使这个关系在上电和拆装后仍成立。

---

## 1. RoboMaster 常用电机

RoboMaster 生态里，控制器最常打交道的是这么几款电机。它们的差别不只是大小，还包括减速比、反馈分辨率、固件命令语义和电调内环。是否需要标定最终由“机构是否需要机械绝对零点”决定，不能只看编码器标签。

| 型号 | 减速比 | 力矩常数 | 编码器 | 典型用途 |
| --- | --- | --- | --- | --- |
| **GM6020** | 1:1（直驱） | 0.741 N·m/A（配置前核对手册） | 单圈角度反馈 | 云台 yaw / pitch |
| **M3508** | ≈19.2:1（3591:187） | 0.3 N·m/A（配置前核对手册） | 电机轴单圈角度反馈 | 底盘轮、摩擦轮 |
| **M2006** | 36:1 | 0.18 N·m/A（配置前核对手册） | 电机轴单圈角度反馈 | 拨弹盘、小负载关节 |
| **达妙（DM）** | 视型号与固件 | 视型号与固件 | 单圈/双编码器，能力依型号 | 需要复合位控/力控的关节 |

表里的参数只能用于建立初始配置。同一产品在不同电调固件、减速器和协议模式下可能有不同命令语义与缩放，最终以实物固件、驱动配置和台架校验为准。

### 1.1 三种控制模式

电机内置的电调（如 C620 对应 M3508）支持不同的控制"口味"，对应机器人学里三种最基本的量：

- **力矩控制（Effort）**：直接指定电机输出多大力矩（本质是控电流）。这是 RM 电机最底层、最通用的模式——rm-controls 里绝大多数控制器最终都是往关节写力矩。因为力矩控制最灵活，速度环、位置环都可以在上位机用软件 PID 叠在力矩之上实现
- **速度控制（Velocity）**：指定电机转多快。底盘轮、摩擦轮天然是速度需求
- **位置控制（Position）**：指定电机转到哪个角度。拨弹盘"每次精确转一格"、机械臂"转到某个姿态"是位置需求

这里有个 rm-controls 的关键设计选择：**控制器侧大多统一暴露 `EffortJointInterface`，速度环和位置环放在上位机。** 但"关节力矩接口"不等于所有电调在电线上都接收同一种物理量：

- C610/C620 一类电调通常接收电流参考，由电调完成换相、PWM 和内电流环。
- GM6020 的旧固件/配置可能把命令解释为电压参考，新固件也可能提供电流闭环；命令组帧、量程和转换系数必须与实际固件模式一致。
- 达妙（DM）的 MIT 模式接收复合命令，对应 [communication](./communication.md) 里的 `RmEcatMitSlave`。其典型语义是

$$
\tau_{cmd}=K_p(q_d-q)+K_d(\dot q_d-\dot q)+\tau_{ff}
$$

这里的 `P_MAX/V_MAX/T_MAX` 决定位置、速度和力矩字段的线性量化范围。主机与电机配置不一致时，报文仍可能合法但物理量比例已经错了，所以发送前必须按双方一致的范围钳位。切入 MIT 模式时先令 $q_d=q$、$\dot q_d=0$ 并渐进恢复增益，可避免目标沿用造成瞬时冲击。

### 1.2 力矩常数的意义

力矩常数（N·m/A）描述每安培电流对应的电机轴力矩。控制器算出关节力矩后，Transmission 先按减速比换到电机轴力矩，协议驱动再按力矩常数和电调量程换成命令码。

换电机或减速器后应**重新验证并按需整定**：正确的单位换算、方向和减速比必须先由硬件层吸收，不能靠 PID 增益补一个倍率错误；即使量纲已经正确，惯量、摩擦、带宽和饱和边界变化仍可能要求重新整定。

---

## 2. 编码器：单圈角、多圈展开与机械零点

电机反馈里常见的是一个模 $C$ 的**单圈角度码**，RM 电机通常取 $C=8192$。软件可按相邻样本的最短差把它展开成上电后的连续圈数：

$$
\Delta c_k=\operatorname{wrap}_{[-C/2,C/2)}(c_k-c_{k-1})
$$

$$
C_k=C_{k-1}+\Delta c_k,\qquad q_a=\frac{2\pi C_k}{C}
$$

这里隐含两个重要条件：相邻**有效**采样间电机轴转动必须小于半圈，且丢包后的时间间隔也要计入；否则软件可能沿错误方向展开。掉电后 $C_k$ 的历史通常会丢失，除非设备明确提供并保存多圈位置。

### 2.1 三件不能混为一谈的事

| 能力 | 它回答的问题 | 它不能自动回答的问题 |
| --- | --- | --- |
| 单圈绝对角 | 转子当前位于这一圈的哪个角度 | 已经转过多少圈 |
| 掉电保持多圈 | 断电前后累计转过多少圈 | 机构的机械零点在哪里 |
| 机械零点已知 | 当前关节相对机器人基准姿态是多少 | 编码器链路是否连续、方向是否正确 |

因此，“有绝对编码器”不等于“免标定”。GM6020 或达妙即使能给出单圈绝对角，安装齿位、机构拆装和关节基准仍可能要求静态 offset 或外部参考；达妙的双编码器也不自动等于掉电保持输出轴多圈。反过来，底盘驱动轮只关心速度，即使不知道机械零点也可以不做找零。

拨弹盘是典型反例：上电能读到一个合法的单圈角度，并不代表它正好对齐弹位。需要绝对弹位时，应通过 Hall、机械限位或可靠的保存零点把“编码器角”与“机构零位”对应起来。后文的标定控制器解决的正是这一步。

### 2.2 低速测速与量化

不少电调把速度按整数 rpm 返回。换到关节侧时：

$$
\dot q_j=\mathrm{rpm}\,\frac{2\pi}{60n}
$$

其中 $n$ 是带符号减速比。GM6020 直驱时 1 rpm 已约为 $0.1047\,\mathrm{rad/s}$，低速闭环很容易出现台阶。可以用展开角做 $m\Delta t$ 窗口差分来提高分辨率，但窗口越长延迟越大；云台还可结合经过坐标变换、时间对齐和零偏处理的 IMU 角速度，具体见 [gimbal](./gimbal.md) §2。

---

## 3. Transmission：关节和电机之间的翻译层

### 3.1 为什么需要它

控制器想的是"关节"：给 pitch 关节一个 0.3 rad 的目标、往 trigger 关节输出 2 N·m。执行器接口描述的是电机轴的 rad、rad/s 和 N·m；其下的协议驱动才处理编码器码和电流码。关节与电机轴之间还隔着减速箱、安装方向、零点偏移等机械现实。

**Transmission（传动）就是把关节空间和执行器空间互相翻译的一层。** 它是双向的：

```
控制器写命令：  关节力矩/位置  ──Transmission──►  电机轴力矩/位置   （下行）
读回反馈：      关节角度/速度  ◄──Transmission──  电机轴角度/速度   （上行）
```

这层翻译发生在硬件抽象层，控制器完全无感——控制器永远只跟"关节"打交道，Transmission 自动把它换算成电机认识的量。这正是 [communication](./communication.md) 全景图里那个"Transmission 换算"步骤。

Transmission 由 **URDF** 文件描述（`*.transmission.urdf.xacro`），硬件层启动时加载。rm-controls 用到三种。

### 3.2 SimpleTransmission：一个电机对一个关节

最常见的一种：一个电机通过减速箱驱动一个关节。它只需要两个静态参数——**带符号减速比**和 **URDF 关节 offset**：

```xml
<transmission name="trans_trigger_joint">
  <type>transmission_interface/SimpleTransmission</type>
  <actuator name="trigger_joint_motor">
    <mechanicalReduction>-27.5</mechanicalReduction>   <!-- 减速比 -->
  </actuator>
  <joint name="trigger_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <offset>3.55</offset>   <!-- 示例值，单位 rad；必须按实物核对 -->
  </joint>
</transmission>
```

- **减速比（mechanicalReduction）**：电机轴转角与关节转角的比例。负值同时表达安装方向反向。
- **URDF offset $q_0$**：模型中的静态关节零点，单位是 rad，随配置加载。

若将执行器侧位置、速度和力矩写作 $q_a,\dot q_a,\tau_a$，关节侧写作 $q_j,\dot q_j,\tau_j$，理想 `SimpleTransmission` 的关系是：

$$
q_j=\frac{q_a}{n}+q_0,\qquad \dot q_j=\frac{\dot q_a}{n}
$$

$$
\tau_j=n\tau_a,\qquad \tau_a=\frac{\tau_j}{n}
$$

位置/速度除以减速比，力矩则乘以减速比，这样才保持理想功率 $\tau_j\dot q_j=\tau_a\dot q_a$。$n<0$ 时方向符号也随之传递。

这里还要区分两个 offset：URDF 的 $q_0$ 是**静态模型参数**；`ActuatorExtraInterface` 里的 calibration offset 是**本次运行的执行器校正量**，在传播前修正 $q_a$。运行时标定不会动态改写 URDF，也不应把两者叠成一个含义不明的“零点数”。

### 3.3 DifferentialTransmission：差速器

有些机构用**差速器**：两个电机的转动通过差速齿轮耦合，共同决定两个关节的运动。典型场景是差动云台——两个电机一起转控制 pitch，反向转控制另一个自由度。

`DifferentialTransmission` 描述的就是这种"两个电机 ↔ 两个关节"的耦合关系。它的换算不再是简单的乘减速比，而是两个电机值的加减组合。因为两个电机是耦合的，它们的标定也必须一起做——这就是后面要讲的**差动标定**存在的原因。

### 3.4 MultiActuatorTransmission：多电机带一个关节

还有一种是**多个电机共同驱动同一个关节**——比如一个大负载关节，单个电机力矩不够，用两个电机并联出力。当前代码中的类名是 `MultiActuatorTransmission`：它接受多个 actuator 对一个 joint，关节命令力矩会按执行器数和各自减速比拆分。

要注意它不是一个对任意 N 电机都完整对称的融合器：当前实现把执行器 effort 求和为关节 effort，速度计算中却固定除以 2，而位置只取第一个 actuator 的位置。因此它的反馈公式实际按双电机机构写成；使用多于两个执行器或要求冗余位置反馈时，必须先补齐/验证实现，不能仅因类名是 `Multi` 就假设已完成通用融合。

### 3.5 三种 Transmission 小结

三种类型都是 ROS control `transmission_interface::Transmission` 抽象基类的实现：

```
transmission_interface::Transmission          （抽象基类）
  ├── SimpleTransmission            1 电机 : 1 关节 —— 绝大多数关节（轮子、拨盘、云台单轴）
  ├── DifferentialTransmission      2 电机 : 2 关节（耦合）—— 差动云台、差速机构
  └── MultiActuatorTransmission     N 电机 : 1 关节 —— 常见为双电机并联出力
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
> 对配置为需要找零的关节，rm-controls 可以**上电自动标定**：启动标定控制器后让机构寻找参考点，动态计算 actuator calibration offset 并写入内存。机械变更后仍应检查搜索方向、限位和安全空间，不能把自动找零当成无需验收。这套机制的核心是下文的 `ActuatorExtraInterface` + `CalibrationQueue` + 标定控制器三层配合。

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

配置里用 `need_calibration: true` 声明一个执行器需要标定。当前 EtherCAT 路径从 `rm_ecat_hw/device_configurations/*.yaml` 的 `can_motors` 读取它；遗留 CAN 路径则从 `rm_hw` 参数树读取同名字段，两个 YAML 结构不能直接互拷：

```yaml
# rm_ecat_hw/device_configurations/standard6_chassis.yaml
can_motors:
  - name: "trigger_joint_motor"
    type: "M3508"
    need_calibration: true    # ← 需要运行时找零
  - name: "left_front_pivot_motor"
    type: "GM6020"
    # 缺省 = false；是否找零取决于该机构配置
```

`calibrated` 每次控制器 `starting()` 时都被强制置回 false——也就是**每次启动都重新标定**，绝不信任上一次的零点。这是 RoboMaster"频繁重启、每次自动标定"现实的直接体现。

### 4.4 标定时硬件层的两个特殊行为

标定是个"鸡生蛋"的问题：还没标定完，关节的绝对位置就是错的，如果这时硬件层照常做关节限位保护，就会因为"位置不对"而误触发限幅，电机根本跑不到限位。当前 EtherCAT `write()` 的处理是：

1. **先正常限幅，再恢复标定执行器命令**：它先保存未限幅命令、执行所有 joint limit、重新传播；随后仅对 `needCalibration && !calibrated` 的 actuator 恢复先前的 effort。效果是标定执行器不受该 joint-space 限制卡住，其他 joint 仍然受限，并非全局“跳过限幅”。
2. **自动应用 offset**：读反馈时使用 `pos = 原始值 + offset`；标定完成、`calibrated = true` 后，上层控制器立刻拿到校正后的角度，无需额外换算。

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

适用关节：需要外部机械基准的云台 yaw/pitch、飞镖发射架俯仰/偏航等。

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

当前 EtherCAT 硬件层会同时注册标准 `ImuSensorInterface` 和 `RmImuSensorInterface`，两者指向同一份姿态四元数、角速度、线加速度与协方差数据；后者仅额外带有采样时间戳。滤波不是“选了 Rm 接口”才发生：遗留 CAN `rm_hw` 路径会解析 IMU filter/零偏配置，而 EtherCAT 路径把从站提供的采样值拷入接口。IMU 有个绕不开的问题是**零偏**——静止时读数也不为零。下面是遗留 CAN 配置中的示例：

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

它虽然也是一块物理接口板，但**和 IMU / GPIO 不是一回事**：前两者由当前 `rm_ecat_hw` 在 1kHz 路径通过 `hardware_interface` 管理；本工作树中的 ToF 接口属于遗留 CAN `rm_hw` 路径。裁判系统则由独立的 **`rm_referee` 节点**读串口、跑在非实时的**决策层**，没有任何 `hardware_interface`。本系列中的底盘功率和发射热量路径由 `rm_manual` 订阅其消息实现，所以把 `rm_referee` 归为**决策层的输入/输出驱动**（见 [overview](./overview.md) §3.1）；编排细节见 [manual](./manual.md)。放在本节只是顺带扫盲，别把它当成硬件抽象层的一部分。

### 5.4 ToF 雷达

ToF（Time of Flight）雷达是测距传感器，用光飞行时间测量到障碍物的距离，用于避障/防撞。硬件层用 `TofRadarInterface` 提供测距数据，对应 `tof_radar_controller`，只读并发布，不涉及关节。

---

## 6. 新鲜度、故障与无扰恢复

“收到过一帧”不等于“现在健康”。硬件层至少应分别维护下面四类状态，避免把所有异常压成一个 `online` 布尔值：

| 状态 | 典型判断 | 失效后的意义 |
| --- | --- | --- |
| **反馈年龄** | `now - feedback_stamp` | 电机状态是否仍可用于闭环 |
| **命令年龄** | `now - command_stamp` | 上层意图是否仍然新鲜 |
| **总线状态** | EtherCAT state、CAN error/passive/bus-off | 链路是否还能可靠收发 |
| **设备故障码** | 过压、欠压、过流、过温、编码器或驱动故障 | 不能当成普通掉帧自动忽略 |

阈值要从实际控制周期、总线更新率、制动距离和机构风险推导，教程中的固定毫秒数不能直接照抄。安全链通常按以下顺序处理：

1. 指令过期：停止沿用旧目标，进入该机构定义的零力矩、受控减速或安全保持状态。
2. 反馈过期或总线故障：闭环测量已经不可信，清除相关 PID 积分，撤销驱动命令并上报具体电机。
3. 电机故障码有效：锁存故障和首次时间，避免自动重试反复冲击；满足人工确认和恢复条件后再清除。

恢复时不要直接接回故障前目标。先用当前测量初始化位置目标和规划器，确认连续收到若干新鲜反馈，再按斜坡恢复输出；模式切换同样遵循 [control](./control.md) 中的无扰切换原则。硬件 watchdog、Linux 进程存活和命令新鲜度是三层不同保护，任何一层都不能替代另外两层。

---

## 7. 小结

- 线协议使用编码器码/电流码，**执行器空间和关节空间都应是 SI 单位**；协议驱动先解码，Transmission 再处理机械传动。
- 常用电机包括 GM6020、M3508/M2006 和达妙；上位机常用关节力矩接口，但 GM6020 固件模式、C610/C620 电流参考与达妙 MIT 复合命令的线语义并不相同。
- 单圈绝对角、掉电保持多圈和机械零点是三件事；是否需要标定取决于机构是否需要已知零点，而不是一个“绝对/增量”标签。
- Transmission 三型：`SimpleTransmission`（1 电机 : 1 关节，带符号减速比和静态 offset）、`DifferentialTransmission`（2:2 耦合）、`MultiActuatorTransmission`（N:1，常见 2:1）。运行时 calibration offset 与 URDF 静态 offset 分层管理。
- 标定 = **找零 → 算 offset → 存 offset → 标记完成**；状态存在旁路的 `ActuatorExtraInterface`，硬件层会在限幅后恢复未标定 actuator 的命令，并在读取时自动应用 offset。
- 三种方式：撞限位、读 Hall、差动。标定控制器继承 `MultiInterfaceController`，靠模板参数声明接口、claim/release joint、`countdown_` 检测堵转。
- 反馈年龄、命令年龄、总线状态和设备故障码必须分别跟踪；失联时清积分并进入安全输出，恢复时以当前状态重新初始化。

下一站先读 [control](./control.md)：闭环、PID、串级环和前馈怎样把关节状态变成稳定的力矩命令；再读 [transform](./transform.md)，理解关节和 IMU 数据怎样组成 TF 树。之后进入三个业务领域文档 [chassis](./chassis.md) / [gimbal](./gimbal.md) / [shooter](./shooter.md)。
