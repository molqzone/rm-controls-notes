# 决策层：上层怎么编排下层

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（标定）、以及三个机构文档 [chassis](./chassis.md) / [gimbal](./gimbal.md) / [shooter](./shooter.md)

前面几篇讲的都是**下层**：底盘怎么动、云台怎么瞄、子弹怎么打，还有它们背后的通信、硬件、标定、TF。这一篇讲**上层**——`rm_manual`，整个系统的决策中枢。理解了下层三个机构，才能看懂上层是怎么把它们编排起来的：什么时候启动谁、遥控器一拨要发生什么、上电后按什么顺序标定、指令怎么发下去、不同兵种的差异又藏在哪。

---

## 1. rm_manual 的定位

回到 overview 的三层架构：决策层在最上面，它**不参与 1kHz 实时控制环**。控制层的底盘/云台/发射控制器在 1kHz 里算力矩，而决策层跑在**约 100Hz** 的 `ros::Rate` 循环里（不是实时线程，就是普通 ROS 节点），通过 ROS 话题和下层通信。

决策层不止 `rm_manual` 一个节点——它是**决策核心 `rm_manual` + 一圈 I/O 驱动节点**（overview §3.1）。三个 I/O 驱动把物理接口翻译成话题喂给核心、也把核心的反馈发回去，全都非实时、都不碰控制环：

| I/O 驱动 | 输入 → rm_manual | rm_manual → 输出 | 物理链路 | 频率 |
| --- | --- | --- | --- | --- |
| `rm_dbus` | `DbusData`（DT7 遥控器拨杆/键鼠） | —— | 串口 `/dev/usbDbus` | 60Hz |
| `rm_vt` | `VTKeyboardMouseData`（图传链路键鼠） | 客户端 UI / 自定义数据 | 串口 `/dev/usbImagetran`（921600 baud） | 100Hz |
| `rm_referee` | 比赛状态/血量/功率/热量<br>+ 超级电容电源管理数据 | 客户端 UI / 地图交互<br>+ 超电状态指令 | 串口 `/dev/usbReferee`（115200 baud）<br>拓扑：裁判系统 → 超电 → 电脑<br>裁判系统 + 超电共线 | 80Hz，下行 UI 默认 150ms 间隔 |

本文主要讲**决策核心** `rm_manual` 自己（下面第 2 节起）；I/O 驱动只需记住：操作手输入和裁判系统数据是它们采集、以话题形式送进来的，`rm_manual` 从不直接读串口。各驱动都是独立 ROS 节点，不碰 1kHz 控制环、不直接操作电机。

`rm_referee` 的下行与上行是同一路串口：

- **下行**（接收）：80Hz 轮询 `Base::serial_.read()`，读到原始字节后按裁判系统协议解帧（`0xA5` 帧头 → CRC 校验 → cmd_id 分发 → publish 对应 topic），利用 `unpack_buffer_` 确保跨帧的正确拼接。一旦超过 5 秒收不到数据，置 `referee_data_is_online_ = false`。
- **上行**（发送）：由 `send_serial_data_timer_` 驱动（默认 150ms 间隔），将 UI 图形、字符、交互数据（按键、哨兵/雷达指令）组帧发回裁判系统客户端。

  裁判系统串口协议的 UI 帧有几种固定规格：一帧可携带 **1 图、2 图、5 图或 7 图**（不同 `cmd_id`），外加单独的字符帧。`sendQueue()` 每拍根据队列积压量选最划算的规格：

  ```
  队列 ≥7 张 → sendSevenGraph() 一次发 7 张
  队列 ≥5 张 → sendFiveGraph()  一次发 5 张
  队列 ≥2 张 → sendDoubleGraph() 一次发 2 张
  队列 =1 张 → sendSingleGraph() 一次发 1 张
  队列 ≤14 张且有字符 → sendCharacter() 穿插发一个字符
  ```

  先发字符还是先发图形有优先级：队列里图形少于 14 张时先处理一个字符（因为字符排队更敏感），否则按上述规格从大到小打包。发完一批后下一拍（150ms）继续处理队列剩余，避免串口拥塞。

注意：这一路串口的物理拓扑是**裁判系统 → 超级电容 → 电脑**，裁判系统和超级电容挂在同一条总线上、共用同一个 tty（`/dev/usbReferee`）。数据天然混在一起，`rm_referee` 按 `cmd_id` 区分：

- 裁判系统的比赛状态、血量、功率/热量等走标准 ID（如 `ROBOT_STATUS_CMD`、`POWER_HEAT_DATA_CMD`）
- 超级电容的电源管理数据走 `POWER_MANAGEMENT_*` 系列 ID（`POWER_MANAGEMENT_SAMPLE_AND_STATUS_DATA_CMD` 等），解包后 publish 成 `power_management/sample_and_status` 等 topic，供 `ChassisCommandSender` 里的 `PowerLimit` 状态机使用



`rm_manual` 管的是"**什么时候做、做什么**",不管"算法怎么算":

- 遥控器拨杆、键盘按键 → 该切哪个模式、发什么指令
- 什么时候启动 `chassis_controller`、什么时候切到标定控制器、谁来仲裁 joint 冲突
- 上电后先标定拨盘还是先标定云台，这个顺序谁来编排
- 英雄、工程、哨兵、无人机行为完全不同，但底层控制器可以复用——差异写在哪

它的主循环极简：

```cpp
ros::Rate loop_rate(100);           // ~100Hz
while (ros::ok()) {
  ros::spinOnce();                  // 处理 ROS 回调（遥控器、裁判系统、视觉…）
  manual_control->run();            // 执行决策逻辑
  loop_rate.sleep();
}
```

`spinOnce()` 消化掉这一拍收到的所有输入，同时也触发回调中附带的动作——比如 `dbusDataCallback` 末尾会调 `sendCommand()` 把决策打包发往下层。`run()` 做的是不依赖具体输入的同步决策，基类干三件事：

```cpp
void ManualBase::run() {
  checkReferee();                  // 裁判系统状态检查（电源事件、online 检测、publish manual_to_referee）
  ecat_reconnected_event_.update(ecat_bus_is_online_);
  controller_manager_.update();    // 刷新缓存的控制器开关请求
}
```

标定流水线的推进不在基类 `run()` 里——因为不是所有兵种都有标定控制器。有标定的子类（如英雄）在自己的 `run()` 里加上：

```cpp
void ChassisGimbalShooterManual::run() {
  ChassisGimbalManual::run();
  chassis_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  gimbal_calibration_->update(ros::Time::now());
}
```

`sendCommand` 则不是放在 `run()` 里统一调，而是挂在了 `dbusDataCallback` 末尾——因为指令发布的时机天然随遥控器输入走，断连时发的就是上一拍的旧值（控制器端有 `timeout` 保护兜底）。效果上两者都是 ~100Hz（遥控器数据就是这个频率），但附着点不同。

下面把这些拆开讲。它内部由四大块组成：**事件引擎（InputEvent）、控制器编排（ControllerManager）、标定流水线（CalibrationQueue）、指令发布（CommandSender）**。

> **一个包分工要先记住**：只有顶层的 Manual 类树和 `InputEvent` 在 `rm_manual` 包里；`ControllerManager` / `CalibrationQueue` / `CommandSender` 这些复用组件都在 `rm_common`（`rm_common/decision/`）里。而且这三者都建在一个 `ServiceCallerBase<>` 之上——它在**独立线程异步**调 ROS 服务，绝不在 100Hz 主循环里同步等待。正因为这套编排设施独立于任何具体机器人，11 种兵种才能共享它。下面各节的类都可以对号入座到这两个包。

---

## 2. 输入怎么变成事件：InputEvent

操作手的输入有两条物理通道，各由一个 I/O 驱动节点采集成话题：`rm_dbus` 发 `DbusData`（DT7 遥控器拨杆 + 键鼠），`rm_vt` 发 `VTKeyboardMouseData`（图传链路的键鼠）。`rm_manual` 订阅这些话题，不关心它们从哪条链路来——两者都灌进同一套 `InputEvent` 机制。下面讲的就是这套机制。

### 2.1 电平 vs 事件

遥控器的拨杆、按键、鼠标输出的都是**电平信号**（此刻是 0 还是 1）。但业务逻辑要的是**事件**——"拨杆*被拨到*了 UP 位"（一次动作），而不是"拨杆*现在在* UP 位"（每帧都成立的状态）。如果每帧去检查当前电平，就得自己记住上一帧是什么、手动比较，代码会很乱。

`InputEvent` 就是把"电平 → 事件"的转换封装起来的小工具。核心是记住上一次的状态，检测变化沿：

```cpp
void InputEvent::update(bool state) {
  if (state != last_state_) {                 // 状态变了
    if (state && rising_handler_)  rising_handler_();    // 0→1 上升沿
    if (!state && falling_handler_) falling_handler_();  // 1→0 下降沿
    last_state_ = state;
    last_change_ = ros::Time::now();
  }
  if (state && active_high_handler_)          // 保持为 1 期间，每帧回调（带时长）
    active_high_handler_(ros::Time::now() - last_change_);
}
```

### 2.2 四种触发方式

| 设置方法 | 触发时机 | 典型用途 |
| --- | --- | --- |
| `setRising` | 0→1 上升沿 | 按下按键、拨杆到位 |
| `setFalling` | 1→0 下降沿 | 松开按键 |
| `setActiveHigh` | 保持为 1 时每帧（传入保持时长） | 长按持续操作 |
| `setDelayTriggered` | 保持指定时长后触发一次 | 长按检测、防误触 |

用起来是这样的——绑定一次，之后每帧喂电平即可：

```cpp
// 构造时绑定：右拨杆拨到 UP 时切 PC 模式
right_switch_up_event_.setRising(boost::bind(&ManualBase::rightSwitchUpRise, this));

// 每帧在遥控器回调里喂当前电平
right_switch_up_event_.update(data->s_r == rm_msgs::DbusData::UP);
```

长按检测是个很实用的例子——发射就用它区分单发和连发：

```cpp
void leftSwitchUpOn(ros::Duration duration) {
  if (duration > ros::Duration(1.))        // 按住超过 1 秒 → 持续射击
    shooter_cmd_sender_->setMode(ShootCmd::PUSH);
  else if (duration < ros::Duration(0.02)) // 轻点 → 单发
    shooter_cmd_sender_->setMode(ShootCmd::PUSH);
  else
    shooter_cmd_sender_->setMode(ShootCmd::READY);
}
```

这里 `setMode` 设的正是 [shooter](./shooter.md) 里那个发射状态机的目标状态。事件引擎把"操作手的动作"翻译成"下层机构的指令"。

### 2.3 顶层遥控器状态机

在事件之上，ManualBase 还有一个顶层状态机，决定机器人整体处于什么操作模式，由遥控器右拨杆和连接状态驱动：

```cpp
enum { PASSIVE, IDLE, RC, PC };
```

| 状态 | 含义 | 触发 |
| --- | --- | --- |
| **PASSIVE** | 安全模式，所有控制器停止 | 遥控器断连 / 上电初始 |
| **IDLE** | 主控制器运行但不响应摇杆 | 遥控器 ON，右拨杆 Down |
| **RC** | 摇杆控底盘/云台，左拨杆控发射 | 右拨杆 Mid |
| **PC** | 键盘鼠标控制 | 右拨杆 Up |

**安全性优先**是这里的核心原则：遥控器一断连，立刻回到 PASSIVE、停掉所有主控制器和标定控制器——绝不让机器人失控暴走。这和 [chassis](./chassis.md) 里"底盘 timeout 后速度归零"是同一套安全思路，只不过在更高层。

```cpp
void ManualBase::remoteControlTurnOn() {           // 遥控器上电
  controller_manager_.startStateControllers();     // 先启状态发布（robot_state 等）
  controller_manager_.startMainControllers();      // 再启主控制器
  state_ = IDLE;
}
void ManualBase::remoteControlTurnOff() {          // 遥控器掉线
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
  state_ = PASSIVE;                                // 回安全模式
}
```

### 2.4 建议：重连后的武装互锁

边沿检测有一个危险初始化问题：机器人上电或遥控器重连时，射击拨杆/鼠标可能已经保持在“按下”。如果把第一次收到的高电平解释为上升沿，机器人会在操作手没有做新动作的情况下开火。当前 `InputEvent` / `ShooterCommandSender` 没有独立的 `armed` 状态或按 `ShootCmd.stamp` 拒绝过期指令；下面是一项应补充的安全设计，而不是现有实现。

发射输入因此需要一个独立的 `fire_input_armed` 锁存：

```cpp
if (!remote_fresh || state_ == PASSIVE || shooter_fault) {
  fire_input_armed = false;
  saw_release = false;
} else if (!fire_level) {
  saw_release = true;                  // 重连后先确认一次松开/中位
  fire_input_armed = true;
} else if (fire_input_armed && rising_edge) {
  requestFire();                       // 只有新的有效边沿才能开火
}
```

这里的重点不是变量名，而是规则：**静态射击电平不能当事件，失去安全前提后必须重新观察到释放。** 命令超时、发射掉电、标定未完成、热量保护和卡弹故障也都应撤销武装。长按连发仍可由 `InputEvent` 计时，但应建立在已经武装、输入持续新鲜的前提上。

---

## 3. 怎么切换控制器：ControllerManager

`ControllerManager` 是 rm_manual 和 ROS 原生 `controller_manager` 之间的桥。它要解决的核心问题，正是 [hardware](./hardware.md) 反复强调的那个约束：**多个控制器共用同一个 joint 时，必须互斥地启停**（一个 joint 同一时刻只能被一个控制器 claim）。

### 3.1 三类生命周期

它把所有控制器按生命周期分成三类（对应 overview 讲的分类）：

```cpp
std::vector<std::string> state_controllers_;       // 状态发布
std::vector<std::string> main_controllers_;        // 主控制
std::vector<std::string> calibration_controllers_; // 标定
```

- **state_controllers**（如 `robot_state_controller`、`joint_state_controller`）：遥控器 ON 时启动、始终运行。为什么要独立成一类？因为 `robot_state_controller` 必须**先于**其他控制器启动——它提供 [transform](./transform.md) 讲的 `RobotStateInterface`（TF 总线），别的控制器都依赖它查坐标系。
- **main_controllers**（`chassis` / `gimbal` / `shooter` / `orientation`……）：正常运行时的主控逻辑。
- **calibration_controllers**（`trigger_calibration_controller`……）：仅在标定阶段短暂运行。

三类清单从配置里读（`controllers_list`），启动时预加载。

### 3.2 安全切换：缓冲 + 异步

切换控制器不是立刻同步执行的，而是**先写进缓冲区，再由 `update()` 异步调用**一次性完成：

```cpp
void ControllerManager::update() {
  if (!switch_caller_.isCalling()) {              // 上一次切换还没完成就不叠加
    switch_caller_.startControllers(start_buffer_);
    switch_caller_.stopControllers(stop_buffer_);
    if (!start_buffer_.empty() || !stop_buffer_.empty()) {
      switch_caller_.callService();               // 异步调用（新线程），不阻塞主循环
      start_buffer_.clear(); stop_buffer_.clear();
    }
  }
}
```

这种"缓冲 + 一次原子切换"的设计防止了标定流水线频繁切换时的竞态——ROS 的 `switch_controller` 保证先 stop 再 start，不会出现两个控制器同时 claim 一个 joint 的中间态。

---

## 4. 标定流水线：CalibrationQueue

[hardware](./hardware.md) 里讲了单个标定控制器怎么工作（撞限位、读 Hall、差动），但没讲**谁来按顺序编排它们**——上电后先标拨盘还是先标云台？一个标完再标下一个的节奏谁控制？答案就是 `CalibrationQueue`。

### 4.1 它是一个编排器

一条标定流水线（比如 `shooter_calibration`）是若干**标定步骤**的有序列表，每个步骤描述"启动哪些标定控制器、停止哪些主控制器、轮询哪些完成服务":

```yaml
shooter_calibration:
  - start_controllers: [controllers/trigger_calibration_controller]  # 启动标定控制器
    stop_controllers:  [controllers/shooter_controller]              # 停止主控制器（让出 joint）
    services_name: [/controllers/trigger_calibration_controller/is_calibrated]  # 轮询是否标完
```

`CalibrationQueue` 本身不控制电机、不做决策，只做一件事：**按顺序 启停控制器 + 轮询结果**。它用两阶段切换保证标定控制器和主控制器永不同时运行：

```cpp
void CalibrationQueue::update(const ros::Time& time, ...) {
  if (switched_) {                                    // 已切到标定控制器，等它标完
    if (calibration_itr_->isCalibrated()) {           // 这一步标定完成
      controller_manager_.startControllers(itr->stop_controllers);   // 恢复主控制器
      controller_manager_.stopControllers(itr->start_controllers);   // 停掉标定控制器
      calibration_itr_++;                             // 前进到下一步
      switched_ = false;
    } else if (每 200ms) itr->callService();          // 轮询 is_calibrated 服务
  } else {                                            // 准备这一步
    switched_ = true;
    controller_manager_.startControllers(itr->start_controllers);    // 启动标定控制器
    controller_manager_.stopControllers(itr->stop_controllers);      // 停止主控制器
  }
}
```

这就是 [hardware](./hardware.md) 结尾"谁来保证先停 shooter 再启标定"的答案。而"标定时底盘云台照常动",是因为这里只停了 shooter、动的 joint 集合互不相交（见 hardware 第 4.7 节）。

### 4.2 上层只需 reset + update

对 rm_manual 来说，用标定流水线只有两个动作：在合适的时机 `reset()`（把迭代器拨回开头，重新标），每帧 `update()`（推进）。**什么时候 reset** 由裁判系统事件驱动——底盘/云台/发射电源 ON、比赛自检、比赛开始、EtherCAT 断线重连、手动 Ctrl+Q，都会触发重标：

```cpp
void chassisOutputOn() { chassis_calibration_->reset(); }   // 底盘电源 ON
void gimbalOutputOn()  { gimbal_calibration_->reset(); }
void shooterOutputOn() { shooter_calibration_->reset(); }
void ecatReconnected() { shooter_calibration_->reset(); gimbal_calibration_->reset(); }
void ctrlQPress()      { /* 全部 reset，手动重标 */ }
```

为什么要标这么多次？因为 RoboMaster 是"频繁重启、机械可能被撞歪"的比赛环境——每次电源上电、每次开赛前，都重标一遍确保零点是最新的，这是有意的鲁棒性设计。

---

## 5. 指令发布：CommandSender

rm_manual 从不直接操作控制器，而是**发布 ROS 话题**间接控制。`CommandSender` 系列封装了"把决策打包成消息、发到对应控制器的 command 话题"这件事。这正是 overview 说的"决策层 ↔ 控制层走 ROS 话题"。

每个下层机构对应一个 sender：

```
ChassisCommandSender  → /cmd_chassis                              → chassis_controller
GimbalCommandSender   → /controllers/gimbal_controller/command    → gimbal_controller
ShooterCommandSender  → /controllers/shooter_controller/command   → shooter_controller
```

它们挂在一棵共同的类树上（都在 `rm_common`）——基类管发布骨架，`TimeStampCommandSenderBase` 补上时间戳，各机构子类填各自的消息构建：

```
CommandSenderBase<MsgType>
  └── TimeStampCommandSenderBase<>
        ├── ChassisCommandSender   内含 PowerLimit（§6.1 CHARGE/NORMAL/BURST）
        ├── GimbalCommandSender
        ├── ShooterCommandSender   内含 heat_limit_（§6.2）
        └── Vel2DCommandSender / MultiDofCommandSender / …
```

以发射为例，事件引擎调 `setMode` 设好意图，`sendCommand` 打包发布，[shooter](./shooter.md) 的控制器在 1kHz 环里读取执行：

```cpp
class ShooterCommandSender : ... {
  void sendCommand(const ros::Time& time) override {
    msg_.mode = mode_;               // STOP/READY/PUSH —— shooter 状态机的目标
    msg_.wheel_speed = wheel_speed_; // 摩擦轮转速 → 决定子弹初速
    msg_.hz = hz_;                   // 射频 —— 受热量限制约束（见下节）
    publisher_.publish(msg_);
  }
  void setMode(uint8_t m) { mode_ = m; }        // 上层按键事件调用
  void setBulletSpeed(double s) { wheel_speed_ = s; }
};
```

**指令-控制分离**：sender 只管打包和发布，不管执行；控制器只管执行，不管"该不该做"。这是分层设计在指令通道上的体现。

### 5.1 新增一条指令通道：怎么做、要当心什么

新增一个机构、或给现有指令加字段时，会碰到"加一条指令通道"这件事。从消息定义到接进 Manual，完整步骤：

1. **定义消息**：在 `rm_msgs` 里加 `XxxCmd.msg`（字段 + 建议带 `std_msgs/Header` 或 `float64 stamp`），重新编译生成头文件。
2. **写 CommandSender 子类**：继承 `CommandSenderBase<rm_msgs::XxxCmd>`；**如果控制器要判指令时效**，改继承 `TimeStampCommandSenderBase<>`（它自动填时间戳）。覆写 `sendCommand(time)`：把内部状态写进 `msg_` 再 `publisher_.publish(msg_)`；加若干 `setXxx()` 供上层设意图。
3. **控制器侧订阅**：对应控制器订阅这个 command 话题，用 `RealtimeBuffer`（非实时回调 `writeFromNonRT` 写、`update()` 里 `readFromRT` 读）安全取用——就是 [transform](./transform.md) 那套跨线程模式。
4. **接进 Manual**：在对应 Manual 子类里持有这个 sender、构造时从参数读 `topic`、把它绑到 `InputEvent` 回调（按键→`setXxx`），并在 `sendCommand(time)` 聚合里调它的 `sendCommand()`。
5. **配置**：`rm_manual/<robot>.yaml` 给它一个 `topic`，且必须和控制器订阅的话题名**完全一致**。

几个最容易踩的坑：

- **每拍都要发，不是"变了才发"**：`sendCommand` 得在每个决策周期被调用（`rm_manual` 里它挂在 `dbusDataCallback` 末尾，由 `spinOnce` 驱动在 ~100Hz）。很多控制器有 `timeout` 保护（如 [chassis](./chassis.md)），一旦一段时间收不到指令就当断连、速度归零/急停。只在值变化时发布会触发误急停。
- **要判时效就必须带时间戳**：控制器如果检查指令新鲜度，sender 必须用 `TimeStampCommandSenderBase` 并让 `stamp = ros::Time::now()`，否则控制器会一直认为指令过期而拒绝执行。
- **话题名一致是头号哑火原因**：sender 配的 `topic` 和控制器订阅的对不上，就静默失联、毫无报错。这和 [hardware](./hardware.md) 里电机名四层一致是同一类坑。
- **sender 只打包、不写控制逻辑**：闭环、状态机都在控制层。裁判系统约束（功率/热量）可以像 `ChassisCommandSender`/`ShooterCommandSender` 那样内嵌 `PowerLimit`/`heat_limit_` 子模块（§6），但那是"约束指令"，不是"执行指令"。
- **给安全默认值**：`msg_` 要有安全初值（如 `mode = STOP`、速度 0）；遥控器掉线/进入 PASSIVE 时应发安全值，别让机构带着上一条指令乱跑（呼应 §2.3 的 `remoteControlTurnOff`）。

一句话：**加指令 = 定义 msg → 写 sender 子类 → 控制器订阅（RealtimeBuffer）→ 接进 Manual 的事件与聚合发布 → 配好一致的 topic**；当心"每拍发、带时间戳、话题名一致、只打包不执行"。

### 5.2 建议：安全仲裁与状态机边界

当前 `rm_manual` 和发射控制器分别有掉线处理、模式切换、卡弹与热量逻辑，但没有一个覆盖所有条件的统一 fail-closed 总门。下面是建议的仲裁架构，不能当作当前功能说明。

一个机构往往同时有输入手势、正常动作、卡弹恢复、热量预测等多个状态机。它们可以并行或嵌套，但最终发命令前应经过**单向的优先级仲裁**：

```
掉线 / 急停
  > 机构掉电 / 未标定
  > 故障锁定
  > 自动恢复
  > 正常操作意图
```

高优先级条件一旦成立，低优先级状态不得把命令改回危险值。进入或退出外层状态时，要重置内层计时器、积分、一次性事件和待执行目标。例如从“卡弹恢复”退出不能保留恢复前的长按事件，否则下一拍可能直接重新 PUSH。

自动开火还应把操作手和视觉看成**双重许可**：操作手保持开火意图只是第一道门，视觉目标与 `fire` 判定必须带独立时间戳并保持新鲜；视觉“不可信”、超时或目标丢失都应 fail closed。人工“信任视觉”开关只能允许使用这路数据，不能替代新鲜度检查。完整发射许可条件见 [shooter](./shooter.md) §2.1。

所有状态持续时间都用 `ros::Duration` 或累计真实 `period`，不要用固定拍数代替秒数。控制频率调整、某拍超时或仿真步长变化后，固定拍数会悄悄改变安全阈值。

---

## 6. 裁判系统交互：功率与热量管理

裁判系统（[hardware](./hardware.md) §5.3 介绍过）是决策层的重要输入，由 I/O 驱动节点 **`rm_referee`** 读串口、拆成一堆话题——比赛状态、血量、功率/热量、电容、射击数据——`rm_manual` 订阅这些话题据此做决策，也通过 `rm_referee` 把客户端 UI / 地图数据发回裁判系统。两个最典型的用途是功率和热量管理，它们都藏在 CommandSender 里。

### 6.1 底盘功率管理

[chassis](./chassis.md) 讲过底盘控制器会做功率约束；决策层的职责是选择当前走哪条预算分支。`ChassisCommandSender` 内嵌一个 `PowerLimit` 状态机，封装在 `rm_common/decision/power_limit.h` 里。这里下发的 `power_limit` 是**预算**，不是对实际电气功率的测量保证；控制层仍要结合电机模型、反馈新鲜度和执行器限幅落实它。

#### 5 种模式

```cpp
typedef enum {
  CHARGE = 0,  // 降低本地底盘预算的分支
  BURST = 1,   // 请求高预算的分支
  NORMAL = 2,  // 常规预算分支
  ALLOFF = 3,  // 本地状态机中的关闭状态
  TEST = 4,
} Mode;
```

#### 控制决策树

`setLimitPower()` 的决策逻辑（以英雄/步兵为例，工程直接 400W 不参与）：

```
裁判系统不在线
  → safety_power_（安全功率，底盘能动但受限）
裁判系统在线
  └─ 超电不在线 或 expect_state_ == ALLOFF
       → normal()：仅用裁判系统 chassis_power_limit_
  └─ 超电在线
       └─ chassis_power_limit_ > burst_power_
            → burst_power_（裁判限值本身已超爆发上限，直接钳位）
         └─ 按 expect_state_ 分流
              ├─ NORMAL → normal()
              ├─ BURST  → burst(chassis_cmd, is_gyro)
              ├─ CHARGE → charge()
               └─ default → zero()（功率=0，底盘动不了）
```

各分支结束后还会调用 `applyPosturePowerScale()`。当前成员默认值为 1.0，且这份 `PowerLimit` 没有对外设置它的方法，因此按当前代码通常不改变上面的结果。

> **配置核对**：当前 `PowerLimit` 读取的是 `enable_burst_cap_threshold`、`disable_burst_cap_threshold`、`disable_normal_cap_threshold`、`enable_gyro_cap_threshold`、`disable_gyro_cap_threshold`、`gyro_power` 和 `upstairs_power` 等键。现有多份 `rm_manual/*.yaml` 仍使用 `enable_use_cap_threshold`、`enable_cap_gyro_threshold`、`charge_power`、`standard_power` 等旧键名；它们不是源码中的别名，会触发缺参日志并让相应成员保留默认值。修改车辆配置前应按实际构造函数逐项核对，不能只沿用旧 YAML。

> 当前实现不应被概括成“旧超电两跳、新超电一跳”。`PowerLimit` 是 NUC 侧的本地预算计算器：它根据裁判状态、电源管理数据和 `expect_state_` 写入 `ChassisCmd.power_limit`，自身不发送超电串口控制帧。
>
> `rm_manual` 同时会把 `power_limit_state` 放进 `ManualToReferee`。在当前仓库中，`rm_referee` 只把它交给 `ChassisTriggerChangeUi` 更新 UI 图形颜色；没有源码证据证明颜色或 `power_limit` 会直接控制超电。因此，旧/新超电协议、所谓“几跳”以及超电固件如何决定充放电，都需要对应固件或协议文档，不能从这里推导。
>
> 配置中仍可见 `is_new_capacitor`，但当前 `PowerLimit` 不读取该字段；它不是当前行为分支。

#### 三种核心模式

**BURST**——高预算分支：

```cpp
void burst(rm_msgs::ChassisCmd& chassis_cmd, bool is_gyro) {
  if (cap_state_ != ALLOFF && cap_energy_ > capacitor_threshold_
      && chassis_power_buffer_ > power_buffer_threshold_) {
    if (is_gyro)
      setGyroPower(chassis_cmd);     // 小陀螺模式，用 gyro_power_
    else
      setBurstPower(chassis_cmd);    // 正常爆发，用 burst_power_
  } else
    expect_state_ = NORMAL;           // 条件不满足，回落 normal
}
```

`setBurstPower()` 内部还有迟滞保护（`enable_burst_cap_threshold_` / `disable_burst_cap_threshold_`），防止超电容量在阈值附近来回切。`setGyroPower()` 同理，用另一套 `enable_gyro_cap_threshold_` / `disable_gyro_cap_threshold_`。

这里的 `is_gyro` 指持续主动自转的小陀螺行为，不等同于底盘正面追随云台的 FOLLOW；两者的模式边界见 [chassis](./chassis.md) §3。

**CHARGE**——降低本地底盘预算：

```cpp
void charge(rm_msgs::ChassisCmd& chassis_cmd) {
  allow_use_cap_ = false;
  chassis_cmd.power_limit = chassis_power_limit_ * 0.70;  // 只给 70% 限制功率
}
```

**NORMAL**——常规预算分支：

```cpp
void normal(rm_msgs::ChassisCmd& chassis_cmd) {
  allow_use_cap_ = false;
  if (cap_state_ != ALLOFF && cap_energy_ > disable_normal_cap_threshold_
      && chassis_power_buffer_ > power_buffer_threshold_)
    chassis_cmd.power_limit = chassis_power_limit_ + extra_power_;
  else
    chassis_cmd.power_limit = chassis_power_limit_;
  if (chassis_cmd.power_limit > max_power_limit_)
    chassis_cmd.power_limit = max_power_limit_;
}
```

注意 normal 模式下的 `extra_power_`：当前代码仅在电源管理状态非 `ALLOFF`、容量高于阈值且裁判功率缓冲足够时，把 `chassis_power_limit_ + extra_power_` 写入命令，再由 `max_power_limit_` 钳位。不要把它归因到某种“旧/新超电”协议；某车辆把它设为 0 只是配置选择。

CHARGE 并不是“功率很低”或“免费充电”，而是当前代码主动把本地驱动预算乘以 `0.70`。各迟滞阈值和 `0.3s` 在线判据都只是当前代码/车辆配置的快照，必须结合当季规则、链路频率和实车能量测量验证。更完整的“数据陈旧即撤销 BURST、连续新鲜样本后再恢复”是应补充的安全策略，不是 `PowerLimit` 当前实现的完整状态机。

#### 当前数据路径

**输入（裁判系统 / 电源管理模块 → NUC）**：电源管理数据由 `rm_referee` 解包并发布到 `power_management/sample_and_status`。`ManualBase` 将其转交给 `PowerLimit::setCapacityData()`；这个函数当前实际使用的字段如下：

| 消息字段 | `PowerLimit` 的当前用途 |
| --- | --- |
| `stamp` | 每次收到消息时与当前时间比较；小于 0.3 s 时置 `capacity_is_online_ = true` |
| `capacity_remain_charge` | 写入 `cap_energy_` |
| `state_machine_running_state` | 写入 `cap_state_` |
| `chassis_power` / `capacity_discharge_power` | 字段会随消息到达，但此 `PowerLimit` 当前不读取 |

`chassis_power_buffer_` 则来自独立的裁判 `PowerHeatData` 消息。裁判在线在 `ManualBase::checkReferee()` 中每轮按最后一条 `PowerHeatData` 的时间戳重新判断；容量在线只在 `setCapacityData()` 收到消息时更新，当前没有“0.3 s 后自动清 false”的独立计时器。两者不能混为一个信号，后者也是现有代码需要补强的点。

物理拓扑通常是**裁判系统 → 超级电容 → 电脑**，所以串口数据异常可能来自链路中任一环节；仅凭本仓库不能把裁判数据丢失归因到超电故障。

**状态展示（NUC → 裁判系统客户端 UI）**：`ManualBase::checkReferee()` 发布 `ManualToReferee`；`rm_referee::RefereeBase::manualDataCallBack()` 将其交给 `ChassisTriggerChangeUi`。该 UI 对应 BURST = 橙色、CHARGE = 绿色、NORMAL = 白色、其他状态 = 黑色，并进入 UI 队列后作为裁判系统 UI 帧发送。

```
rm_manual                              rm_referee
PowerLimit::getState()
  └─ ManualToReferee ───────────────► RefereeBase::manualDataCallBack()
                                          └─ ChassisTriggerChangeUi
                                               └─ UI 图形 / 串口 UI 帧
```

这条 UI 路径是状态展示，不应被描述为 NUC 经裁判系统向超电发送控制命令。`PowerLimit::updateState()` 的输入来自 `rm_manual` 自己的事件处理；两条路径在当前代码中彼此独立。

### 6.2 发射热量管理

[shooter](./shooter.md) 说过"热量限制不在发射控制器、而在决策层"——就在这里。`ShooterCommandSender` 里的 `heat_limit_` 根据配置选择本地累计热量或裁判上报热量，再结合裁判给出的上限和冷却速率计算射频 `hz`。配置在 rm_manual：

```yaml
shooter:
  heat_limit:
    low_shoot_frequency:  1
    high_shoot_frequency: 3
    burst_shoot_frequency: 6
    minimal_shoot_frequency: 1
    safe_shoot_frequency: 1
    heat_coeff: 1
    local_heat_protect_threshold: 0
    use_local_heat: true
    type: "ID1_42MM"        # 弹丸类型，决定单发热量与上限
```

当前 `HeatLimit` 确实接收两类信息，但没有把它们做成带时间戳的融合器：

```
rm_referee：官方热量 / 上限 / 冷却速度（权威、但有链路延迟）
                                 ┐
                                 ├─► heat_limit_ ─► 安全射频 hz ─► ShootCmd
shooter_controller：疑似射出事件（低延迟、但会误报/漏报）
                                 ┘
```

当前行为是：`ShooterCommandSender` 用 `HeatLimit` 计算 `ShootCmd.hz`，控制器只执行射频。`use_local_heat = true` 时用本地累计热量；否则使用裁判上报的枪口热量。它以 `has_shoot` 的 false → true 边沿把本地热量加上单发热量，并由 0.1 s 定时器按冷却速度递减；裁判离线时直接返回固定 5.0 Hz，BURST 模式直接返回配置的爆发射频。

要把这两路数据融合得保守，下面四条是**建议的后续设计**，当前 `HeatLimit` 尚未实现：

1. 官方样本和本地射出事件都带时间戳；
2. 收到官方热量后先按冷却模型传播到当前时刻，再叠加样本时间之后的本地事件；
3. 为已下发未检测的弹、漏检与通信延迟预留安全热量，不能把显示余量全部花完；
4. 官方数据陈旧时采用本地预测的保守上界并降频，不能把掉线解释成“当前热量为 0”。

本地预测方程、持续射频 $c/h$ 和分段降频策略见 [shooter](./shooter.md) §5。每发热量、冷却速度和上限应来自当季裁判数据/配置；教程中的固定数值只代表当时规则，不应写死。

裁判系统还驱动一系列安全事件：血量归零 → `robotDie()` 停掉所有主控制器；复活 → 恢复；各机构电源 ON → 触发对应标定（第 4.2 节）。rm_manual 也会**发回**反馈给裁判系统（功率状态、当前射频、视觉检测颜色等）。

---

## 7. 兵种差异如何体现：继承体系

英雄、工程、哨兵、无人机的行为天差地别，但底层控制器（底盘/云台/发射）是复用的。差异怎么组织？rm_manual 用**继承 + 模板方法**，而不是在一个大类里写满 `if (robot_type == "hero")`。

```
ManualBase                              通用：遥控器状态机、裁判系统、ControllerManager
  └── ChassisGimbalManual               底盘 + 云台：摇杆映射、速度控制
        ├── ChassisGimbalShooterManual  英雄：+ 发射、弹道、标定、视觉跟踪
        │     └── ...CoverManual        步兵：+ 弹仓盖
        ├── EngineerManual              工程：+ 机械臂、动作服务器、GPIO、矿石收集
        ├── DartManual                  飞镖：+ 飞镖发射、GPIO 标定
        └── DroneManual                 无人机（独立分支）
BalanceManual（直接继承 ManualBase）      两轮平衡（无标准底盘/云台）
```

**启动时按 `robot_type` 实例化对应子类**：

```cpp
robot = getParam(nh, "robot_type", "error");
if (robot == "hero")      manual = new ChassisGimbalShooterManual(...);
else if (robot == "standard") manual = new ChassisGimbalShooterCoverManual(...);
else if (robot == "engineer") manual = new EngineerManual(...);
// ...
```

**模板方法**的意思是：基类定好骨架流程（遥控器回调、发指令的时机），子类只覆写需要定制的"钩子"。比如 `ChassisGimbalShooterManual` 覆写 `leftSwitchUpOn` 加上发射逻辑，`EngineerManual` 覆写按键处理加上几十个机械臂专用绑定。共享的 80% 逻辑留在基类，兵种独有的 20% 落在子类——这就是 11 种机器人能复用同一套下层控制器的原因。

> 本文不深入各子类的代码细节（工程的动作序列、飞镖的 GPIO 标定等），只需理解这个组织方式：**共性上提、差异下沉**。

---

## 8. 配置项说明

决策层配置在 `rm_manual/<robot>.yaml`，是和操作手体验最直接相关的一层：

```yaml
rm_manual:
  robot_type: "hero"                  # 决定实例化哪个子类
  dbus_topic: "/rm_ecat_hw/dbus"      # 遥控器话题

  chassis: { follow_source_frame: "yaw", safety_power: 60, ... }
  vel:                                # 速度-功率分段映射（功率越高限速越高）
    max_linear_x:
      - [ 50.0, 1.7 ]                 # [功率上限, 对应最大线速度]
      - [ 220.0, 5.0 ]
  gimbal: { max_yaw_vel: 3.14, track_timeout: 0.5, ... }
  shooter:
    heat_limit: { low_shoot_frequency: 1, high_shoot_frequency: 3, type: "ID1_42MM" }

  controllers_list:                   # ControllerManager 读的三类清单
    state_controllers:  [controllers/robot_state_controller, controllers/joint_state_controller]
    main_controllers:   [controllers/chassis_controller, controllers/gimbal_controller, controllers/shooter_controller]
    calibration_controllers: [controllers/trigger_calibration_controller]

  shooter_calibration:                # 标定流水线（CalibrationQueue 读）
    - start_controllers: [controllers/trigger_calibration_controller]
      stop_controllers:  [controllers/shooter_controller]
      services_name: [/controllers/trigger_calibration_controller/is_calibrated]
```

常见调整：

- **调底盘最大速度** → `vel.max_linear_x` 分段映射（不在 chassis 控制器里）
- **调射频/热量策略** → `shooter.heat_limit`
- **加一个控制器** → 在 `controllers_list` 对应类别里加一行；若是标定控制器，还要加一条标定流水线
- **加一个需标定的关节** → 参见 [hardware](./hardware.md) 的标定 + 这里的 `calibration_controllers` 与流水线；硬件路径、URDF Transmission、`rm_controllers` 和 `rm_manual` 中的 actuator / joint / controller 名称必须逐项核对
- **改按键/拨杆映射** → 在对应 Manual 子类的事件绑定里（代码层）

> 硬件配置有两条启动路径，不能混写为同一条链：
>
> - **EtherCAT**：`rm_ecat_hw.launch` 传入 `config/rm_ecat_hw/<robot>.yaml`；该 setup 文件再引用 `device_configurations/*.yaml`，其中定义 `can_motors`、IMU、GPIO 等设备。
> - **遗留 CAN**：`rm_can_hw.launch` 加载 `config/rm_control/rm_hw/actuator_coefficient.yaml` 和 `config/rm_control/rm_hw/<robot>.yaml`，再启动 `rm_hw`。
>
> 两条路径最终都要与 URDF Transmission、`rm_controllers/<robot>.yaml` 和 `rm_manual/<robot>.yaml` 的名称及控制器关系一致。`rm_referee/<robot>.yaml` 是独立的裁判串口/UI 配置；机器人 ID、功率和热量由裁判消息提供，不是这条 actuator 配置链的下一层。

---

## 9. 小结

- **决策层 = 决策核心 `rm_manual` + I/O 驱动 `rm_dbus`/`rm_vt`/`rm_referee`**（全非实时）。驱动采集遥控器/图传/裁判系统 → 话题 → 核心，核心的 UI/反馈也经它们发回；`rm_manual` 本身不读串口。
- **rm_manual 是决策核心**，跑在约 100Hz 的 `ros::Rate` 循环（`run()` + `spinOnce` 驱动的回调），不参与 1kHz 控制环，通过 ROS 话题编排下层——管"什么时候做、做什么"。
- **InputEvent** 把遥控器/键盘的电平信号转成上升沿/下降沿/长按等事件，之上还有 PASSIVE/IDLE/RC/PC 顶层状态机，断连即回安全态；独立武装互锁与命令新鲜度总门仍是建议补强项。
- **ControllerManager** 按 state / main / calibration 三类生命周期管理控制器，用缓冲 + 异步原子切换避免 joint 冲突竞态。
- **CalibrationQueue** 编排标定步骤：按顺序"启标定控制器 + 停主控制器 + 轮询完成",上层只需 `reset()` + `update()`；由裁判系统电源/比赛事件触发重标。
- **CommandSender** 把决策打包成消息发给下层控制器，指令与执行分离。统一安全仲裁、自动开火双重许可与命令新鲜度总门是建议补强项，不是当前完整实现。
- **裁判系统交互**：`PowerLimit`（底盘预算 CHARGE/NORMAL/BURST）和 `HeatLimit`（发射热量→射频）都在 CommandSender 里；当前热量模型是本地事件计数器或裁判热量二选一，不是带时间戳的融合器。
- **兵种差异**用继承 + 模板方法组织：共性在基类，差异在子类，11 种机器人复用同一套下层控制器。

到这里，从最底层的通信硬件、到中间的控制算法、再到最上层的决策编排，整个 rm-controls 的心智模型就完整了。回到 [overview](./overview.md) 那张三层架构图，现在每一格应该都能填上具体内容了。
