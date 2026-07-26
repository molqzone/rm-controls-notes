# 发射：子弹是怎么打出去的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（拨盘为什么要标定、力矩/速度/位置控制）

发射控制器（`shooter_controller`）claim 的是两个摩擦轮 joint 和一个拨弹盘 joint，负责把一颗颗弹丸稳定、精确地打出去。这篇讲清楚：发射机构由什么组成、四状态发射状态机、摩擦轮转速怎么控、卡弹了怎么自动解除、以及热量限制。

---

## 1. 发射机构组成

RoboMaster 的发射机构就两个部件：

```
        弹仓
         │
   ┌─────┴─────┐
   │  拨弹盘    │  ← 旋转式弹仓，每转一格推一颗弹进摩擦轮
   │ (trigger) │     M2006/M3508，位置控制
   └─────┬─────┘
         ▼
  ┌───┐     ┌───┐
  │左 │ 弹→ │右 │   ← 两个摩擦轮高速反向对转，
  │轮 │     │轮 │      夹住弹丸把它甩出去
  └───┘     └───┘      M3508，速度控制
         │
         ▼  弹丸射出
```

- **摩擦轮（friction wheel）**：左右两个高速旋转的轮子，弹丸从中间穿过时被摩擦力加速甩出。**摩擦轮转速决定子弹初速**——这正是 [gimbal](./gimbal.md) 弹道解算要的那个"initial_vel"。用 M3508，做**速度控制**。
- **拨弹盘（trigger）**：一个旋转的弹仓盘，转一格就推一颗弹进摩擦轮。它决定**发射节奏和单发精度**。用 M2006/M3508，做**位置控制**——每次精确转过"一格"的角度。

这里回收 [hardware](./hardware.md) 的两个伏笔：

1. 拨弹盘用**增量式编码器**、又需要知道绝对角度（"现在停在哪一格"），所以**必须标定**。标定让它上电后零点对齐弹位，否则每次发射角度都错，导致双发或空发。
2. 摩擦轮虽然也用增量式电机，但只做速度控制、不在乎绝对位置，所以**不标定**。

控制器声明的接口很简单——只要力矩输出：

```cpp
class Controller : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,   // 向摩擦轮/拨盘发力矩
    rm_control::RobotStateInterface>            // 仅声明（保证初始化顺序），实际不用
```

> 注：`RobotStateInterface` 这里只是**声明**、并不真正使用。这是框架的一个惯例——运动控制类控制器都把它列为基础依赖，好让 `robot_state_controller` 保证先于它们初始化（见 [transform](./transform.md)）。发射控制器自己不查 TF。

三个电机都用力矩接口，但挂不同的子控制器：摩擦轮挂速度 PID（`JointVelocityController`，速度→力矩），拨盘挂位置 PID（`JointPositionController`，角度→力矩）。这又是 [hardware](./hardware.md) 说的"rm-controls 统一用力矩模式，速度/位置环在上位机软件实现"的体现：

```cpp
std::vector<std::vector<effort_controllers::JointVelocityController*>> ctrls_friction_;  // 摩擦轮：速度 PID（二维，可多组）
effort_controllers::JointPositionController                           ctrl_trigger_;    // 拨盘：位置 PID
```

`ctrls_friction_` 是二维的，因为可以有多组摩擦轮（左组/右组、每组前后轮）。控制器通过子控制器的 `joint_` 成员读反馈做判断：`ctrl_trigger_.joint_.getPosition()` 用于对齐弹位、`getVelocity()/getEffort()` 用于卡弹检测、`ctrls_friction_[i][j]->joint_.getVelocity()` 用于就绪判断和射出检测。

---

## 2. 发射状态机

发射控制器的核心是一个四状态状态机。为什么用状态机？因为"打一发子弹"不是一个瞬间动作，而是一串有先后、有条件的阶段：先让摩擦轮转起来、对齐拨盘、再推弹、万一卡住还要解卡。

```
  ┌────┐
  │STOP│  初始/停止/安全：摩擦轮停转，拨盘保持不动
  └─┬──┘
    │ 收到 ShootCmd(mode=READY)
    ▼
 ┌─────┐
 │READY│  摩擦轮加速到目标转速，拨盘归零对齐到最近弹位
 └──┬──┘
    │ mode=PUSH
    ▼
 ┌────┐   检测到卡弹（力矩大 + 速度低，持续 block_duration）
 │PUSH│ ──────────────────────────────────────► ┌─────┐
 └────┘   推弹发射                                │BLOCK│  拨盘反转 anti_block_angle 解卡
    ▲                                            └──┬──┘
    │  解卡成功（到位）或超时（block_overtime）        │
    └────────────────────────────────────────────────┘
```

### 2.1 STOP —— 安全停止

`update()` 每帧就是按当前状态分派到对应处理函数——这就是上面那张状态图在代码里的样子：

```cpp
enum { STOP, READY, PUSH, BLOCK };

void Controller::update(...) {
  switch (state_) {
    case STOP:  stop(time, period);  break;   // 摩擦轮停、拨盘锁位
    case READY: ready(period);       break;   // normalize() 对齐 + 摩擦轮加速
    case PUSH:  push(time, period);  break;   // 判就绪 → 推弹 + 卡弹检测
    case BLOCK: block(time, period); break;   // 反转解卡
  }
  setSpeed(cmd_);          // 摩擦轮目标速度 + 防堵转（§3）
  judgeBulletShoot(...);   // 靠转速骤降推断是否射出（§2.4）
}
```

每个状态函数用 `state_changed_` 标志区分"进入时执行一次"和"每帧执行"。下面逐个看。

进入 STOP 时把摩擦轮转速设 0，拨盘"保持当前位置"（不是断电松开，而是位置 PID 锁住不动）：

```cpp
ctrl_friction->setCommand(0.);                              // 摩擦轮停
ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition()); // 拨盘锁在原地
```

回想 [hardware](./hardware.md) 讲的标定编排：拨盘标定完成后 `shooter_controller` 被恢复，此刻它就处于 STOP——摩擦轮和拨盘都不会乱动，等收到 `ShootCmd` 才进入 READY。

### 2.2 READY —— 备弹

进入时执行一次 `normalize()`：把拨盘对齐到**最近的标准弹位**。

这里有个 [hardware](./hardware.md) 埋的细节——标定把拨盘限位设成了零点，但限位处不一定正好卡着一颗待发的弹。`normalize()` 就是"标定后的位置对齐":以"每格角度 = 2π / 每转弹数"为刻度，把拨盘 snap 到最近的整格：

```cpp
double push_angle = 2*M_PI / push_per_rotation_;      // 每颗弹占的角度
ctrl_trigger_.setCommand(push_angle * std::floor(position / push_angle));  // 对齐到最近弹位
```

同时摩擦轮开始朝目标转速加速。

### 2.3 PUSH —— 发射

PUSH 是核心，每一发都要过两道关：

1. **摩擦轮就绪判断**：只有当摩擦轮实际转速达到目标的一定比例（`push_wheel_speed_threshold`，默认 0.9）时才允许发射。否则轮子没转够，弹丸初速不足、打不远也不准。
2. **节奏控制**：距上一发的间隔要达到 `1/hz`（射频），才推下一颗。

条件满足就让拨盘转过一格（位置控制）把弹推出。这里还分两种模式：

- **低频模式**（hz < `freq_threshold`）：每发之间等拨盘回到标准位置再发下一颗，保证**单发精度**。
- **高频模式**（hz ≥ `freq_threshold`）：给拨盘速度前馈，让它**匀速连续旋转**，实现连发，不再逐发对齐。

### 2.4 判断弹丸是否真的射出去了

没有光电传感器，怎么知道一颗弹真打出去了？靠**摩擦轮转速的骤降**推断：弹丸穿过摩擦轮时会把轮子的转速"拽"下去一下，检测到这个急剧下降就判定"射出一发":

```cpp
double friction_change_speed = |friction_speed| - last_friction_speed;
if (friction_change_speed < -wheel_speed_drop_threshold && !has_shoot_ && ...)
  has_shoot_ = true;   // 发布到 /local_heat_state/shooter_state
```

这个"射出一发"的信号会上报，用于本地热量估算（见第 5 节）。

> **指令与状态的进出口**：决策层的 `ShooterCommandSender`（[manual](./manual.md)）把 `ShootCmd`（`mode`/`wheel_speed`/`hz`）发到 `/command`，控制器经 `cmd_rt_buffer_`（`RealtimeBuffer`，非实时写、实时读）安全取用——又一次 [transform](./transform.md) 提到的跨线程模式。控制器则发布 `state`（当前 STOP/READY/PUSH/BLOCK）和 `/local_heat_state/shooter_state`（射出信号）两个话题。

---

## 3. 摩擦轮转速控制

摩擦轮走速度 PID，目标转速由 `ShootCmd.wheel_speed` 给定——它直接决定子弹初速。控制器还处理一种特殊情况：**摩擦轮自己被卡住**（比如卡了半颗弹）。

摩擦轮防堵转独立于拨盘卡弹检测：检测到摩擦轮"力矩大且速度低"，就进入防堵转模式，按占空比交替输出高速/零速，试图**抖开**卡住的弹：

```cpp
if (joint_.getEffort() >= friction_block_effort_ && joint_.getVelocity() <= friction_block_vel_)
  friction_wheel_block = true;

if (friction_wheel_block) {                       // 抖动解卡
  double command = (count <= duty_cycle*1000) ? anti_friction_block_vel_ : 0.;
  ctrl_friction->setCommand(command);
} else {
  ctrl_friction->setCommand(direction * (wheel_speed + extra + offset));  // 正常目标速度
}
```

---

## 4. 卡弹检测与自动解除

拨盘卡弹（一颗弹卡在拨盘和摩擦轮之间）是发射机构最常见的故障。状态机用 BLOCK 状态自动处理。

**怎么检测卡弹？** 在 PUSH 状态里看拨盘：如果**力矩很大**（`> block_effort`，使劲在推）但**速度很低**（`< block_speed`，却推不动），并且这个情况持续了 `block_duration`，就判定卡弹，转入 BLOCK。这和 [hardware](./hardware.md) 撞限位标定里"力矩大速度低=堵转"的判据是同一个思路。

**怎么解卡？** 进入 BLOCK 后让拨盘**反转**一个角度 `anti_block_angle`，把卡住的弹倒退出来：

```cpp
// 进入 BLOCK
ctrl_trigger_.setCommand(current_position + anti_block_angle);   // 反转
// 反转到位 或 超时(block_overtime)
if (到位 || 超时) { normalize(); state = PUSH; }   // 重新对齐，回 PUSH 继续发
```

反转到位（弹倒出来了）或者超过 `block_overtime`（强制放弃，防止卡死在 BLOCK）后，重新 `normalize()` 对齐，回到 PUSH 继续发射。整个卡弹→解卡对操作手是无感的，机器不会因为一次卡弹就哑火。

---

## 5. 热量限制

> 裁判系统给每个枪口设了**热量上限**。每发射一颗弹增加固定热量，热量随时间冷却。热量超上限会扣血，所以不能无脑连发。

关键区分：**热量限制不在发射控制器里，而在决策层。** 发射控制器只管"收到指令就按 hz 发弹",到底允不允许发、以多高的频率发，是决策层根据裁判系统的实时热量数据算好、通过 `ShootCmd.hz` 下发的。

具体来说，决策层的 `ShooterCommandSender` 里有个 `heat_limit_`，它在发指令前根据当前热量和上限，把射频钳在安全范围内——热量快满就降频甚至停发，冷却下来再放开。配置在 `rm_manual` 里：

```yaml
# rm_manual/<robot>.yaml
shooter:
  heat_limit:
    low_shoot_frequency:  1    # 低档射频（Hz）
    high_shoot_frequency: 3    # 高档射频
    burst_shoot_frequency: 6   # 爆发射频
    type: "ID1_42MM"           # 弹丸类型（决定单发热量、上限）
```

这是分层设计的又一个例子：发射控制器（控制层，1kHz）只专注"怎么把弹稳定打出去",而"该不该打、打多快"这种要读裁判系统、做策略的事，交给决策层（100Hz）。热量限制的决策层细节见 [manual](./manual.md)。

> 顺带一提，还有个"邪修"的**摩擦轮转速自动校准**：每次发射后裁判系统会返回实际弹速，决策层拿它和期望弹速比较，慢慢微调摩擦轮转速补偿磨损。它也藏在 `ShooterCommandSender` 里、默认关闭，同样属于决策层而非发射控制器。

---

## 6. 发射运动学：拨盘角度与弹速

发射机构有两条简单但关键的运动学关系，一条管"发多少、多快",一条管"多快出膛"。

### 6.1 拨盘：转角 ↔ 弹数

拨盘转一整圈推出 `push_per_rotation` 颗弹，记这个数为 $N$。于是**每颗弹对应的拨盘转角**是：

$$\Delta\varphi = \frac{2\pi}{N}$$

- **单发**（低频）：拨盘精确转过 $\Delta\varphi$（位置控制），推一颗、对齐、停。
- **连发**（高频，射频 $f$ 赫兹）：拨盘匀速旋转，角速度为

$$\omega_{\text{trigger}} = f \cdot \Delta\varphi = \frac{2\pi f}{N}$$

这就是为什么 `push_per_rotation` 必须和实际弹仓格数**严格一致**——填错了，每发的转角就错，直接双发或空发（配合 [hardware](./hardware.md) 的拨盘 offset 一起决定弹位对齐）。§2.2 的 `normalize()` 用的正是这个 $\Delta\varphi$ 做刻度对齐。

### 6.2 摩擦轮：转速 ↔ 弹速

两个摩擦轮反向对转，弹丸从中间被夹住加速，**出膛速度约等于摩擦轮的轮缘线速度**：

$$v_{\text{bullet}} \approx \omega_{\text{wheel}} \cdot r_{\text{wheel}}$$

所以设定摩擦轮目标转速 $\omega_{\text{wheel}}$，就等于设定了子弹初速 $v_{\text{bullet}}$——而这个初速正是 [gimbal](./gimbal.md) §4.2 弹道解算要的 `initial_vel`。发射和云台就是通过这个量耦合的。

实际会有滑移、弹丸压缩、摩擦轮磨损，真实弹速略低于理论线速度，且随磨损漂移。所以有一套**摩擦轮转速自动校准**：用裁判系统返回的实测弹速做闭环，慢慢微调 $\omega_{\text{wheel}}$ 补偿误差（[hardware](./hardware.md) 里提到的"邪修法"，默认关闭）。目标弹速到轮速的映射是一张查表，配置在 `bullet_solver` 的 `speed_*_per_speed` / `wheel_speed_*` 里。

一句话：**拨盘运动学定"发多少、多快",摩擦轮运动学定"多快出膛"。**

---

## 7. 标定

发射机构里**只有拨盘（trigger）需要标定**。回到 [hardware](./hardware.md) 的判据：拨盘用 M2006 / M3508（**增量编码器**），且需要知道**绝对角度**来对齐弹位——两个条件都满足，所以要标。摩擦轮虽然也是增量电机，但只控速、不关心绝对位置，**不标定**。

- **方法**：撞限位（Mechanical，[hardware](./hardware.md) §4.5）。拨盘以 `search_velocity`（约 4 rad/s）正转，撞到机械限位后速度骤降到阈值以下，确认后设 offset、标记完成。
- **标定后对齐**：撞限位设的零点不一定正好卡着一颗待发弹，所以紧接着 `normalize()` 把拨盘 snap 到最近弹位（§2.2）。
- **编排**：`shooter_calibration` 流水线。标定时停 `shooter_controller`（让出 `trigger_joint`），标完恢复——此时 shooter 处于 STOP（§2.1）。因为 joint 集合不相交，标拨盘时底盘、云台照常动（[hardware](./hardware.md) §4.7）。
- **触发**：发射电源 ON、比赛自检、比赛开始都会 `reset()` 重标——多次重标以保证"战斗零点"最新（[manual](./manual.md) §4）。

配置示例：

```yaml
# rm_manual：标定流水线
shooter_calibration:
  - start_controllers: [controllers/trigger_calibration_controller]
    stop_controllers:  [controllers/shooter_controller]
    services_name: [/controllers/trigger_calibration_controller/is_calibrated]
```

```yaml
# rm_controllers：标定控制器本体
trigger_calibration_controller:
  type: rm_calibration_controllers/MechanicalCalibrationController
  actuator: [ trigger_joint_motor ]
  velocity:
    search_velocity: 4.0      # 搜索速度 (rad/s)
    vel_threshold: 0.001      # 撞限位判定的速度阈值
    joint: trigger_joint
    pid: { p: 1.2, i: 0, d: 0.0, ... }
```

---

## 8. 配置项说明

发射控制器配置在 `rm_controllers/<robot>.yaml` 的 `shooter_controller` 下：

```yaml
shooter_controller:
  type: rm_shooter_controllers/Controller
  publish_rate: 50
  friction_left:               # 左摩擦轮：速度 PID
    joint: "left_friction_wheel_joint"
    pid: { p: 0.001, i: 0.01, d: 0.0, ... }
  friction_right:              # 右摩擦轮：速度 PID
    joint: "right_friction_wheel_joint"
    pid: { p: 0.001, i: 0.01, d: 0.0, ... }
  trigger:                     # 拨盘：位置 PID
    joint: "trigger_joint"
    pid: { p: 50.0, i: 0.0, d: 1.5, ... }
  push_per_rotation: 8         # 拨盘每转几颗弹（决定每格角度）
  push_wheel_speed_threshold: 0.90  # 摩擦轮转速就绪阈值（占目标比例）
  block_effort: 0.95           # 卡弹力矩阈值
  block_speed: 0.1             # 卡弹速度阈值
  block_duration: 0.05         # 卡弹确认时长
  block_overtime: 0.5          # 反卡弹超时
  anti_block_angle: 0.2        # 反卡弹反转角度
  freq_threshold: 20.0         # 高/低频模式切换阈值
  wheel_speed_drop_threshold: 50.0  # 判断弹丸射出的转速骤降阈值
```

常见调整（都支持 `dynamic_reconfigure` 在线调）：

- **双发/空发** → 不在这里调，去 [hardware](./hardware.md) 说的 URDF 拨盘 `offset`，或检查标定；`push_per_rotation` 要和实际弹仓格数一致
- **连发不顺 / 卡弹频繁** → 调 `block_effort` / `block_speed` / `anti_block_angle`
- **摩擦轮没转够就发** → 调 `push_wheel_speed_threshold`
- **子弹初速（打不远/超速）** → 调摩擦轮目标转速，与 [gimbal](./gimbal.md) 弹道的 `initial_vel` 一起对
- **射频/热量** → 在 `rm_manual` 的 `shooter.heat_limit`，见 [manual](./manual.md)

> **变体：DSHOT 版**。仓库里还有个 `rm_dshot_shooter_controllers`，状态机、卡弹检测、发射检测逻辑和标准版完全一样，唯一区别是摩擦轮改用 `VelocityJointInterface` 通过 DSHOT 数字协议直接给电调发速度指令（标准版是力矩接口 + 软件速度 PID），适用于支持 DSHOT 的 BLDC 电调。

---

## 9. 小结

- 发射机构 = **摩擦轮**（左右对转，速度控制，转速定初速，不标定）+ **拨弹盘**（位置控制，每格一弹，增量编码器需标定）。
- **四状态机 STOP → READY → PUSH → BLOCK**：STOP 安全停、READY 备弹并 `normalize()` 对齐弹位、PUSH 判就绪后推弹、BLOCK 反转解卡。
- 摩擦轮就绪（达目标转速的 90%）才允许发射；靠**摩擦轮转速骤降**推断弹丸已射出（无光电传感器）。
- **卡弹检测**用"力矩大 + 速度低"判据，自动反转解卡，对操作手无感。
- **热量限制不在发射控制器**，而在决策层 `ShooterCommandSender` 里按裁判系统热量数据调射频——分层设计：控制层管"怎么打",决策层管"该不该打"。
- **运动学**：拨盘每颗弹转角 $\Delta\varphi=2\pi/N$、连发角速度 $\omega=2\pi f/N$；摩擦轮弹速 $v\approx\omega_{\text{wheel}}\,r_{\text{wheel}}$，即 gimbal 弹道的 `initial_vel`。
- **标定**：只有拨盘要标（增量编码器 + 需绝对弹位），撞限位 Mechanical，标完 `normalize()` 对齐弹位；标定时停 shooter、底盘云台不受影响。

下一站 [manual](./manual.md)：上层的决策层怎么把遥控器输入、控制器切换、标定编排、指令发布串起来——理解了三个下层机构，就能看懂上层怎么编排它们了。
