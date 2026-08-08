# 发射：子弹是怎么打出去的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（拨盘为什么要标定、力矩/速度/位置控制）、[control](./control.md)（PID、模式切换与积分保护）

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

1. 拨弹盘需要知道机械弹位（“现在停在哪一格”）。单圈编码器角并不自动等于机构零点；若系统不能可靠恢复这个对应关系，就必须用 Hall、限位或其他外部参考标定，否则可能双发或空发。
2. 摩擦轮只做速度控制、不在乎机械零点，所以通常不做找零，但仍要检查速度方向、反馈新鲜度和量化误差。

控制器声明的接口很简单——只要力矩输出：

```cpp
class Controller : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,   // 向摩擦轮/拨盘发力矩
    rm_control::RobotStateInterface>            // 仅声明（保证初始化顺序），实际不用
```

> 注：`RobotStateInterface` 这里只是**声明**、并不真正使用。这是框架的一个惯例——运动控制类控制器都把它列为基础依赖，好让 `robot_state_controller` 保证先于它们初始化（见 [transform](./transform.md)）。发射控制器自己不查 TF。

三个电机在控制器侧都用力矩接口，但挂不同的子控制器：摩擦轮挂速度 PID（`JointVelocityController`，速度→力矩），拨盘挂位置 PID（`JointPositionController`，角度→力矩）。这是上位机统一使用关节 effort 抽象的体现；下游电调实际接收电流、电压还是复合命令，仍由硬件驱动和固件模式决定，见 [hardware](./hardware.md) §1.1。

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
 │READY│  摩擦轮加速到目标转速，拨盘按负向供弹方向对齐
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

### 2.1 发射许可总门

STOP/READY/PUSH/BLOCK 描述的是执行流程。下面的许可表达式是**建议增加的安全层**，不是当前 `rm_shooter_controllers/standard.cpp` 已经实现的一个总门：

```cpp
fire_allow = armed
          && command_fresh
          && shooter_power_on
          && trigger_calibrated
          && wheel_ready
          && heat_allow
          && operator_hold
          && (!auto_fire || (vision_fresh && vision_fire));
```

当前实现的边界是：`shooter_controller` 从 `RealtimeBuffer` 读取最新 `ShootCmd`，按 `mode`（并结合 BLOCK/READY 的内部条件）驱动状态机；它会检查摩擦轮转速、拨盘运动和卡弹，但没有 `armed` 字段，也没有依据 `ShootCmd.stamp` 拒绝过期命令。`rm_manual` 的 `ShooterCommandSender::checkError()` 会根据预判射击、云台误差和目标加速度把 PUSH 改回 READY；`HeatLimit` 负责计算 `hz`，这些机制合起来仍不等于上面的完整互锁。

如果要补齐这层，`armed` 应是独立的武装互锁：上电、遥控器重连或控制器重启后，即使射击按键保持按下，也要先观察到一次松开/中位，再接受新的按下边沿。建议在命令超时、掉电、未标定、故障锁定或回到 PASSIVE/IDLE 时撤销它；这属于后续安全设计，不应写成当前功能。

建议的优先级仍应是：**掉线/急停 > 掉电/未标定 > 故障锁定 > 卡弹恢复 > 正常发射**。这是设计约束，不是当前四状态机已经提供的 FAULT 状态。

### 2.2 STOP —— 安全停止

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
  judgeBulletShoot(...);   // 靠转速骤降估计是否射出（§2.5）
}
```

每个状态函数用 `state_changed_` 标志区分"进入时执行一次"和"每帧执行"。下面逐个看。

进入 STOP 时把摩擦轮转速设 0，拨盘"保持当前位置"（不是断电松开，而是位置 PID 锁住不动）：

```cpp
ctrl_friction->setCommand(0.);                              // 摩擦轮停
ctrl_trigger_.setCommand(ctrl_trigger_.joint_.getPosition()); // 拨盘锁在原地
```

回想 [hardware](./hardware.md) 讲的标定编排：拨盘标定完成后 `shooter_controller` 被恢复，此刻它就处于 STOP——摩擦轮和拨盘都不会乱动，等收到 `ShootCmd` 才进入 READY。

### 2.3 READY —— 备弹

进入时执行一次 `normalize()`：把拨盘沿供弹的负方向对齐到一个标准弹位，**不是按角度距离选择最近槽位**。

这里有个 [hardware](./hardware.md) 埋的细节——标定把拨盘限位设成了零点，但限位处不一定正好卡着一颗待发的弹。`normalize()` 以 `push_angle = 2π / push_per_rotation` 为刻度，用 `std::floor()` 和 `exit_push_threshold` 等偏置选择供弹方向前方的整格；它没有计算相邻槽位的绝对距离。

```cpp
double push_angle = 2*M_PI / push_per_rotation_;
ctrl_trigger_.setCommand(
    push_angle * std::floor((position + 0.01 + exit_push_threshold) / push_angle));
// 正常 PUSH 会把目标位置减去 push_angle
```

同时摩擦轮开始朝目标转速加速。

### 2.4 PUSH —— 发射

PUSH 是核心，每一发都要过两道关：

1. **摩擦轮就绪判断**：只有当摩擦轮实际转速达到目标的一定比例（`push_wheel_speed_threshold`，默认 0.9）时才允许发射。否则轮子没转够，弹丸初速不足、打不远也不准。
2. **节奏控制**：距上一发的间隔要达到 `1/hz`（射频），才推下一颗。

条件满足就让拨盘转过一格（位置控制）把弹推出。这里还分两种模式：

- **低频模式**（hz < `freq_threshold`）：每发之间等拨盘回到标准位置再发下一颗，保证**单发精度**。
- **高频模式**（hz ≥ `freq_threshold`）：给拨盘速度前馈，让它**匀速连续旋转**，实现连发，不再逐发对齐。

### 2.5 估计弹丸是否射出

当前标准控制器确实用**摩擦轮转速的骤降**产生一个疑似射出信号，但实现很具体：只取 `ctrls_friction_[0][0]` 这一只轮，先过低通滤波，再计算 `abs(speed)` 的一阶差分和差分的差分。控制器不在 STOP 时，下降阈值和负的二阶变化同时满足且当前未置位，才把 `has_shoot` 置为 true；速度变化转正后才清回 false。

```cpp
lp_filter_->input(ctrls_friction_[0][0]->joint_.getVelocity());
double speed = lp_filter_->output();
double delta = abs(speed) - last_speed;
double delta_delta = delta - last_delta;
if (state_ != STOP && delta < -wheel_speed_drop_threshold &&
    !has_shoot_ && delta_delta < 0)
  has_shoot_ = true;   // 发布到 /local_heat_state/shooter_state
```

这个信号会上报，用于 `HeatLimit` 的本地热量估算（见第 5 节），但它不是弹丸确实离开枪口的事实。当前实现没有检查拨盘是否正在推弹、没有比较左右轮、没有显式 refractory/debounce，也没有在 BLOCK 恢复阶段单独屏蔽。因此下面这些是改进建议，不是现有行为：

- 只在摩擦轮已稳态且拨盘正在推弹时启用检测；
- 对速度变化使用明确的触发/清除迟滞和 debounce；
- 检查左右轮瞬态的一致性；
- 升速阶段和 BLOCK 恢复阶段禁用检测。

裁判系统的射击数据仍是比赛判罚的权威；本地事件用于低延迟预测和故障降级，不能反过来覆盖官方结果。

> **指令与状态的进出口**：决策层的 `ShooterCommandSender`（[manual](./manual.md)）把 `ShootCmd`（`mode`/`wheel_speed`/`hz`）发到 `/command`，控制器经 `cmd_rt_buffer_`（`RealtimeBuffer`，非实时写、实时读）安全取用——又一次 [transform](./transform.md) 提到的跨线程模式。控制器则发布 `state`（当前 STOP/READY/PUSH/BLOCK）和 `/local_heat_state/shooter_state`（射出信号）两个话题。

---

## 3. 摩擦轮转速控制

摩擦轮走速度 PID，目标转速由 `ShootCmd.wheel_speed` 给定——它直接决定子弹初速。控制器还处理一种特殊情况：**摩擦轮自己被卡住**（比如卡了半颗弹）。

摩擦轮防堵转独立于拨盘卡弹检测：检测到摩擦轮"力矩大且速度低"，就进入防堵转模式，按占空比交替输出高速/零速，试图**抖开**卡住的弹：

当前判据带有每个轮子的方向符号：

```cpp
if (wheel_speed_direction * joint_.getVelocity() <= friction_block_vel_ &&
    abs(joint_.getEffort()) >= friction_block_effort_ && cmd.wheel_speed != 0)
  friction_wheel_block = true;

if (friction_wheel_block) {                       // 抖动解卡
  double command = (count <= duty_cycle * 1000) ? anti_friction_block_vel_ : 0.;
  ctrl_friction->setCommand(command);
} else {
  ctrl_friction->setCommand(wheel_speed_direction *
                             (wheel_speed + extra + offset));
}
```
速度反馈先乘 `wheel_speed_direction` 再与阈值比较，力矩使用绝对值，且只有 `cmd.wheel_speed != 0` 才会触发。进入防堵转后，当前代码把同一个正的 `anti_friction_block_vel_` 或 0 发给所有摩擦轮；它没有按轮方向重新乘符号。另一个实现细节是共享的 `friction_wheel_block` 在嵌套循环中会被后一个轮子覆盖，多轮配置若要表达“任一轮卡住”需要额外修正聚合逻辑。

---

## 4. 卡弹检测与自动解除

拨盘卡弹（一颗弹卡在拨盘和摩擦轮之间）是发射机构最常见的故障。状态机用 BLOCK 状态自动处理。

**怎么检测卡弹？** 当前代码只在 PUSH 中已经满足摩擦轮就绪、发射间隔到期的路径上检查拨盘反馈。第一条条件是 `joint_.getEffort() < -block_effort`（注意是**负力矩**，不是绝对值）且 `abs(velocity) < block_speed`；另一条条件是在预期发射间隔之后仍然几乎没有拨盘速度。疑似卡弹持续 `block_duration` 后才进入 BLOCK。代码没有 `effort_clear` 迟滞，也没有独立的 FAULT/重试计数。

**怎么解卡？** 正常 PUSH 的目标位置每发减小一个 `push_angle`，所以进入 BLOCK 时用 `current_position + anti_block_angle` 把目标移向正方向；`anti_block_angle` 为正时，这才是相对于供弹方向的反向动作：

```cpp
// 进入 BLOCK：正常供弹沿负方向，因此加角度是反向
ctrl_trigger_.setCommand(current_position + anti_block_angle);
// 到位或超过 block_overtime 后重新 normalize，再回 PUSH
```

当前路径在反向目标到位或超过 `block_overtime` 后重新 `normalize()`，再回 PUSH 尝试继续发射。这能处理偶发卡滞，但“超时”不代表卡弹已经解除。以下是建议补充的工程安全措施，当前代码没有实现：

- 配置化的反拨方向、角度、速度和到位容差；
- 单次恢复后的冷却/观察时间；
- 有限重试次数，连续失败后锁定 STOP/FAULT；
- 故障状态和重试次数上报给操作手，必须先松开射击并显式复位才能再次武装。

软件反拨只能让供弹链路暂时松弛，不能修复尖锐棱角、不平滑弯道、错误弹位或缺少单发限位。卡弹频繁时应先修机械；无限自动重试会持续发热、磨损弹丸，甚至把可恢复故障扩大成硬件损坏。

---

## 5. 热量限制

> 裁判系统给每个枪口设了**热量上限**。每发射一颗弹增加固定热量，热量随时间冷却。热量超上限会扣血，所以不能无脑连发。

关键区分：**热量限制不在发射控制器里，而在决策层。** 发射控制器只管"收到指令就按 hz 发弹"；到底允不允许发、以多高的频率发，由决策层结合裁判状态与配置选择的热量来源计算，再通过 `ShootCmd.hz` 下发。

具体来说，决策层的 `ShooterCommandSender` 持有 `HeatLimit`。它接收裁判的机器人状态（热量上限、冷却速度）、功率热量数据、裁判在线状态和发射控制器发布的 `LocalHeatState`，在 `sendCommand()` 时把计算结果写入 `ShootCmd.hz`。这是对**命令频率**的约束，不是发射控制器内部的总许可门。

当前实现有几个容易被忽略的边界：`use_local_heat` 打开时优先使用本地累计热量；`BURST` 模式直接返回配置的爆发射频；裁判离线时当前代码返回固定的 `5.0` Hz，而不是自动归零。因此这些行为必须结合车辆配置和现场安全策略验证。

```yaml
# rm_manual/<robot>.yaml
shooter:
  heat_limit:
    low_shoot_frequency:  1    # 低档射频（Hz）
    high_shoot_frequency: 3    # 高档射频
    burst_shoot_frequency: 6   # 爆发射频
    minimal_shoot_frequency: 1 # 最低保护射频
    safe_shoot_frequency: 1    # 未匹配模式时的保护射频
    heat_coeff: 1              # 接近上限时的线性降频系数
    local_heat_protect_threshold: 0
    use_local_heat: true
    type: "ID1_42MM"           # 弹丸类型（决定单发热量、上限）
```

这是分层设计的又一个例子：发射控制器（控制层，1kHz）只专注"怎么把弹稳定打出去",而"该不该打、打多快"这种要读裁判系统、做策略的事，交给决策层（100Hz）。热量限制的决策层细节见 [manual](./manual.md)。

### 5.1 热量预测模型

当前代码里的本地模型是一个更简单的计数器：

- `HeatLimit::heatCB()` 只在 `has_shoot` 从 false 变 true 时增加一次 `bullet_heat_`；`ID1_42MM` 使用 100，其它已识别类型使用 10。
- 一个 0.1 秒 ROS 定时器每次减去 `shooter_cooling_rate * 0.1`；累计值降到 0 后清零本地发射计数。
- `getShootFrequency()` 根据剩余热量与 `bullet_heat_` 的关系返回 0、冷却速率对应的最低频率、线性降频值或目标频率。

这个模型没有把每个事件的时间戳与官方热量样本做逐事件对齐，也没有显式为未检测到的已下发弹保留不确定度。它是当前实现的近似，不能包装成裁判热量的精确复制。

### 5.2 裁判数据与本地事件怎样合并

如果要把本地事件和官方数据做得更稳健，下面是一套**建议设计**；当前 `HeatLimit` 尚未实现这套带时间戳的融合：

1. 每个裁判热量样本带时间戳和新鲜度；
2. 收到样本后先按冷却模型传播到当前时刻；
3. 只叠加样本时间之后的本地射出事件，避免重复计热；
4. 官方数据陈旧或掉线时使用保守上界并降频/停火；
5. 记录官方值与本地预测的残差，定位漏检、参数错误和时间戳错位。

因此“无裁判热量”只是故障降级，不是更可信的替代测量；任何本地策略都必须给裁判判定留裕量。

可选的 `auto_wheel_speed` 是另一条已经存在的路径：`ShooterCommandSender::updateShootData()` 收到裁判返回的实际弹速后，若偏差超过 `speed_oscillation`，就按固定步长调整 `total_extra_wheel_speed_`。它是可选的摩擦轮速度补偿（多数当前车辆配置为 false），不是热量模型，也不应再称作“邪修法”。

---

## 6. 发射运动学：拨盘角度与弹速

发射机构有两条简单但关键的运动学关系，一条管"发多少、多快",一条管"多快出膛"。

### 6.1 拨盘：转角 ↔ 弹数

拨盘转一整圈推出 `push_per_rotation` 颗弹，记这个数为 $N$。于是**每颗弹对应的拨盘转角**是：

$$\Delta\varphi = \frac{2\pi}{N}$$

- **单发**（低频）：拨盘精确转过 $\Delta\varphi$（位置控制），推一颗、对齐、停。
- **连发**（高频，射频 $f$ 赫兹）：拨盘匀速旋转，角速度为

$$\omega_{\text{trigger}} = f \cdot \Delta\varphi = \frac{2\pi f}{N}$$

这就是为什么 `push_per_rotation` 必须和实际弹仓格数**严格一致**——填错了，每发的转角就错，直接双发或空发（配合 [hardware](./hardware.md) 的拨盘 offset 一起决定弹位对齐）。§2.3 的 `normalize()` 用的正是这个 $\Delta\varphi$ 做刻度对齐。

### 6.2 摩擦轮：转速 ↔ 弹速

两个摩擦轮反向对转，弹丸从中间被夹住加速，**出膛速度约等于摩擦轮的轮缘线速度**：

$$v_{\text{bullet}} \approx \omega_{\text{wheel}} \cdot r_{\text{wheel}}$$

所以设定摩擦轮目标转速 $\omega_{\text{wheel}}$，会影响实际子弹初速。`ShooterCommandSender` 将配置的目标速度和轮速传给 `GimbalCmd.bullet_speed`，跟踪用的 `BulletSolver` 通过这个量耦合；[gimbal](./gimbal.md) §4.2 的 `BallisticSolver` 则仍使用自己的 `initial_vel` 配置，当前不会自动读取该字段。

实际会有滑移、弹丸压缩、摩擦轮磨损，真实弹速略低于理论线速度，且随磨损漂移。当前可选的 `auto_wheel_speed` 反馈在 `ShooterCommandSender` 中读取裁判返回的实际弹速，并按配置的 `speed_oscillation` 调整额外轮速；它默认是否开启由各车的 `rm_manual/<robot>.yaml` 决定。目标速度到轮速的映射参数 `speed_*_per_speed`、`wheel_speed_*` 也在 `rm_manual` 的 `shooter` 节点，不在 `bullet_solver`。

一句话：**拨盘运动学定"发多少、多快",摩擦轮运动学定"多快出膛"。**

---

## 7. 标定

当前常见配置里，发射机构需要建立机械零位的是拨盘（`trigger`）；摩擦轮只做速度控制，通常不参与标定。是否标定仍以硬件配置中的 `need_calibration` 和具体机构为准，不能只从电机型号推断。

- **方法**：撞限位（Mechanical，[hardware](./hardware.md) §4.5）。拨盘以 `search_velocity`（约 4 rad/s）正转，撞到机械限位后速度骤降到阈值以下，确认后设 offset、标记完成。
- **标定后对齐**：撞限位设的零点不一定正好卡着一颗待发弹，所以紧接着 `normalize()` 沿负向供弹方向对齐到标准弹位（§2.3），不是选择几何上最近的槽位。
- **编排**：`shooter_calibration` 流水线。标定时停 `shooter_controller`（让出 `trigger_joint`），标完恢复——此时 shooter 处于 STOP（§2.2）。因为 joint 集合不相交，标拨盘时底盘、云台照常动（[hardware](./hardware.md) §4.7）。
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
  # 注意：standard.cpp 读取的就是 push_wheel_speed_threshold。
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
- **摩擦轮没转够就发** → 先确认配置键名是 `push_wheel_speed_threshold`。当前多个 `rm_config/config/rm_controllers/*.yaml` 仍写 `push_qd_threshold`，而当前 `standard.cpp` 不读取这个旧名字；不迁移时会落到代码默认值 0。
- **子弹初速（打不远/超速）** → 调摩擦轮目标转速，与 [gimbal](./gimbal.md) 弹道的 `initial_vel` 一起对
- **射频/热量** → 在 `rm_manual` 的 `shooter.heat_limit`，见 [manual](./manual.md)

> **变体：DSHOT 版**。仓库里还有个 `rm_dshot_shooter_controllers`，状态机、卡弹检测、发射检测逻辑和标准版完全一样，唯一区别是摩擦轮改用 `VelocityJointInterface` 通过 DSHOT 数字协议直接给电调发速度指令（标准版是力矩接口 + 软件速度 PID），适用于支持 DSHOT 的 BLDC 电调。

---

## 9. 小结

- 发射机构通常是**摩擦轮**（速度控制）+ **拨弹盘**（位置控制）；是否标定由具体机构与硬件配置的 `need_calibration` 决定。
- **四状态机 STOP → READY → PUSH → BLOCK**：STOP 安全停、READY 按负向供弹方向 `normalize()`、PUSH 判就绪后推弹、BLOCK 沿正方向尝试解卡。
- 当前代码没有独立的 `armed`/命令新鲜度总门；这些以及掉电、未标定、连续卡弹锁定属于需要在上层补齐的安全设计。
- 摩擦轮就绪判据使用方向归一化后的速度；注意当前代码读取 `push_wheel_speed_threshold`，而若配置仍使用 `push_qd_threshold`，该参数会被忽略。射出检测只看第一组第一只轮的滤波速度骤降。
- **卡弹检测**当前使用负拨盘力矩或预期发射后仍低速的条件，并持续 `block_duration`；BLOCK 用 `+ anti_block_angle` 沿正方向退回，然后 `normalize()` 再回 PUSH。
- **热量限制**由决策层 `HeatLimit` 计算 `ShootCmd.hz`，用本地 `has_shoot` 上升沿和 0.1 秒冷却计时器近似热量；带时间戳的官方/本地融合是后续改进，不是当前实现。
- **运动学**：拨盘每颗弹转角 $\Delta\varphi=2\pi/N$、连发角速度 $\omega=2\pi f/N$；摩擦轮速度会影响实际弹速，并通过 `GimbalCmd.bullet_speed` 供 `BulletSolver` 使用。
- **标定**：常见配置只标拨盘，但最终以 `need_calibration` 和机构参考为准；标完 `normalize()` 沿供弹方向对齐，标定编排时停用占用同一 joint 的主控制器。

下一站 [manual](./manual.md)：上层的决策层怎么把遥控器输入、控制器切换、标定编排、指令发布串起来——理解了三个下层机构，就能看懂上层怎么编排它们了。
