# 底盘：车是怎么动起来的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（关节与力矩）、[transform](./transform.md)（TF，FOLLOW 模式和里程计要用）

底盘控制器（`chassis_controller`）claim 的是底盘那几个轮子 joint，负责把操作手的"我要往那边走"翻译成每个轮子该出多大力。这篇讲清楚：底盘有哪几种轮系、每种怎么把底盘速度解算成轮速、比赛的功率限制怎么分配、小陀螺（FOLLOW）怎么让底盘自动跟着云台转、以及里程计。

一个贯穿全文的关键点，先说在前面：**底盘控制器最终输出的是力矩（N·m），不是速度。** 为什么这么设计，得从比赛规则讲起。

---

## 1. 为什么最终输出是力矩

底盘控制器的形状不是从"我要写个运动学解算"长出来的，而是被**比赛规则**逼出来的。最关键的一条规则是**功率限制**：

> 裁判系统实时测量底盘功率（各轮电机功率之和）。超功率会扣血，严重超功率直接判负。

这条规则直接决定了底盘控制的最终抽象必须是**力矩**。看两种选择的对比：

```
如果最终抽象是速度：                     如果最终抽象是力矩：
控制器算出"左轮要 100 rpm"              控制器算出"左轮要 0.5 N·m"
   │ 功率超了，想降功率                     │ 功率超了，想降功率
   ▼                                       ▼
只能降速度目标，但速度环 PID 为了追      直接把力矩按比例缩小——功率和力矩
上目标又会自动加大力矩，功率反而          近似线性，缩放力矩 ≈ 缩放功率，
可能不降反升                             干净可控
```

所以底盘控制器内部虽然一路都在算"速度",但**最后一步一定落到每个轮子的力矩**上，功率限制器才能直接缩放它。这是理解整个底盘控制器的主线。

运动学解算的输出其实是各轮的**目标速度**，每个轮子内部再挂一个速度 PID（`JointVelocityController`）把速度误差转成力矩。完整抽象链是：

```
底盘速度 [vx, vy, wz]
   ↓ 逆运动学（各轮系子类实现）
各轮目标速度 [ω₁ ω₂ ω₃ ω₄]
   ↓ JointVelocityController（速度 PID）
各轮力矩 [τ₁ τ₂ τ₃ τ₄]
   ↓ powerLimit()（统一缩放）
限幅后力矩 [τ₁' τ₂' τ₃' τ₄']
   ↓ joint.setCommand(effort)   → 硬件
```

---

## 2. 底盘有哪些轮系

不同机器人用不同的轮系，运动学解算方式也不同。所有轮系都继承自一个模板基类 `ChassisBase<T...>`（模板参数就是它 claim 的硬件接口，对应 overview 接口矩阵）。基类把公共逻辑一次写全，在 `update()` 里按固定顺序调用：

| 基类职责 | 方法 |
| --- | --- |
| 断连保护 | `update()` 里检查 `timeout_`，超时速度归零 |
| 加减速缓冲 | `RampFilter` 平滑速度指令 |
| 模式分派 | `raw()` / `follow()` / `twist()`（§3） |
| 坐标变换 | `tfVelToBase()`（用 [transform](./transform.md) 的 TF） |
| 功率限制 | `powerLimit()`（§4） |
| 里程计 | `updateOdom()`（§5，发 `odom→base_link` TF） |

它只把"底盘速度 ↔ 轮速"这一步留给子类，是两个纯虚函数：

```cpp
virtual void moveJoint(...) = 0;              // 逆解：底盘速度 → 各轮目标速度
virtual geometry_msgs::Twist odometry() = 0;  // 正解：各轮实速 → 底盘速度（里程计用）
```

注意 `moveJoint()` 设的是各轮**目标速度**、不是力矩——力矩由每个轮子内部的 `JointVelocityController`（速度 PID）随后算出，再统一过 `powerLimit()`（§1 那条抽象链）。各子类挂在这棵树上：

```
ChassisBase<T...>                           公共骨架 + 两个纯虚函数
  ├── OmniController                         4×3 伪逆矩阵（chassis2joints_）
  │     └── ActiveSuspensionController       继承 Omni，+ 悬挂腿位置 PID
  ├── SwerveController                       逐轮几何解算，每轮一个 Module
  └── BalanceController                      倒立摆 + IMU，不用运动学矩阵
```

**新增一种轮系 = 继承 `ChassisBase` + 实现那两个纯虚函数**，功率、里程计、模式、断连保护全部白拿——模板方法模式最干净的例子。下面逐个看子类怎么实现这两个函数。

### 2.1 麦克纳姆轮 / 全向底盘（OmniController）

**最常用**，步兵、英雄、哨兵基本都是它。麦克纳姆轮的轮缘上装了一圈斜 45° 的小滚轮——轮子主动转动的同时，滚轮能让它侧向自由滑动。四个这样的轮子组合起来，就能实现平面内三个自由度：前后（vx）、左右（vy）、自转（wz）。

它的运动学是一个 4×3 矩阵：给定底盘速度 $[v_x, v_y, w_z]$，每个轮子的目标转速由轮子的安装位置、滚轮角度、轮半径决定：

$$\omega_i = \frac{-\sin\alpha_i\, v_x + \cos\alpha_i\, v_y + l_i\cos\delta_i\, w_z}{r}$$

其中 $\alpha$ 是滚轮轴线夹角（麦轮通常 $\pm 45°$），$l$ 是轮心到底盘中心距离，$\delta$ 是轮心方向角，$r$ 是轮半径。

代码里把这个矩阵叫 `chassis2joints_`，在 `init()` 时根据 YAML 里每个轮子的配置构建。解算时用**伪逆**：

```cpp
Eigen::Vector4d joint_vel = chassis2joints_.pseudoInverse() * vel_cmd_eigen;
```

**为什么用伪逆？** 4 个轮子控 3 个自由度，是"冗余驱动"——方程比未知数多。伪逆给出最小二乘解，直观理解就是"在能实现目标运动的所有轮速组合里，挑各轮转速平方和最小的那组",最省力也最平顺。

> 严格说麦克纳姆轮和"全向轮（omni wheel）"是两种不同的轮子，但在 rm-controls 里它们共用 `OmniController` 这套矩阵解算框架——区别只是配置里滚轮角度等参数不同。所以代码里的"Omni"泛指这类靠一个 4×3 矩阵解算的全向底盘。

### 2.2 舵轮底盘（SwerveController）

舵轮的每个轮子有**两个独立电机**：一个控制**转向角**（这个轮朝哪个方向），一个控制**驱动速度**（这个轮转多快）。

解算是逐个轮子的几何计算，不用大矩阵：

```
对每个轮子：
① 算轮心的目标速度矢量  V = V_chassis + ω_chassis × R_轮心
② 转向角 = atan2(V.y, V.x)   → 转向电机走位置 PID
③ 驱动轮速 = |V| / 轮半径     → 驱动电机走速度 PID
```

优点是能做到真正的**零半径原地旋转**（每个轮子先转到切向再驱动）；缺点是一个轮子要两个电机，成本高、机械复杂。代码里每个舵轮打包成一个 `Module` 结构体——把"转向 PID + 驱动 PID + 几何参数"绑在一起：

```cpp
struct Module {
  Vec2<double> position_;                // 轮子安装位置
  double pivot_offset_, wheel_radius_;
  JointPositionController* ctrl_pivot_;  // 转向：位置 PID
  JointVelocityController* ctrl_wheel_;  // 驱动：速度 PID
};
```

### 2.3 主动悬挂底盘（ActiveSuspensionController）

**继承自 OmniController**——先照常算四个全向轮，再额外控制悬挂腿的高度。悬挂腿用位置 PID：

```cpp
void moveJoint(...) override {
  OmniController::moveJoint();       // 先算轮子（复用父类）
  for (auto& leg : active_suspension_joints_)
    leg->setCommand(target_pos_);    // 再控悬挂腿高度（位置 PID）
}
```

悬挂有 DOWN（低重心行驶）/ MID / UP（抬高过障）三档，切换时叠一个指数衰减的前馈力补偿摩擦和弹簧力，让升降更平滑。英雄的悬挂版用它。

### 2.4 两轮平衡底盘（BalanceController）

两轮平衡车没有运动学矩阵可言，本质是一个**倒立摆控制器**：读 IMU 的倾角，用 PID 算出为了维持直立轮子需要的加速度，再叠上操作手的速度指令：

```cpp
double balance_acc = balance_pid_.computeCommand(imu_pitch_, period);
double wheel_target = balance_acc + vel_cmd_.x;  // 平衡 + 操作手意图
```

轮子加减速产生的反作用力把车体撑直。

### 2.5 轮系对比

| | Omni（麦轮/全向） | Swerve（舵轮） | ActiveSuspension | Balance（平衡） |
| --- | --- | --- | --- | --- |
| 轮子数 | 4 | 4（8 电机） | 4 + 悬挂腿 | 2 |
| 自由度 | 3（vx vy wz） | 3（vx vy wz） | 3 + 悬挂高度 | 1（vx）+ 平衡 |
| 解算 | 4×3 伪逆矩阵 | 逐轮几何解算 | Omni + 位置 PID | 倒立摆 + IMU |
| 原地旋转 | 近似 | 精确（零半径） | 近似 | 不可 |
| 机械复杂度 | 低 | 高 | 中 | 低 |
| 典型机器人 | 步兵/英雄/哨兵 | 步兵（舵轮版） | 英雄（悬挂版） | 平衡车 |

---

### 2.6 运动学总览：逆解与正解

不管什么轮系，底盘运动学都是在两个空间之间换算：**底盘空间**（整车的 vx, vy, wz）和**关节空间**（各轮转速）。方向不同，用途不同：

| | 逆解（逆运动学） | 正解（正运动学） |
| --- | --- | --- |
| 方向 | 底盘速度 → 各轮转速 | 各轮转速 → 底盘速度 |
| 代码 | `moveJoint()`（纯虚，各子类实现） | `odometry()`（纯虚，各子类实现） |
| 用途 | **控制**——"我要车这么动，各轮该转多快" | **里程计**——"轮子转成这样，车实际怎么动了" |
| 何时算 | 每拍下行 | 每拍上行（`updateOdom`，见 §5） |

两者互为逆运算。以最常用的 Omni 为例：`chassis2joints_` 这个 4×3 矩阵描述"底盘速度→轮速",逆解时对它求**伪逆**（4 轮控 3 自由度、超定，取最小二乘解）；正解时直接用它本身把轮速乘回底盘速度。舵轮则是逐轮几何：逆解 `底盘速度→(转向角, 驱动速度)`，正解反过来。

记住这个正/逆对偶，下面 §4 功率限制作用在逆解的输出（轮速→力矩）上，§5 里程计就是正解——整条链的位置就清楚了。

---

## 3. 三种工作模式

底盘响应操作手的方式分三种模式，由决策层通过 `ChassisCmd` 指定。它们都是从比赛规则来的。

### 3.1 FOLLOW：小陀螺，底盘自动跟随云台

比赛允许底盘一边全向移动一边自转（"小陀螺"），让自己难被瞄准。这要求**云台和底盘解耦**：操作手推摇杆控制的是底盘在**空间中**的移动方向，而底盘自身的朝向由云台决定——底盘始终把"车头"对齐云台指向。

```
摇杆前推 → 底盘朝【云台指向】的前方走
摇杆左推 → 底盘朝【云台指向】的左边走
云台转  → 底盘自动旋转跟上
```

实现就是一个 PID 闭环：查云台 yaw 相对底盘的夹角，让底盘自转把这个夹角追到 0：

```cpp
void ChassisBase::follow(...) {
  tfVelToBase(command_source_frame_);   // 摇杆意图变换到底盘坐标系（用 TF）
  double yaw = 查云台 yaw 相对底盘的角度;
  double follow_error = angles::shortest_angular_distance(yaw, 0);
  pid_follow_.computeCommand(-follow_error, period);
  vel_cmd_.z = pid_follow_.getCurrentCmd() + 前馈项;   // 底盘角速度
}
```

这里就用到了 [transform](./transform.md) 的 TF：`tfVelToBase` 把"操作手想让车往哪走"从云台坐标系转到底盘坐标系，`follow_error` 也来自查询云台和底盘的相对朝向。操作手因此完全不用操心底盘朝向——这正是小陀螺好用的原因。

### 3.2 TWIST：正弦扭动避弹

底盘在 ±45° 范围内按正弦规律来回扭，同时还能全向移动，让自己更难被瞄准。常用于哨兵避弹：

```cpp
vel_cmd_.z = pid_follow_.computeCommand(
    -angles::shortest_angular_distance(yaw,
        twist_angular_ * sin(2*M_PI*time.toSec()) + off_set));
```

### 3.3 RAW / 断连保护

RAW 是"原始"模式，只做坐标变换、不主动产生运动，配合急停用。更重要的是**断连保护**：如果遥控器指令超过 `timeout_` 没更新，底盘自动把速度归零，防止失控暴走：

```cpp
if ((time - cmd.stamp_).toSec() > timeout_) {
  vel_cmd_.x = vel_cmd_.y = vel_cmd_.z = 0.;   // 速度归零，收到新指令自动恢复
}
```

---

## 4. 功率限制：怎么分配总功率

### 4.1 功率从哪来

裁判系统测的底盘功率近似是各轮"力矩 × 转速"之和：

$$P = \tau_1\omega_1 + \tau_2\omega_2 + \tau_3\omega_3 + \tau_4\omega_4$$

功率限制器 `powerLimit()` 的任务是：**在不改变运动方向的前提下，把各轮力矩统一乘一个系数 $k\ (0\le k\le 1)$，使总功率压到限制以下。** 统一缩放才不会改变方向——如果只砍某个轮子，底盘就跑偏了。

### 4.2 为什么用二次方程

天真的想法是"超了就所有轮力矩 ×0.8"。但这里有个耦合：力矩一变，电机转速也会跟着变，功率不是简单跟力矩成正比。所以 rm-controls 用一个**经验二次模型**求缩放系数 k：

```cpp
for (auto& joint : joint_handles_) {
  double cmd = joint.getCommand();   // 当前指令力矩
  double vel = joint.getVelocity();  // 当前实际转速
  a += cmd*cmd * effort_coeff;       // 力矩项
  b += fabs(cmd * vel);              // 交叉项
  c += vel*vel * vel_coeff - power_offset - power_limit;  // 速度项 + 静态损耗
}
double k = (-b + sqrt(b*b - 4*a*c)) / (2*a);   // 解二次方程取正根
for (auto& joint : joint_handles_)
  joint.setCommand(joint.getCommand() * std::min(k, 1.0));  // 统一缩放
```

三个系数的含义：

| 参数 | 含义 |
| --- | --- |
| `effort_coeff` | 力矩对功率的贡献权重 |
| `vel_coeff` | 转速对功率的贡献权重 |
| `power_offset` | 系统静态功耗（电路损耗、摩擦等） |

这三个参数支持 `dynamic_reconfigure` **运行时调参**，不用重编译——初值来自电机标称参数，比赛中看实际功率表现微调。二次模型比线性缩放更贴近真实物理，能减少超调、少扣血。

### 4.3 电容与上坡补偿

- **超级电容**：裁判系统允许用超级电容存能——充电时功率极低，放电时爆发高功率。所以功率控制不是一个静态上限，而是一个 CHARGE→NORMAL→BURST 的**状态机**，由决策层的 `PowerLimit` 管理（见 [manual](./manual.md)）。
- **上坡补偿**：上坡时重力让后轮负载加大，若统一缩放会导致后轮力矩不足爬不上去。所以检测到俯仰角超阈值时，给**后轮**多分配一些力矩（乘一个 >1 的 `scale_`）。

---

## 5. 里程计

里程计走的是**正运动学**方向：读各轮实际转速 → 用 `odometry()` 合成底盘速度 → 积分成位姿 → 发布 `odom→base_link` 的 TF。

```cpp
void ChassisBase::updateOdom(...) {
  geometry_msgs::Twist v = odometry();      // 各轮实速 → 底盘速度（子类实现）
  position += linear_vel * dt;              // 积分位置
  rotation += angular_vel * dt;             // 积分朝向
  robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");  // 发 TF
}
```

它发布的是**航迹推算（Dead Reckoning）**——靠轮速积分推位置，不是绝对定位，长时间会累积误差（轮子打滑尤其明显）。所以通常配合视觉/激光 SLAM 做修正：外部定位通过 `outsideOdomCallback` 灌进来，融合掉漂移。这条 `odom→base_link` 的 TF 也正是 [transform](./transform.md) 里说的、`chassis_controller` 作为 TF 生产者贡献的那条边。

---

## 6. 标定

底盘要不要标定，取决于关节是不是需要**绝对角度**——这正是 [hardware](./hardware.md) 第 2 节讲的那条判据。

**大多数底盘不用标定。** 普通麦轮 / 全向 / 舵轮底盘的驱动电机（3508 / 2006）虽然是增量式编码器，但只做**速度控制**，根本不关心"绝对转到哪一格"（[hardware](./hardware.md) §2.2）。所以这些底盘的 `chassis_calibration` 流水线通常是**空的**——上电即可用。

**需要标定的是位置控制关节**，主要出现在特殊底盘上：

- 轮腿 / 平衡机器人的**髋关节（hip）**——要知道腿的绝对角度才能算平衡
- 主动悬挂的**悬挂腿**——要知道悬挂高度
- 舵轮的**转向关节**（若用增量编码器）——要知道轮子朝向的绝对零点

这些关节用**撞限位标定**（Mechanical，[hardware](./hardware.md) §4.5）：驱动到机械限位、速度骤降、设 offset。

编排上有个和 [hardware](./hardware.md) §4.7 呼应的细节。那节说"标定一个机构时其他机构照常动，因为 joint 集合互不相交"。底盘标定恰恰是**相交**的反例：髋关节同时被 `chassis_controller` 和 `leg_controller` 用到，所以标定髋关节时必须**两个都停**：

```yaml
# legged_balance 的 chassis_calibration
chassis_calibration:
  - start_controllers: [controllers/right_hip_calibration_controller]
    stop_controllers:  [controllers/chassis_controller, controllers/leg_controller]  # 共享关节，一起停
    services_name: [/controllers/right_hip_calibration_controller/is_calibrated]
  - start_controllers: [controllers/left_hip_calibration_controller]
    stop_controllers:  [controllers/chassis_controller, controllers/leg_controller]
    services_name: [/controllers/left_hip_calibration_controller/is_calibrated]
```

两个髋关节各一步、串行标定。触发时机是**底盘电源 ON**（`chassisOutputOn` → `chassis_calibration_->reset()`）。整套标定编排（谁先谁后、怎么轮询完成）由决策层的 `CalibrationQueue` 管，见 [manual](./manual.md) §4。

---

## 7. 配置项说明

底盘控制器配置在 `rm_controllers/<robot>.yaml` 的 `chassis_controller` 下。关键字段：

```yaml
chassis_controller:
  type: rm_chassis_controllers/OmniController   # 轮系类型，决定用哪个子类
  publish_rate: 100
  wheel_radius: 0.07625        # 轮半径
  timeout: 0.1                 # 断连保护超时（秒）
  pid_follow: { p: 10.0, i: 0, d: 0.01, ... }   # FOLLOW 模式的跟随 PID
  power:                       # 功率限制系数（支持 dynamic_reconfigure）
    effort_coeff: 1.90
    vel_coeff: 0.0088
    power_offset: -9.8
  wheels:                      # 每个轮子一段
    left_front:
      pose: [0.2018, 0.172, 0.0]   # x y z 安装位置
      roller_angle: -0.785          # 滚轮角度（-45° = -π/4）
      joint: left_front_wheel_joint # URDF 关节名（要和 hardware 配置一致）
      pid: { p: 0.25, i: 0, d: 0.0, ... }   # 该轮速度 PID
```

常见调整：

- **换轮系** → 改 `type`（`OmniController` / `SwerveController` / `ActiveSuspensionController` / `BalanceController`），并按该子类要求补齐 `wheels` 或悬挂/平衡参数
- **调功率表现** → 改 `power` 三系数，可用 `dynamic_reconfigure` 在线调
- **调跟随手感** → 改 `pid_follow`
- **调最大速度** → 不在这里，在决策层 `rm_manual` 的 `vel.max_linear_x` 分段映射里（功率越高限速越高），见 [manual](./manual.md)

---

## 8. 小结

- 底盘控制器最终输出**力矩**，因为功率限制需要直接缩放力矩——这是整个设计的主线。
- 抽象链：底盘速度 →（逆运动学）各轮目标速度 →（速度 PID）力矩 →（功率缩放）限幅力矩 → 硬件。
- 轮系：Omni（麦轮/全向，4×3 伪逆，最常用）、Swerve（舵轮，逐轮几何，零半径转向）、ActiveSuspension（继承 Omni + 悬挂）、Balance（倒立摆 + IMU）。
- **FOLLOW = 小陀螺**：查云台相对底盘夹角，PID 让底盘自转跟平，操作手只管空间移动方向，用到 TF。
- **功率限制**用二次经验模型求统一缩放系数，兼顾力矩-转速耦合；配合超级电容状态机和上坡后轮补偿。
- 里程计是轮速积分的**航迹推算**，发布 `odom→base_link` TF，靠外部 SLAM 修正漂移。
- **运动学**分逆解（`moveJoint`，底盘速度→轮速，控制用）和正解（`odometry`，轮速→底盘速度，里程计用），互为逆运算。
- **标定**：普通底盘轮只控速、不标定；只有轮腿髋关节、悬挂等位置关节要撞限位标定，且因共享关节需和 `leg_controller` 一起停。

下一站 [gimbal](./gimbal.md)：云台怎么瞄准，串级 PID、重力补偿、弹道解算和自瞄。
