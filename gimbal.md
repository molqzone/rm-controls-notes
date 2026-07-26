# 云台：是怎么瞄准的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（关节、力矩、IMU）、[transform](./transform.md)（TF，弹道和补偿都要查坐标系）

云台控制器（`gimbal_controller`）claim 的是 yaw 和 pitch 两个 joint，负责让枪管指向该指的地方。这篇讲清楚：云台的双轴结构、为什么用串级 PID、pitch 为什么要重力补偿、打移动目标的弹道解算、视觉自瞄，以及底盘乱动时云台怎么稳住指向。

和底盘一样，云台控制器的复杂度不是来自"PID 调参",而是来自**比赛对射击精度的四条硬要求**，我们会一条条对应到实现。

---

## 1. 双轴结构与硬件

### 1.1 yaw / pitch 两轴

云台是一个**串联**机构：

```
       枪管 / 相机
         │
      pitch 轴（上下俯仰）
         │
      yaw 轴（水平旋转）
         │
       底盘
```

- **yaw 轴**：水平旋转，通常是**连续关节**（可以无限转，不限位）
- **pitch 轴**：上下俯仰，有物理限位（比如 -0.5 ~ 0.5 rad）

两轴机械耦合——yaw 一转，pitch 电机跟着整体转。控制器内部其实用一个 `unordered_map` 支持任意数量的轴（预留扩展，比如某些哨兵用三轴双 yaw），但目前 rm-controls 的所有机器人都只用 yaw + pitch 两轴。

### 1.2 电机与 IMU

- 云台电机通常是 **GM6020**：绝对值编码器，[hardware](./hardware.md) 讲过——上电就知道绝对角度，**不需要标定**，而且力矩大到可以直驱云台。
- **IMU 装在云台上**（yaw 和 pitch 之间，`gimbal_imu`）。为什么装云台不装底盘？因为云台控制器关心的是"枪口是否水平、指向是否稳",而不是"底盘是否水平"。底盘在颠簸地面上倾斜时，云台要能独立保持指向。

云台控制器声明的硬件接口，正对应它要做的事：

```cpp
class Controller : public controller_interface::MultiInterfaceController<
    rm_control::RobotStateInterface,          // 查 TF：世界系、底盘-云台相对角、相机位置
    hardware_interface::ImuSensorInterface,   // IMU 数据（重力补偿、姿态）
    hardware_interface::EffortJointInterface> // 向 yaw/pitch 发力矩
```

---

## 2. 串级 PID：角度环套速度环

### 2.1 结构

云台控制的核心是**串级（cascade）PID**：外环管角度，内环管速度，最后输出力矩。

```
目标角度 pos_des
   │
   ▼  外环：角度 → 目标速度
pid_pos_（角度误差 → 目标角速度）      ← 消除角度静差
   │
   ├── + 速度前馈（vel_des × k_v）      ← 让响应更跟手
   ├── + 底盘补偿（chassis_compensation）← 抗底盘扰动（见第 5 节）
   ▼  内环：速度 → 力矩
ctrls_（目标角速度 - 实际角速度 → 力矩） ← 克服阻力矩
   │
   ├── + 重力补偿（gravityFeedForward）  ← pitch 抗自重（见第 3 节）
   ▼
joint.setCommand(effort)
```

外环是一个位置 P 控制器，把"角度差"变成"该以多快的角速度去追";内环是一个速度 PI 控制器（其实就是 [chassis](./chassis.md) 里也用到的 `JointVelocityController`），把"速度差"变成力矩。

代码里这两环不写死 yaw/pitch，而是用 map 支持任意轴（目前只用两轴，预留扩展）——索引 0 = yaw、1 = pitch：

```cpp
std::unordered_map<int, std::unique_ptr<effort_controllers::JointVelocityController>> ctrls_;    // 内环：速度→力矩
std::unordered_map<int, std::unique_ptr<control_toolbox::Pid>>                        pid_pos_;   // 外环：角度→速度
std::unordered_map<int, urdf::JointConstSharedPtr>                                    joint_urdfs_; // 供限位钳制
```

上面那张结构图落到代码就是：`pid_pos_[i]` 算目标速度 → 加前馈和底盘补偿 → 喂给 `ctrls_[i]` 算力矩 → `setCommand`，`joint_urdfs_[i]` 供 `setDesIntoLimit()` 钳限位。

### 2.2 为什么要串级，不用单环

如果只用一个"角度 → 力矩"的单环 PID：当底盘急转、外力矩扰动云台时，**必须等到角度已经偏出去了**，PID 才反应过来去纠正——滞后很明显，云台会晃。

串级的妙处在内环：底盘的扰动会**先反映成角速度的变化**，内环速度 PID 在角度还没明显偏移时就已经通过速度偏差把扰动顶回去了，响应快一个数量级。角度环则只需专注消除稳态的角度静差。

**速度前馈**（`vel_des × k_v`）是另一个手感关键：操作手摇杆给的是"想转多快",这个速度直接前馈到内环，不用经过"角度积分→再微分"的延迟，云台就更跟手。

---

## 3. 重力补偿（对应规则三）

> **规则三：云台质心不在旋转轴上。** pitch 电机要一直扛着云台自重产生的重力矩，且这个力矩随 pitch 角变化。

如果不管它，pitch PID 只能靠积分项慢慢把重力静差补上——结果是积分饱和、响应变慢、快速运动时过冲。正确做法是**前馈**把重力矩直接算出来加上去：

$$\tau_{\text{gravity}} = m\,g\,d\cos\theta_{\text{pitch}}$$

其中 $m$ 是云台质量，$d$ 是质心到 pitch 轴距离，$\theta_{\text{pitch}}$ 是当前俯仰角。

```cpp
double Controller::gravityFeedForward(...) {
  if (!enable_gravity_compensation_) return 0.0;
  double pitch_angle = pos_real[PITCH];
  return mass_ * g_ * distance_ * std::cos(pitch_angle);
}
```

**重力补偿是前馈不是反馈**——它直接叠在 PID 输出上，让 PID 不必靠积分就能抵消重力，因此更快更稳。这正是 overview 里"调云台 pitch 重力补偿"那个调车例子的原理：把 PID 临时置零、松手，云台往下掉就加大 `gravity`，往上抬就减小，几分钟调好。

```yaml
feedforward:
  gravity: -1.944                              # 重力前馈力矩系数
  enable_gravity_compensation: true
  mass_origin: [0.06662, -0.012383, 0.038901]  # 云台质心在云台坐标系中的位置
```

---

## 4. 弹道解算（对应规则一）

> **规则一：弹丸有飞行时间。** 弹丸初速十几到几十 m/s，目标在 5~15m 外，弹丸要飞 0.3~1.5 秒。这期间目标在移动，空气阻力还让它下坠。

所以云台不能只是"指向目标现在的位置",而要**预测目标在弹丸命中那一刻的位置，并补偿弹丸下坠**。这拆成两个求解器。

### 4.1 BulletSolver：目标现在往哪、命中时到哪

它回答"当弹丸飞到时，目标在哪"。因为"飞行时间"依赖"距离",而"距离"又依赖"预测位置",这是个鸡生蛋问题，用**迭代**几次收敛：

```cpp
for (int i = 0; i < max_iterations; i++) {
  double flight_time = distance / bullet_speed;   // 估飞行时间
  flight_time += total_delay_;                    // 加上控制/视觉延迟
  target_pos = current_pos + target_vel * flight_time
             + 0.5 * target_accel * flight_time*flight_time;  // 预测目标位置
  distance = norm(target_pos - gun_pos);          // 用新位置重算距离
  if (converged) break;                           // 一般 3~5 次收敛
}
```

视觉给的目标位置在**相机坐标系**下，BulletSolver 用 [transform](./transform.md) 的 `lookupTransform` 查相机→世界系的变换，把目标换算到世界系再预测。

### 4.2 BallisticSolver：算该抬多少 pitch

预测出命中点后，还要算"枪口该抬多高才能让弹丸正好落在那里"。空气阻力不能忽略——15m 外弹丸比真空模型多下坠约 0.3m，足够打偏。所以用 **RK4 数值积分**仿真弹道，再用**二分法**搜出命中所需的 pitch 角。

状态向量 $s = [x, y, z, v_x, v_y, v_z]$，动力学方程（$i$ 取 $x,y,z$）：

$$\frac{dv_i}{dt} = -C_d\cdot\tfrac{1}{2}\rho\, v\, v_i / m \quad(\text{空气阻力}),\qquad \frac{dv_z}{dt}\ \text{再减去}\ g$$

其中 $v=\sqrt{v_x^2+v_y^2+v_z^2}$ 是速率。给定一个 pitch 角 $\theta$，从枪口初值出发：

$$s(0) = [\,0,\ 0,\ 0,\ v_0\cos\theta,\ 0,\ v_0\sin\theta\,]$$

每步 $dt$ 做一次 RK4 积分推进，看落点是否命中目标；外层用二分法搜索使弹道命中的最佳 $\theta$，即最终的 `ballistic_pitch_`。

配置里的弹道参数：

```yaml
ballistic_solver:
  mass: 0.0445           # 弹丸质量 (kg)
  radius: 0.02125        # 弹丸半径
  gun_offset_x: 0.20     # 枪口相对云台中心的偏移
  Cd_value: 0.63         # 空气阻力系数
  initial_vel: 16.4      # 子弹初速 (m/s)
  rk4_simulate_step: 0.01
```

### 4.3 子弹速度是变量

子弹初速由摩擦轮转速决定（见 [shooter](./shooter.md)），操作手还能在线加减速（V 键加、G 键减）。子弹速度一变，弹道就变，所以 shooter 会把当前子弹速度传给云台（`setBulletSpeed`），弹道解算自动适应。云台和发射之间这条"子弹速度"的耦合，是两个机构协作的典型例子。

---

## 5. 底盘前馈补偿（对应规则二）

> **规则二：底盘运动扰动云台。** 底盘快速自转（小陀螺）时，牵连运动的力矩会把云台指向带偏。

串级 PID 的内环已经能顶一部分，但更主动的办法是**前馈**：直接测底盘角速度，提前给 pitch 一个补偿量，不等偏差出现。

### 5.1 先拿到底盘速度

底盘速度不是别人告诉云台的，而是云台自己**从 TF 微分**出来的——查 `odom→base_link` 变换，对位置和姿态求微分得到线速度和角速度，再过一个滑动窗口滤波器（默认 20 点）去噪：

```cpp
void Controller::updateChassisVel() {
  odom2base_ = robot_state_handle_.lookupTransform("odom", "base_link", time);
  // 位置微分→线速度，四元数微分→角速度
  chassis_vel_->update(linear_vel, angular_vel, period);  // 带滤波
}
```

这又是一次 [transform](./transform.md) 的 TF 消费。承接这份速度的是一个 `ChassisVel` 对象，内部两个 `Vector3WithFilter<double>`（线速度、角速度各一，默认 20 点窗口）：

```cpp
class ChassisVel {
  std::shared_ptr<Vector3WithFilter<double>> linear_, angular_;
  void update(double lin[3], double ang[3], double period);  // period>0.1 时清空滤波器，防断线突变
};
```

滤波器那个"长时间没更新就清空"的保护，避免重连时的突变把云台甩飞。

### 5.2 补偿模型是经验三次多项式

补偿量是底盘自转角速度 $w_z$ 的三次多项式：

$$\text{compensation}(w_z) = a\,w_z^3 + b\,w_z^2 + c\,w_z + d$$

纯理论上牵连力矩和角速度是线性关系，但实际有摩擦、齿轮间隙、线缆阻力等一堆非线性因素，所以用**三次多项式拟合实测数据**。这四个系数 `chassis_comp_a/b/c/d` 支持 `dynamic_reconfigure` 在线调——赛场上记录不同自转速度下需要的补偿量，拟合出曲线。这个补偿量被叠进串级 PID 的内环前（第 2 节结构图里那条 `+ 底盘补偿`）。

---

## 6. 四种工作模式

云台响应指令的方式分四种模式，由决策层通过 `GimbalCmd` 指定。`update()` 按当前模式分派到四个函数，每个只负责算目标角度 `pos_des`，然后统一汇到同一条控制链：

```
update()
  ├── rate(...)    RATE：摇杆速度积分成角度
  ├── track(...)   TRACK：updateBallisticSolution() → 弹道角
  ├── direct(...)  DIRECT：直接设角度
  └── traj(...)    TRAJ：世界系锁定
  然后统一 → setDesIntoLimit() → moveJoint()（串级 PID，§2）→ + gravityFeedForward()（§3）
```

模式只决定"目标角度是多少",不改控制结构——控制链（串级 PID + 重力补偿）始终是同一条。

### 6.1 RATE：手动瞄准（默认）

操作手摇杆控制云台**转多快**（角速度），控制器积分成角度。这样手感自然——摇杆回中，云台停住：

```cpp
vel_des[YAW]   = cmd_gimbal_.rate_yaw;              // 摇杆 → 目标角速度
vel_des[PITCH] = cmd_gimbal_.rate_pitch;
pos_des[YAW]   += vel_des[YAW] * dt;                // 积分成角度
pos_des[PITCH] += (vel_des[PITCH] + chassis_compensation_) * dt;
setDesIntoLimit(pos_des[PITCH], joint_urdfs_[PITCH], ...);  // pitch 限位保护
```

`setDesIntoLimit` 会把目标角度钳进 URDF 定义的关节限位（yaw 是连续关节不限位，pitch 限）。

### 6.2 TRACK：视觉自瞄（对应规则四）

视觉检测到目标，通过 `/track` 话题发来目标的位置/速度/加速度。控制器进入 TRACK 模式，跑第 4 节的弹道解算，把解出的瞄准角设为目标：

```cpp
void Controller::track(...) {
  updateBallisticSolution(time);        // BulletSolver + BallisticSolver
  pos_des[YAW]   = ballistic_yaw_;
  pos_des[PITCH] = ballistic_pitch_;
}
```

完整自瞄链路（视觉 → 决策层切 TRACK → 弹道解算 → 云台指向 → 配合发射）横跨多个模块，决策层那半段见 [manual](./manual.md)。

### 6.3 DIRECT：直接设角度

直接把云台开到指定角度，用于云台归零、工程机械臂末端定位等预设位姿。

### 6.4 TRAJ：轨迹/部署模式

云台锁定到**世界坐标系**下的预设指向（RATE/TRACK 是在云台坐标系下控制）。关键区别就在坐标系：因为锁在世界系，**底盘怎么转，云台指向都不变**——用于英雄的"部署模式",把云台钉成一个稳定的射击平台。

---

## 7. 云台运动学：指向与关节角

底盘运动学换算的是"底盘速度 ↔ 轮速";云台运动学换算的是**关节角 $(\theta_{\text{yaw}}, \theta_{\text{pitch}})$ 和枪口指向之间的关系**。云台是 2 自由度串联机构——yaw 绕竖直轴定方位、pitch 绕水平轴定俯仰。

### 7.1 正解：关节角 → 指向

设枪口在两轴都为零时指向 $+x$。pitch 把它绕水平轴抬起 $\theta_{\text{pitch}}$，得到在 yaw 坐标系下的指向单位向量：

$$\hat{d}_{\text{yaw}} = (\cos\theta_{\text{pitch}},\ 0,\ \sin\theta_{\text{pitch}})$$

再经 yaw 绕竖直轴旋转 $\theta_{\text{yaw}}$，就得到枪口在底盘/世界系里的最终指向。直观说：**pitch 决定抬多高，yaw 决定朝哪个方位**。

### 7.2 逆解：目标指向 → 关节角

控制时要反过来：给定"想打的方向"（世界系，来自弹道解算给出的命中点），反算该转到的 $(\theta_{\text{yaw}}, \theta_{\text{pitch}})$。设目标相对枪口的位移水平分量为 $(\Delta x, \Delta y)$：

$$\theta_{\text{yaw}}^{\text{des}} = \operatorname{atan2}(\Delta y,\ \Delta x)$$

yaw 就是普通的水平方位角，纯几何。但 **pitch 不是** 简单的 $\operatorname{atan2}(\Delta z, \text{水平距离})$——它是弹道解算（§4.2）搜出来的抬枪角，已经补偿了弹丸下坠和飞行时间：

$$\theta_{\text{pitch}}^{\text{des}} = \texttt{ballistic\_pitch\_} \quad(\ne \text{纯几何仰角})$$

**这就是云台逆解和普通机械臂逆解的最大不同：pitch 由弹道物理决定，不是几何解。** 这也是为什么弹道解算这么重要——它就是云台逆运动学里 pitch 那一半。

### 7.3 坐标系桥接与限位

目标来自视觉，在**相机系**下；要先用 [transform](./transform.md) 的 TF 把它换到世界系算出 $(\theta_{\text{yaw}}, \theta_{\text{pitch}})$，跟踪时再换回云台系。逆解出的目标角还要过 `setDesIntoLimit()` 钳进 URDF 关节限位——yaw 通常是连续关节（可无限转、不限位），pitch 有机械限位（如 $-0.5 \sim 0.5$ rad）。

最后，逆解只给出**目标角**，真正"追上"这个角、抵抗扰动的是 §2 的串级 PID。一句话理清分工：**弹道/逆解算"该指哪个角",串级 PID 负责"把云台开到那个角"。**

---

## 8. 标定

云台哪些轴要标定，取决于电机——回到 [hardware](./hardware.md) 第 2 节那条判据：**绝对编码器免标、增量编码器要标**。

- 用 **GM6020（绝对编码器）**的 yaw / pitch **不需要标定**（[hardware](./hardware.md) §2.1）。英雄就是这样，上电即知云台绝对角。
- 需要标定的是**增量编码器**的轴，分两类：
  1. **增量编码器版的 yaw / pitch**（部分步兵、飞镖）——用 **GPIO / 霍尔标定**（[hardware](./hardware.md) §4.5 三步法，精度高）。
  2. **云台上的辅助增量关节**：瞄准镜 `scope`、图传 `image_transmission`——用**撞限位标定**（Mechanical）。

各兵种的差异很典型：

- **英雄** `gimbal_calibration` 两步——先标图传（`search_velocity` 12.56，高速），再标瞄准镜（`search_velocity` −3.1415，反向）。yaw/pitch 免标。
- **飞镖** 的 `left_pitch` / `right_pitch` / `yaw` 三个 GPIO 标定**并发**：一步里同时启三个标定控制器，`CalibrationQueue` 等**全部** `is_calibrated` 返回 true 才算这步完成。

英雄配置示例：

```yaml
gimbal_calibration:
  - start_controllers: [controllers/image_transmission_calibration_controller]
    stop_controllers:  [controllers/image_transmission_controller]
    services_name: [/controllers/image_transmission_calibration_controller/is_calibrated]
  - start_controllers: [controllers/scope_calibration_controller]
    stop_controllers:  [controllers/scope_controller]
    services_name: [/controllers/scope_calibration_controller/is_calibrated]
```

触发时机是**云台电源 ON**（`gimbalOutputOn` → `gimbal_calibration_->reset()`）。注意标定 scope / image_transmission 时只停这两个辅助控制器，`gimbal_controller` 照常运行——云台还能瞄（joint 集合不相交，[hardware](./hardware.md) §4.7）。整套编排见 [manual](./manual.md) §4。

---

## 9. 配置项说明

云台配置在 `rm_controllers/<robot>.yaml` 的 `gimbal_controller` 下：

```yaml
gimbal_controller:
  type: rm_gimbal_controllers/Controller
  controllers:
    yaw:
      joint: "yaw_joint"
      pid:     { p: 2.4,  i: 28.0, d: 0.0, ... }   # 速度环（内环）PID
      pid_pos: { p: 10.0, i: 0.0,  d: 0.0, ... }   # 角度环（外环）PID
      k_v: 1.0      # 速度前馈系数
      accel: 40.0   # 最大角加速度
    pitch:
      joint: "pitch_joint"
      pid:     { p: 1.5,  i: 19.0, d: 0.0, ... }
      pid_pos: { p: 14.5, i: 0,    d: 0.0, ... }
      k_v: 1.0
      accel: 40.0
  bullet_solver:     { ... }   # 目标预测参数（延迟、初速映射）
  ballistic_solver:  { ... }   # RK4 弹道参数（质量、阻力、初速）
  feedforward:                 # 重力补偿
    gravity: -1.944
    enable_gravity_compensation: true
    mass_origin: [0.06662, -0.012383, 0.038901]
```

常见调整：

- **调云台手感/稳定性** → `pid`（内环速度）和 `pid_pos`（外环角度），可用 `dynamic_reconfigure` 在线调。口诀仍是 P 提响应、I 消静差、D 减震荡
- **pitch 掉头/抬头** → 调 `feedforward.gravity`
- **小陀螺时云台晃** → 调 `chassis_comp_a/b/c/d`（底盘前馈，在线调）
- **弹道打不准** → 调 `ballistic_solver` 的初速、阻力系数等
- **限位** → 在 URDF 的关节 `<limit>` 里改，不在这份 YAML

---

## 10. 小结

云台控制器的四大功能，正对应四条比赛规则：

| 规则 | 实现 |
| --- | --- |
| 弹丸有飞行时间、目标在动 | **弹道解算**：BulletSolver 迭代预测命中点 + BallisticSolver RK4 补下坠 |
| 底盘运动扰动云台 | **底盘前馈**：从 TF 微分出底盘角速度，三次多项式补偿 |
| 云台质心偏心 | **重力补偿**：前馈 $m\,g\,d\cos\theta_{\text{pitch}}$，让 PID 不靠积分 |
| 视觉自瞄 | **TRACK 模式**：收 `/track` → 弹道解算 → 云台指向 |

- 控制核心是**串级 PID**：外环角度→目标速度，内环速度→力矩，内环负责快速抗扰。
- **运动学**：yaw 定方位、pitch 定俯仰；逆解里 $\theta_{\text{yaw}}=\operatorname{atan2}(\Delta y,\Delta x)$ 是纯几何，但 $\theta_{\text{pitch}}$ 由弹道解算给出（补下坠），这是云台逆解的特别之处。
- **标定**：GM6020 的 yaw/pitch 免标；增量编码器的 yaw/pitch 用 GPIO 霍尔标定，辅助关节 scope/图传用撞限位；标定辅助关节时云台仍能瞄。
- 云台用 GM6020（绝对编码器，免标定），IMU 装云台上以独立保持指向。
- 云台与 [shooter](./shooter.md) 通过"子弹速度"耦合，与 [chassis](./chassis.md) 通过 TF 上的底盘运动耦合，大量使用 [transform](./transform.md) 的坐标查询。

下一站 [shooter](./shooter.md)：子弹是怎么打出去的——摩擦轮、拨弹盘、发射状态机、卡弹检测与热量限制。
