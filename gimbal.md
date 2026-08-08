# 云台：是怎么瞄准的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（关节、力矩、IMU）、[control](./control.md)（串级 PID 与前馈）、[transform](./transform.md)（TF，弹道和补偿都要查坐标系）

常见的枪械云台是 yaw 和 pitch 两个 joint 的串联机构，`gimbal_controller` 负责把配置中的关节带到目标。这篇先以常见双轴为例，讲清楚串级 PID、pitch 重力补偿、移动目标的弹道解算、视觉自瞄，以及底盘扰动下怎样保持指向；具体关节数和命名必须以对应兵种配置为准。

和底盘一样，云台控制器的复杂度不是来自"PID 调参",而是来自**比赛对射击精度的四条硬要求**，我们会一条条对应到实现。

---

## 1. 双轴结构与硬件

### 1.1 典型的 yaw / pitch 两轴

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

两轴机械耦合——yaw 一转，pitch 电机跟着整体转。这张图只描述常见构型，不能当成所有兵种的 joint 表：当前飞镖配置就有 `left_pitch`、`right_pitch` 和 `yaw` 三个云台轴。阅读后面的双轴公式时，要把它理解为该构型的示例；多轴机构必须按完整运动链、限位和控制器配置分别处理。

### 1.2 电机与 IMU

- 云台电机常见 **GM6020**。它能在上电后给出转子**单圈**角度，但这个读数不等于机构关节的机械零位：减速比、装配相位、传动间隙和多圈关系都还需要定义。是否要标定由机构和硬件配置中的 `need_calibration` 决定，不能仅凭“用了 GM6020”就断言免标定。
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

### 2.3 RATE 是速率指令，不是拆掉位置环

“摇杆控制转速”容易产生一个误解：RATE 模式是不是只闭速度环、松手后就完全不管角度？**rm-controls 不是这样。** RATE 描述的是上层指令语义，控制器仍把速率积分成位置目标，再走同一套位置外环和速度内环：

$$
\omega_{cmd}=\omega_{operator}+\omega_{comp},\qquad
q_d[k]=q_d[k-1]+\Delta t\,\omega_{cmd}[k]
$$

$$
\omega_d=PID_{pos}(q_d-q)+k_v\omega_{cmd}
$$

这样兼得两件事：操作手感受上是“拨多少就转多快”，摇杆回中后位置目标不再变化；受到碰撞或后坐力时，位置环又会把云台拉回松手那一刻的指向。真正拆掉位置环的“纯速控”没有这种位置保持和抗扰能力，不能和 RATE 混为一谈。

离散积分必须使用本拍实际的 `period`，不能硬编码 0.001 s。yaw 的角度误差要做 unwrap / 最短路，pitch 的目标在积分前后都要受机械限位保护。RATE、TRACK、DIRECT、TRAJ 互相切换时，还要把新模式的内部目标接到当前角或新模式首个合法目标，并清理旧模式的积分与滤波状态；否则切换第一拍就会因目标不连续而猛跳。

### 2.4 IMU 角速度不能直接当关节速度

GM6020 报文的角度分辨率约为 $2\pi/8192$ rad，而速度通常按整数 rpm 给出，低速时量化明显。IMU 的角速度更细，理论上可以改善云台速度反馈，但它测到的是**传感器刚体相对惯性系的三维角速度**，不是某个关节相对父关节的速度。不能把 IMU 的 y/z 分量直接塞进 pitch/yaw 速度环。

以底盘系 $C$、yaw 系 $Y$、pitch 系 $P$、惯性系 $O$ 为例，角速度递推关系是：

$$
{}^Y\boldsymbol\omega_{Y/O}=
{}^Y R_C\,{}^C\boldsymbol\omega_{C/O}+\dot\alpha\,\boldsymbol e_z
$$

$$
{}^P\boldsymbol\omega_{P/O}=
{}^P R_Y\,{}^Y\boldsymbol\omega_{Y/O}+\dot\beta\,\boldsymbol e_y
$$

要得到关节速度，应把基座角速度变换到对应关节坐标系，从末端惯性角速度中扣掉父级运动，再投影到关节轴上。实际部署还要同时处理：

- IMU 与编码器的时间戳对齐；
- 陀螺零偏、温漂和滤波延迟；
- IMU 安装方向与 URDF/TF 的一致性；
- 某个传感器失效时回退到电机速度反馈。

当前控制链里的 `JointVelocityController` 使用 joint 速度反馈；上面的推导是把 IMU 引入内环时必须满足的约束，不代表声明了 `ImuSensorInterface` 就已经自动完成了关节速度解算。

---

## 3. 重力补偿（对应规则三）

> **规则三：云台质心不在旋转轴上。** pitch 电机要一直扛着云台自重产生的重力矩，且这个力矩随 pitch 角变化。

如果不管它，pitch PID 只能靠积分项慢慢把重力静差补上——结果是积分饱和、响应变慢、快速运动时过冲。正确做法是**前馈**把重力矩直接算出来加上去。

### 3.1 从通用模型到平地简化式

对任意关节 $i$，重力矩最稳妥的写法是把质心重力对关节轴取矩：

$$
\tau_{g,i}=\boldsymbol a_i^T\left(\boldsymbol r_c\times m\boldsymbol g\right)
$$

其中 $\boldsymbol a_i$ 是关节轴，$\boldsymbol r_c$ 是关节轴原点到负载质心的向量；三个向量必须先变换到同一个坐标系。这个形式自然覆盖底盘倾斜、质心有侧向偏移时 yaw 也承受重力矩的情况。

底盘水平、只考虑 pitch 时可化成：

$$
\tau_g=-K\cos(\beta+\gamma),\qquad K=mgd
$$

其中 $\beta$ 是 pitch 角，$d$ 是质心到转轴的距离，$\gamma$ 是“pitch 零位”与质心连线之间的安装相位；正负号取决于 URDF 关节正方向。配置里的 `mass_origin` 正是在描述质心向量，不能在质心明显偏移时仍把模型简化成没有相位的单个 $\cos\beta$。

```cpp
double Controller::gravityFeedForward(...) {
  if (!enable_gravity_compensation_) return 0.0;
  // 概念式：先把重力与质心向量变到同一坐标系，再投影到关节轴。
  return joint_axis.dot(mass_origin.cross(mass * gravity_vector));
}
```

**重力补偿是前馈不是反馈**——它直接叠在 PID 输出上，让 PID 不必靠积分就能抵消大部分重力，因此更快更稳；剩余的质心误差、摩擦和装配变化仍由反馈环修正。

```yaml
feedforward:
  gravity: -1.944                              # 重力前馈力矩系数
  enable_gravity_compensation: true
  mass_origin: [0.06662, -0.012383, 0.038901]  # 云台质心在云台坐标系中的位置
```

### 3.2 怎样安全地标定重力参数

图纸测量能给初值，但线缆、螺丝、相机和弹仓都会改变真实质心。更可靠的方法是在多个静态 pitch 角采集维持平衡所需的力矩 $u_i$，拟合：

$$
u_i=A\cos\beta_i+B\sin\beta_i
$$

再由 $A,B$ 恢复 $K,\gamma$。每个角度最好从正、反两个方向各接近一次并取中值，减小库仑摩擦对重力参数的污染。若底层命令是电流，要按 $\tau=K_t I$ 做单位换算；已是 N·m 的 effort 就不要再次乘除转矩常数。

实车标定时不要“清零全部 PID 后松手”：

1. 机械托住云台，设置保守的关节、速度和力矩限制；
2. 保留低增益稳定环，或用可靠工装固定在采样角；
3. 从很小的前馈开始，观察稳定后的反馈输出，而不是等机构自由下坠；
4. 覆盖完整合法角域，最后用人工轻推检验能否恢复；
5. 弹仓与 pitch 固连时，满弹和空弹应分别验证，固定参数只是折中。

### 3.3 摩擦与基座加速度前馈的边界

重力不是唯一的可预测扰动。低速云台常有库仑摩擦，可从多个正反转速点采样维持匀速所需的力矩，拟合一个受限的奇函数，例如：

$$
\tau_f(\omega)=\operatorname{sat}\left[
\operatorname{sgn}(\omega)(a|\omega|^2+b|\omega|+c)
\right]
$$

零速附近应改成线性过渡或 `tanh`，否则 `sign` 会因噪声反复翻转、产生抖振。更完整的模型还能加入 $M(q)\ddot q+C(q,\dot q)\dot q$，补偿底盘加减速带来的惯性力矩；但 IMU 线加速度在 RM 车体振动下噪声很大，双轴云台通常做到重力项和必要的摩擦项就已经足够。复杂模型只有在日志证明它解决了明确误差时才值得部署。

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

预测出命中点后，还要算“枪口该抬多高才能让弹丸正好落在那里”。当前 `BallisticSolver` 先由水平位移求 yaw，再把问题化成垂直平面内的 **x-z 二维**弹道；它不是三维积分，也不是二分搜索。

积分状态是 `s = [x, z, vx, vz]`，初值为 `x = z = 0`、`vx = v0*cos(theta)`、`vz = v0*sin(theta)`。代码使用的阻力大小为：

```text
F_drag = 0.5 * air_density * C_D * pi * radius^2 * v^2
v = sqrt(vx^2 + vz^2)
```

其中 `C_D = Cd_value + Cd_slope * (target_distance - Cd_distance)`。因此每个 RK4 子步的方程是：

```text
dx/dt  = vx
dz/dt  = vz
dvx/dt = -(F_drag / mass) * vx / v
dvz/dt = -g - (F_drag / mass) * vz / v
```

积分推进到 `x >= target_distance` 后，代码在线性插值的 `z_at_target` 上计算残差 `f(theta) = z_at_target - target_height`。外层以查表 `output_pitch_match` 的结果为初值，用有限差分近似导数的**牛顿迭代**更新 pitch；每步受 `max_newton_step` 限制，达到 `newton_convergence_tol` 或超过 `max_newton_iterations` 后结束。迭代失败时回退到初始查表角。

配置里的弹道参数：

```yaml
ballistic_solver:
  mass: 0.0445           # 弹丸质量 (kg)
  radius: 0.02125        # 弹丸半径
  gun_offset_x: 0.20     # 枪口相对云台中心的偏移
  Cd_value: 0.63         # 空气阻力系数
  Cd_distance: 12.0      # 阻力系数的距离参考
  Cd_slope: 0.0          # 阻力系数随距离的斜率
  initial_vel: 16.4      # 子弹初速 (m/s)
  rk4_simulate_step: 0.01
  newton_pitch_epsilon: 0.0001
  max_newton_iterations: 5
```

### 4.3 子弹速度是变量

子弹初速由摩擦轮转速决定（见 [shooter](./shooter.md)），操作手还能在线加减速。当前链路不是 shooter 控制器直接调用云台：`rm_manual` 从 `ShooterCommandSender` 取速度后，调用 `GimbalCommandSender::setBulletSpeed()` 填入 `GimbalCmd`；跟踪用的 `BulletSolver` 会消费这个字段。

要注意这与本节的 `BallisticSolver` 是两条代码路径：当前 `BallisticSolver::solver()` 使用自身配置的 `initial_vel`，并不读取 `GimbalCmd.bullet_speed`。因此不能把它描述成“RK4 弹道会随当前摩擦轮速度自动更新”；若要做到这一点，须在配置重载或代码中显式把速度接到该求解器。

---

## 5. 底盘运动补偿（对应规则二）

> **规则二：底盘运动扰动云台。** 底盘快速自转（小陀螺）时，牵连运动的力矩会把云台指向带偏。

串级 PID 的内环已经能顶一部分，但更主动的办法是**前馈**：估计底盘速度，提前给关节目标或力矩一个补偿量，不等偏差出现。这里要分清两层：坐标运动学给出“为了惯性系稳定，关节理论上该反转多快”；经验模型再补机械摩擦、线缆和时延留下的残差。

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

### 5.3 几何自稳与经验补偿不是同一件事

如果目标是“底盘转动时枪口在惯性系保持不动”，应先从角速度递推得到关节反向速度。仍沿用 §2.4 的坐标系：

$$
\dot\alpha_{comp}=-\boldsymbol e_z^T
{}^Y R_C\,{}^C\boldsymbol\omega_{C/O}
$$

$$
\dot\beta_{comp}=-\boldsymbol e_y^T{}^P R_Y
\left({}^Y R_C\,{}^C\boldsymbol\omega_{C/O}
+\dot\alpha_{comp}\boldsymbol e_z\right)
$$

这两个量可以并入 §2.3 的 $\omega_{cmd}$，既参与位置目标积分，也作为速度前馈。它表达的是**几何自稳**：把基座角速度在可控关节轴上的分量抵消掉。双轴云台只能约束 yaw/pitch 两个投影，不能让完整三维角速度都为零，底盘 roll 扰动仍然无法凭空消失。

本节前面的 TF 微分 + 三次多项式是**经验残差补偿**：它可以吸收执行器滞后、摩擦、线缆和结构非线性，但不应被当作上述几何关系的推导。若两条通道同时使用，要分别记录输出并防止重复补偿；经验多项式只能在采样覆盖过的角速度区间内使用，区间外必须限幅。

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

操作手摇杆控制云台**转多快**（角速度），控制器积分成角度。这样手感自然——摇杆回中后目标角停止变化，位置环继续守住当前指向：

```cpp
double dt = period.toSec();                         // 使用本拍真实周期
vel_des[YAW]   = cmd_gimbal_.rate_yaw;              // 摇杆 → 目标角速度
vel_des[PITCH] = cmd_gimbal_.rate_pitch;
pos_des[YAW]   = unwrap(pos_des[YAW] + vel_des[YAW] * dt);
pos_des[PITCH] += (vel_des[PITCH] + chassis_compensation_) * dt;
setDesIntoLimit(pos_des[PITCH], joint_urdfs_[PITCH], ...);  // pitch 限位保护
```

`setDesIntoLimit` 会把目标角度钳进 URDF 定义的关节限位（yaw 是连续关节，误差走最短路；pitch 有界）。注意这仍是串级**位置控制**，不是只发角速度的纯速控，见 §2.3。

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

### 7.4 异构云台不只是“多加一个电机”

控制器内部用 map 保存多个轴，只解决了数据组织问题，并不意味着三轴、大小双 yaw 或双枪云台自动获得正确控制。增加一个轴至少要重新回答四个问题：

1. **运动链怎样递推**：每增加一个关节，就多一段旋转变换和一项相对角速度；IMU 速度解算、重力矩和弹道枪口位姿都要沿完整 TF/关节链计算，不能直接把几个编码器角度相加。
2. **在哪个参考系闭环**：小 yaw 若在世界系控制，底盘和大 yaw 的运动已经包含在实际姿态里，不应再重复叠一遍“反向自稳”；若在父关节系控制，则必须显式补父级运动。
3. **有限关节怎样回中**：大小双 yaw 的小轴通常不能无限转。应让大 yaw 缓慢接管低频大角度运动，把小 yaw 保持在中央高响应区，并用滞回避免两个轴在分界点来回抢控制权。
4. **机构之间怎样避碰**：双枪云台的合法角区会随另一枪的位置实时变化。应由几何模型生成动态限位，给碰撞边界留制动距离；目标选择加入锁定滞回，避免两个目标权重接近时频繁切换。TF、目标或限位计算失效时回安全位，而不是继续沿上一条轨迹运动。

三轴 yaw-roll-pitch 云台还多一个事实：roll 可以补双轴云台无能为力的横滚扰动，但也增加质量和转动惯量。它的自稳目标应写成末端姿态约束，再通过雅可比或角速度递推分配到三个关节；不能把双轴公式复制一份改变量名。

这些属于针对机械构型的扩展设计，不是当前双轴 `gimbal_controller` 的默认能力。实现前应先把 URDF、动态限位、故障回退和仿真碰撞测试补齐，再谈 PID 参数。

---

## 8. 标定

云台轴是否标定不能只按编码器类型判断。GM6020 的单圈角度仍需与机构零位建立关系；当前硬件层是否等待标定，以对应电机配置中的 `need_calibration` 为准。外部参考可以是 GPIO/Hall、机械限位或其他与机构相匹配的方法。

当前配置里的两个例子说明了这个判据：

1. 英雄的 `gimbal_calibration` 队列只编排图传 `image_transmission` 和瞄准镜 `scope`；不要把这推广成“所有 GM6020 云台轴都免标”。
2. 飞镖的 `left_pitch`、`right_pitch`、`yaw` 都配置了 `need_calibration: true`，并分别使用 GPIO 标定控制器；尽管这三轴同为 GM6020，仍必须先建立机构零位。

各兵种的差异很典型：

- **英雄** `gimbal_calibration` 两步——先标图传（`search_velocity` 12.56，高速），再标瞄准镜（`search_velocity` −3.1415，反向）。这是该车的辅助关节标定编排，不是由电机型号推出的通用结论。
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

触发时机是**云台电源 ON**（`gimbalOutputOn` → `gimbal_calibration_->reset()`）。标定 scope / image_transmission 时只停相应辅助控制器，`gimbal_controller` 的 joint 集合不相交时可以继续运行；是否能同时瞄准仍要按该车实际的 controller claim 配置确认。整套编排见 [manual](./manual.md) §4。

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
- **标定**：由机构零位参考和每个电机的 `need_calibration` 决定，而不是简单按“GM6020/增量编码器”二分；飞镖三轴 GM6020 就需要 GPIO 标定，英雄当前队列标的是 scope/图传。
- GM6020 提供单圈转子角；IMU 装在云台上有助于独立保持指向，但二者都不自动定义机械零位。
- 云台与 [shooter](./shooter.md) 通过"子弹速度"耦合，与 [chassis](./chassis.md) 通过 TF 上的底盘运动耦合，大量使用 [transform](./transform.md) 的坐标查询。

下一站 [shooter](./shooter.md)：子弹是怎么打出去的——摩擦轮、拨弹盘、发射状态机、卡弹检测与热量限制。
