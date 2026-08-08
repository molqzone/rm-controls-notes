# 底盘：车是怎么动起来的

> **前置知识**：[overview](./overview.md)、[hardware](./hardware.md)（关节与力矩）、[control](./control.md)（速度环、前馈与限幅）、[transform](./transform.md)（TF，FOLLOW 模式和里程计要用）

底盘控制器（`chassis_controller`）claim 的是底盘轮组 joint，负责把操作手的“我要往那边走”翻译成每个轮子可执行的命令。这篇依次讲轮系运动学、速度规划、FOLLOW/小陀螺/TWIST 的边界、力矩与功率分配、安全降级以及里程计。

一个贯穿全文的关键点，先说在前面：**底盘控制器最终输出的是力矩（N·m），不是速度。** 为什么这么设计，得从比赛规则讲起。

---

## 1. 为什么最终输出是力矩

底盘控制器的形状不只由运动学决定，还受**底盘功率上限、缓冲能量和执行器饱和**约束。规则的处罚与供电策略会随赛季变化：有的版本先消耗缓冲能量，耗尽后限制或切断底盘输出。实现必须读取当季裁判数据和规则，不能把“超功率扣血/判负”写成永久不变的逻辑。

这些约束解释了为什么控制链最终要落到**力矩/电流命令**。只修改速度目标时，内层速度 PID 可能为了追踪目标继续增大力矩；在执行器命令层统一做功率分配，约束才有明确作用点：

```
如果只限速度目标：                         如果在执行器命令层分配：
目标速度下降                               先预测每个电机的电气功率
   │                                        │
速度 PID 仍可能因大误差顶到饱和            统一缩放或重分配电流/力矩
   │                                        │
实际功率未必按预期下降                     再验算总功率并执行硬限幅
```

这并不表示电气功率与力矩严格成正比；铜损、铁损、静态损耗和再生制动都必须在功率模型里单独处理，见 §4。当前常见的速度闭环骨架是：

```
带时间戳的底盘速度命令 [vx, vy, wz]
   ↓ 模式仲裁、坐标变换、斜坡规划
可实现的底盘速度目标
   ↓ 逆运动学（各轮系子类实现）
各轮目标速度 [ω₁* ... ωₙ*]
   ↓ 速度 PID + 可选力矩前馈
各轮原始力矩/电流命令
   ↓ 电气功率预算 + 执行器限幅
安全命令 → joint.setCommand(effort)
```

扩展到力控底盘时，主要输出可由整车期望力/力矩前馈产生，单轮速度环只保留较弱的 P 反馈防止超速。无论采用哪条路径，**坡度/阻力前馈要先合成，功率分配和执行器限幅放在最后**，否则补偿项会绕过安全约束。

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
  ├── OmniController                         n×3 轮速雅可比
  │     └── ActiveSuspensionController       继承 Omni，+ 悬挂腿位置 PID
  ├── SwerveController                       逐轮几何解算，每轮一个 Module
  └── BalanceController                      倒立摆 + IMU，不用运动学矩阵
```

**新增一种轮系 = 继承 `ChassisBase` + 实现那两个纯虚函数**，功率、里程计、模式、断连保护全部白拿——模板方法模式最干净的例子。下面逐个看子类怎么实现这两个函数。

### 2.1 麦克纳姆轮 / 全向底盘（OmniController）

**最常用**，步兵、英雄、哨兵基本都是它。麦克纳姆轮的轮缘上装了一圈斜 45° 的小滚轮——轮子主动转动的同时，滚轮能让它侧向自由滑动。四个这样的轮子组合起来，就能实现平面内三个自由度：前后（vx）、左右（vy）、自转（wz）。

先固定一个不会混淆方向的定义。令底盘速度

$$
\boldsymbol\xi=
\begin{bmatrix}v_x&v_y&\omega_z\end{bmatrix}^{T}
$$

第 $i$ 个轮心在底盘坐标系的位置为 $(x_i,y_i)$，轮子的**有效驱动力方向**为 $\mathbf d_i=[d_{ix},d_{iy}]^T$，半径为 $r_i$。这一轮对应的雅可比行是：

$$
J_i=\frac{1}{r_i}
\begin{bmatrix}
d_{ix}&d_{iy}&-d_{ix}y_i+d_{iy}x_i
\end{bmatrix}
$$

把 $n$ 行叠起来得到 $J\in\mathbb R^{n\times3}$。于是控制下行和里程计上行分别是：

$$
\boldsymbol\omega=J\boldsymbol\xi,qquad
\hat{\boldsymbol\xi}=J^+\boldsymbol\omega
$$

给定底盘速度时，$\boldsymbol\omega=J\boldsymbol\xi$ 已经给出唯一的轮速目标，**不需要用伪逆去“挑最小轮速”**。伪逆 $J^+$ 主要用于从冗余轮速反估底盘速度；满列秩时 $J^+J=I_3$，但一般 $JJ^+\ne I_n$，轮子打滑时结果只是最小二乘估计。

实现中矩阵变量名和存储方向可能不同，读代码时不要猜名字：只要检查维度即可。$n\times3$ 矩阵乘 $3\times1$ 底盘速度得到 $n\times1$ 轮速；反向才是 $3\times n$ 的伪逆乘轮速。配置中的滚子角必须先转换成**有效驱动力方向**，不能把滚子轴线角直接塞进公式。

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

舵轮能把轮子先转到期望方向再驱动，牵引方向灵活；代价是每个模块需要转向和驱动两台电机，机械与标定更复杂。麦轮、全向轮和舵轮在理想运动学下都能令 $v_x=v_y=0,\omega_z\ne0$，因此“零半径原地旋转”不是舵轮独有能力，差别主要在牵引利用率、滚子打滑和机械复杂度。

代码里每个舵轮通常打包成一个 `Module`，把“转向 PID + 驱动 PID + 几何参数”绑在一起：

```cpp
struct Module {
  Vec2<double> position_;                // 轮子安装位置
  double pivot_offset_, wheel_radius_;
  JointPositionController* ctrl_pivot_;  // 转向：位置 PID
  JointVelocityController* ctrl_wheel_;  // 驱动：速度 PID
};
```

逐轮 `atan2` 之后还要做三项工程处理：

1. **就近转位**。令 $e=\operatorname{wrap}(\theta_d-\theta)$。若 $|e|>\pi/2$，将目标舵角翻转 $\pi$，同时把驱动轮速取反，使转向最多走 $90^\circ$；在 $\pi/2$ 附近加迟滞，防止来回选择两条等价路径。
2. **转向误差投影**。轮子尚未对准时使用 $\omega_{w,eff}=\omega_w^*\cos e$，舵向差 $90^\circ$ 时不强行驱动。力控路径对应投影轮扭矩，不能在同一路径把速度和扭矩各乘一次造成重复衰减。
3. **零速保持**。当目标轮速接近 0 时保持上次有效舵角，不能调用 `atan2(0,0)` 后让转向轴随机跳变。

舵轮转向适合位置环→速度环→电调电流环的串级结构。先调内环，外环输出必须限制在内环可实现范围；失能或反馈丢失时清积分，恢复时令目标舵角等于当前角。

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
| 解算 | $n\times3$ 雅可比 | 逐轮几何解算 | Omni + 位置 PID | 倒立摆 + IMU |
| 理想原地旋转 | 可以 | 可以 | 可以 | 构型受限 |
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

以最常用的 Omni 为例，若按 §2.1 定义 $J$，逆运动学直接算 $\boldsymbol\omega=J\boldsymbol\xi$，正运动学用 $\hat{\boldsymbol\xi}=J^+\boldsymbol\omega$。它们只在满列秩、无打滑和模型准确时近似互逆；舵轮则是逐轮几何求目标舵角/轮速，再由各模块测量反估底盘运动。

记住这个方向：§4 的功率约束作用在逆运动学和轮组控制之后的执行器命令上，§5 的里程计使用正运动学估计底盘速度。

---

## 3. 坐标系、工作模式与速度规划

代码版本可能只暴露 `RAW/FOLLOW/TWIST` 等枚举，但应先按**行为语义**理解：FOLLOW 是“朝向追随”，小陀螺是“持续自转”，TWIST 是“有限角度往复”。三者不能混成一个模式名。

### 3.1 云台系平移命令到底盘系

操作手通常希望“摇杆前推”始终沿云台瞄准方向移动。若 $\theta_{GC}$ 是云台系相对底盘系的平面转角，则：

$$
\mathbf v_C=R(\theta_{GC})\mathbf v_G
$$

旋转方向取决于 TF 的父子定义，必须用一个“云台左转 $90^\circ$、摇杆前推”的单元测试核对，不能凭矩阵名字猜正负号。变换还要使用与命令/反馈时间一致的 TF；高转速下可用识别出的总延迟 $T_d$ 做一阶预测：

$$
\theta_{used}=\operatorname{wrap}(\theta_{GC}+T_d\omega_z)
$$

转写中的 $7\,\mathrm{ms}$ 只是某台车的示例，不能当默认参数，预测项符号也要按相对角定义实测确认。

### 3.2 FOLLOW：底盘正面追随云台

FOLLOW 的目标是把底盘正面逐渐对齐云台正面，以改善默认朝向和通过性。相对角误差必须先包到 $[-\pi,\pi)$，再交给角度环：

$$
e_f=\operatorname{wrap}(\theta_{GC}-\theta_{set}),\qquad
\Delta\omega_f=PID(e_f)
$$

零点附近应设死区或连续降低增益，避免云台与底盘相互追逐。转写出现过约 $\pm\pi/12$ 的死区，它只是队伍样例，应按间隙和带宽实测。跟随修正与操作手角速度的关系也必须明确，例如

$$
\omega_z^*=\omega_{operator}+\Delta\omega_f
$$

或由 FOLLOW 独占角速度；不能在代码里无条件覆盖而不说明策略。增益正负要从 $\dot\theta_{GC}\approx\omega_G-\omega_C$ 和本车坐标约定推导，不能照抄一个“负 $K_p$”。

### 3.3 小陀螺 / SPIN：持续主动自转

小陀螺是在平移的同时主动给底盘叠加持续角速度 $\omega_{spin}$，用来改变装甲板暴露方向。它依赖 §3.1 的云台系→底盘系平移变换，但**不要求底盘正面对齐云台**。如果 FOLLOW 仍在闭环追零，它会与持续自转互相对抗，所以模式仲裁应让 SPIN 暂停 FOLLOW 积分，并在退出时用当前相对角无扰恢复。

持续自转会同时提高轮速、转向延迟和功率需求。$\omega_{spin}$ 应经过轮速可行域和实时功率预算联合限幅，而不是先固定一个高转速再让各轮分别饱和。

### 3.4 TWIST：有限角度往复

TWIST 让底盘围绕某个相对朝向按正弦或其他轨迹往复，例如：

$$
\theta_d(t)=\theta_0+A\sin(2\pi f t)
$$

再由角度环产生 $\omega_z^*$。它和 SPIN 的区别是角度有界、平均角速度接近 0。$A$、$f$ 和角速度上限都要经过底盘稳定性、线缆和功率验证；切换时以当前角初始化 $\theta_0$，避免目标相位突跳。

### 3.5 RAW 与三级安全链

RAW 只表示“不叠加自动朝向修正”，**不等于急停**。即使在 RAW 下，命令新鲜度、速度/力矩限幅和反馈健康检查仍必须生效。最低限度的安全链是：

1. **指令超时**：控制循环仍健康但上层命令陈旧，按受控减速度回零并清除模式积分。
2. **电机反馈超时**：测量不可用于闭环，受影响轮组置安全输出、清 PID 积分并上报单轮故障。
3. **独立 watchdog**：主循环卡死或 HardFault 时由 MCU/从站复位或失能。只在一次完整控制周期和关键任务健康后喂狗，不能在固定中断里无条件喂。

复位后的默认输出必须失能，并记录复位原因。重新收到命令和反馈时先用当前状态初始化目标，再走斜坡恢复，不能直接续用故障前的旧命令。

### 3.6 RampFilter：硬规划与软规划

对标量规划状态 $x$、目标 $r$ 和周期 $\Delta t$，基础斜坡是：

$$
x_h^+=x+\operatorname{clip}(r-x,-a\Delta t,a\Delta t)
$$

同向加速用 $a_{acc}$，减速或反向用更合适的 $a_{dec}$；反向时先减到 0，再向另一方向加速。仅分别限制 $v_x,v_y$ 会在加速时改变平移方向，若方向保持重要，应限制二维加速度向量范数。

功率限制可能让实际速度长期落后于硬规划值。若仍积累规划状态，操作手松键后会出现“命令欠账”，车继续追赶旧目标。软规划的做法是：当滤波后的实速落在新目标 $r$ 与硬规划值 $x_h^+$ 之间时，把规划状态重置到实速。模式切换、失能和反馈恢复时也要重置规划器；实际速度滤波不能过重，否则软规划本身会引入明显滞后。

---

## 4. 力矩前馈与电气功率约束

### 4.1 约束的不是只有机械功率

单个电机的机械功率是带符号的 $P_{mech}=\tau\omega$：驱动时通常为正，制动回馈时可能为负。裁判/电源模块约束的则是**电气输入功率**，还包含铜损、铁损、驱动器和静态损耗。一个可用于辨识的经验模型是：

$$
\hat P(I,\omega)=
k_0+k_1I+k_2\omega+k_3I\omega+k_4I^2+k_5\omega^2
$$

$k_3I\omega$ 近似机械功率，$k_4I^2$ 是产生二次项的主要来源，$k_5\omega^2$ 可近似铁损。拟合时应使用电机侧反馈相电流、减速器前转子角速度和同步采样的电气功率，数据覆盖正反转、驱动、制动和多种负载；不同批次、温度和老化后的参数仍要复核。

把 $|\tau\omega|$ 或 `fabs(cmd * vel)` 全部算成耗电功率会丢掉再生符号。如果是为了留安全裕量而采用的保守策略，应明确写成策略，而不能说它等于裁判测量值。模型还依赖“一个控制周期内 $\omega$ 近似不变、电流环能跟随命令、轮子没有腾空”这些条件；电压饱和、过热降额或电流环失效时都要降级。

### 4.2 统一缩放为什么会得到二次方程

若把所有原始电流命令 $I_i^0$ 同乘 $\eta\in[0,1]$，代入上式并令总预测功率等于 $P_{max}$：

$$
\sum_i\hat P(\eta I_i^0,\omega_i)=P_{max}
$$

可整理成 $A\eta^2+B\eta+C=0$，其中

$$
A=\sum_i k_{4i}(I_i^0)^2
$$

$$
B=\sum_i(k_{1i}+k_{3i}\omega_i)I_i^0
$$

$$
C=\sum_i(k_{0i}+k_{2i}\omega_i+k_{5i}\omega_i^2)-P_{max}
$$

从 $[0,1]$ 中取最大的有效根，可以尽量保留原命令并维持轮间电流/扭矩比例。必须处理 $A\approx0$、负判别式、无有效根和电流饱和；限幅后再用模型验算一次总功率，无解时保守置零或进入已定义的低功率模式。

若沿用 rm-controls 中以 `effort_coeff/vel_coeff/power_offset` 表示的经验二次式，也应先聚合成

$$
\hat P(k)=Ak^2+Bk+C_0
$$

最后只做一次 $C=C_0-P_{max}$ 再求根。`power_offset` 必须说明是每台电机常量还是整车常量，不能在遍历每个轮子时重复减一次总功率上限。求根时假设本周期速度近似不变；二次项来自电流平方损耗，不是“缩小力矩后速度立刻变化”。

### 4.3 保留再生功率的逐轮分配

另一种策略先按原始命令预测每个电机功率 $P_i^0$，保持发电电机不变，只缩放耗电部分：

$$
\rho=
\frac{P_{max}-\sum_{P_i^0<0}P_i^0}
{\sum_{P_i^0>0}P_i^0}
$$

然后令耗电电机目标为 $\rho P_i^0$，把目标功率和当前 $\omega_i$ 代回单电机模型求 $I_i$。二次方程有两根时，取最接近原始 $I_i^0$ 且满足电流限幅的一根。它能利用回馈功率，但各轮电流比例会改变，可能破坏原先的整车力/力矩分配；统一电流缩放更能保持方向，具体策略应按底盘控制目标选择。

### 4.4 缓冲能量与超级电容

当裁判系统不提供足够快的瞬时功率反馈时，电机模型负责快速前馈，缓冲能量作慢反馈。概念模型是：

$$
E_{k+1}=\operatorname{sat}
\left(E_k+(P_{rule}-P_{measured})\Delta t,0,E_{max}\right)
$$

可令 $e_E=E-E_{set}$，再用带输出限幅和 anti-windup 的 PI/PID 调整预算：

$$
P_{budget}=P_{rule}+PID(e_E)
$$

能量偏低时预算自动收紧。$E_{max}$、$E_{set}$ 和处罚逻辑都必须版本化配置；缓冲处于 0 或饱和值时，不能再由 $\Delta E/\Delta t$ 唯一反推功率。

超级电容相关的 CHARGE/NORMAL/BURST 标签由决策层 `PowerLimit` 管理，见 [manual](./manual.md) §6.1。当前代码可验证的是：CHARGE 分支把本地 `power_limit` 乘以 `0.70`，NORMAL/BURST 按容量、缓冲与配置选择预算；它没有在此仓库中发送可验证的超电控制帧。因此“充电/放电由何处决定”需要对应硬件固件或协议文档，不能从这些模式名推断。裁判或电容数据陈旧时退到安全预算是应补充的策略，当前容量在线标志也没有独立超时计时器。

### 4.5 坡度与轮组阻力前馈

“俯仰超过阈值就放大后轮”只对车头朝上坡的单一姿态成立，侧坡、倒车上坡和下坡都会分错。更统一的做法是把世界系重力变换到底盘系：

$$
\mathbf g_C={}^CR_W
\begin{bmatrix}0&0&-g\end{bmatrix}^T,qquad
\mathbf F_{ff}=-m
\begin{bmatrix}g_{Cx}&g_{Cy}\end{bmatrix}^T
$$

再把期望平面力/偏航力矩一起分配到各轮。若进一步按法向载荷分配牵引力，要承认四点接触的 $N_i$ 静力学上不唯一，并满足 $|F_{t,i}|\le\mu N_i$；接触丢失或车体与坡面不平行时模型应退出。

轮组阻力也可在接近赛场材质的平地上逐轮标定：

$$
\tau_{ff,i}=\operatorname{sgn}(\omega_i^*)
\left(a_{0i}+a_{1i}|\omega_i^*|+a_{2i}|\omega_i^*|^2\right)
$$

零速附近用穿过原点的连续线性段或 `tanh` 代替硬 `sign`，高速端钳到标定区间，避免正负跳变和多项式外推。重力、阻力和加速度前馈都应在功率限制**之前**合成。

### 4.6 特殊构型的预算顺序

- **舵轮**：转向电机也属于底盘功率统计，不能只算驱动电机。可先给转向保留上限，再把实际剩余预算交给驱动轮；具体比例是队伍参数，不是规则常数。
- **平衡底盘**：先保留维持直立所需的稳定力矩，只压缩运动分量。把全部电流统一缩小可能直接破坏平衡。
- **力控底盘**：推荐顺序是“目标速度/加速度 → 期望整车力/力矩 → 坡度和阻力前馈 → 轮扭矩分配 → 弱速度反馈 → 功率重分配 → 电流命令”。这属于扩展控制策略，不应误写成所有版本 rm-controls 的既有实现。

---

## 5. 里程计

里程计走的是**正运动学**方向：读各轮实际转速 → 用 `odometry()` 合成**底盘坐标系**速度 → 旋转到 `odom` 系 → 积分成位姿 → 发布 `odom→base_link` 的 TF。若当前航向为 $\theta$：

$$
\dot x=\cos\theta\,v_x^C-\sin\theta\,v_y^C
$$

$$
\dot y=\sin\theta\,v_x^C+\cos\theta\,v_y^C,qquad
\dot\theta=\omega_z^C
$$

直接把底盘系 $v_x^C,v_y^C$ 累加到世界位置，会在底盘转向后沿错误方向积分。

```cpp
void ChassisBase::updateOdom(...) {
  geometry_msgs::Twist v_c = odometry();    // 各轮实速 → 底盘系速度
  Vec2 v_odom = rotate2d(yaw, {v_c.linear.x, v_c.linear.y});
  position += v_odom * dt;
  yaw = wrap(yaw + v_c.angular.z * dt);
  robot_state_handle_.setTransform(odom2base_, "rm_chassis_controllers");  // 发 TF
}
```

它发布的是**航迹推算（Dead Reckoning）**，不是绝对定位。轮径误差、轮组几何误差和打滑都会累积；高动态时还要保证轮速与航向时间对齐。外部视觉/激光定位可通过 `outsideOdomCallback` 等路径做校正，但应由明确的融合器处理协方差和跳变，不能把外部位姿无条件硬覆盖。这条 `odom→base_link` TF 正是 [transform](./transform.md) 中 `chassis_controller` 贡献的那条边。

---

## 6. 标定

底盘要不要标定，取决于关节是不是需要**绝对角度**——这正是 [hardware](./hardware.md) 第 2 节讲的那条判据。

**大多数底盘驱动轮不用机械找零。** 普通麦轮、全向轮和舵轮的驱动电机只做速度控制，不关心机构零点（[hardware](./hardware.md) §2）。所以这些驱动轮无需进入 `chassis_calibration` 流水线；但方向、轮径、反馈新鲜度仍必须检查。

**需要标定的是位置控制关节**，主要出现在特殊底盘上：

- 轮腿 / 平衡机器人的**髋关节（hip）**——要知道腿的绝对角度才能算平衡
- 主动悬挂的**悬挂腿**——要知道悬挂高度
- 舵轮的**转向关节**（若无法直接恢复机械零点）——要知道轮子朝向的绝对零点

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
  timeout: 0.1                 # 上层命令超时示例；反馈超时由硬件层另管
  pid_follow: { p: 10.0, i: 0, d: 0.01, ... }   # FOLLOW 模式的跟随 PID
  power:                       # 功率限制系数（支持 dynamic_reconfigure）
    effort_coeff: 1.90
    vel_coeff: 0.0088
    power_offset: -9.8
  wheels:                      # 每个轮子一段
    left_front:
      pose: [0.2018, 0.172, 0.0]   # x y z 安装位置
      roller_angle: -0.785          # 原配置角定义；必须核对如何换成有效驱动力方向
      joint: left_front_wheel_joint # URDF 关节名（要和 hardware 配置一致）
      pid: { p: 0.25, i: 0, d: 0.0, ... }   # 该轮速度 PID
```

常见调整：

- **换轮系** → 改 `type`（`OmniController` / `SwerveController` / `ActiveSuspensionController` / `BalanceController`），并按该子类要求补齐 `wheels` 或悬挂/平衡参数
- **辨识功率模型** → `power` 三系数描述现有经验模型，先用同步采集数据离线辨识，再用 `dynamic_reconfigure` 小范围验证；不能靠比赛中盲调掩盖单位/符号错误
- **调跟随手感** → 改 `pid_follow`
- **调最大速度** → 不在这里，在决策层 `rm_manual` 的 `vel.max_linear_x` 分段映射里（功率越高限速越高），见 [manual](./manual.md)

配置中的数值只是某台车的示例。至少要随车记录轮半径、轮位姿、坐标方向、控制周期、功率模型版本和辨识数据集；换轮胎、减速器、电机固件或电容后重新验证。

---

## 8. 小结

- 底盘控制链最终输出力矩/电流命令，功率分配和执行器限幅必须在所有前馈与反馈合成之后统一执行。
- 对 $J\in\mathbb R^{n\times3}$，逆运动学是 $\boldsymbol\omega=J\boldsymbol\xi$，轮速里程计是 $\hat{\boldsymbol\xi}=J^+\boldsymbol\omega$；伪逆主要用在反估底盘速度。
- 轮系包括 Omni（麦轮/全向）、Swerve（舵轮）、ActiveSuspension 和 Balance。前三者理想上都能原地旋转；舵轮还要做就近转位、轮速反向和 $\cos e$ 投影。
- **FOLLOW、SPIN 和 TWIST 是三种不同语义**：朝向追随、持续自转、有限角度往复；平移命令需按同一时刻的云台/底盘相对角变换。
- `RampFilter` 要区分加速、减速与反向，结合实速做软重置，避免功率受限后积累“命令欠账”。
- 功率约束对象是电气输入功率。统一电流缩放能保持轮间扭矩比例，逐轮功率分配能利用再生；两者都要处理模型失效、求根边界和限幅后复算。
- 坡度补偿应从重力向量变换和整车力分配出发，而不是固定放大“后轮”；重力/阻力前馈必须先于功率限制。
- 里程计先把底盘系速度旋转到 `odom` 系再积分，仍会受轮径误差和打滑影响，外部定位需经明确的融合逻辑校正。
- **标定**：普通底盘轮只控速、不标定；只有轮腿髋关节、悬挂等位置关节要撞限位标定，且因共享关节需和 `leg_controller` 一起停。

下一站 [gimbal](./gimbal.md)：云台怎么瞄准，串级 PID、重力补偿、弹道解算和自瞄。
