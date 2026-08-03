# rm-controls-notes 文档规划草案

> **背景**：`01-~09-` 目录下的旧文档已 deprecated。`rm-controls-notes/` 是从零开始重写的文档集，目标读者是**完全没有调车经验的新人**。

---

## 核心理念

- **按业务领域组织**，不按软件组件（controller）组织
- 文档名 = 业务领域：`chassis.md`、`gimbal.md`、`shooter.md`
- **每篇文档回答一组递进的问题**，新人按顺序读就能建立起完整的心智模型
- **不假设读者有任何前置知识**
- **依赖关系单向**：overview → communication → hardware → transform → domain docs

---

## 建议的文档清单

### 1. overview.md ✅（已有，需确认是否适配零基础）

**职责**：新人读完后能回答"这套系统长什么样、分几层、每层管什么"。

覆盖：

- 什么是无下位机（不用 MCU 写控制算法）
- 系统三层架构的轮廓（决策层/控制层/硬件抽象层）
- **从机械机构到控制器**：机器人分几个机构（底盘/云台/发射）→ 每个机构对应一个控制器 → 控制器之间互不干扰
- 9 个控制器一句话简介
- 硬件抽象层做了什么（和硬件通信、统一接口）

**不覆盖**：

- Transmission、offset、标定（放 hardware.md）
- 各业务领域的算法细节（放 chassis.md / gimbal.md / shooter.md）

---

### 2. communication.md（待写）

**职责**：讲清楚"电脑和电机之间怎么通信"。

覆盖：

- 为什么需要通信协议（电脑不是直接连电机的）
- CAN 总线：历史、特点、CAN ID、CAN FD
- EtherCAT：什么是 EtherCAT、从站拓扑、分布式时钟
- rm-controls 为什么从 CAN 迁移到 EtherCAT（带宽、稳定性）
- 通信链路全景：上位机 → 网卡 → EtherCAT 从站板 → 电机
- 硬件抽象层如何封装通信差异（rm_hw vs rm_ecat_hw）

**不覆盖**：
- 具体电机的控制协议（放 hardware.md）
- Transmission 和标定（放 hardware.md）

**前置知识**：overview

---

### 3. hardware.md（待写）

**职责**：硬件相关的所有内容——电机、传感器、Transmission、标定，一篇讲完。

覆盖（按阅读顺序）：

1. **RoboMaster 常用电机**：3508/6020/2006/达妙，力矩/速度/位置控制模式
2. **编码器**：增量式 vs 绝对式，为什么增量式每次上电不知道自己在哪
3. **Transmission**：为什么需要它（关节弧度 ↔ 电机值的翻译层）
   - SimpleTransmission：减速比、offset、负减速比=反装
   - DifferentialTransmission：差速器
   - DualActuatorTransmission：双电机带一个关节
5. **标定**（紧接 offset 讲）：
   - 为什么需要标定（承接编码器部分）
   - 标定的本质：找零 → 算 offset → 存 offset → 标记完成
   - 状态流转：needCalibration → calibrating → calibrated
   - 三种标定方式：撞限位、读 Hall、差动
   - 标定时硬件层跳过限幅、自动应用 offset
   - 标定编排：多个机构按顺序标，一个标完再标下一个
   - **calibration_controller 的实现**：如何继承 MultiInterfaceController、如何 claim/release joint、如何检测堵转
   - 为什么标定时其他机构还能动（引用 overview 的独立启停概念）
6. **其他硬件**（按需）：IMU、裁判系统、GPIO、ToF 雷达

**不覆盖**：
- 底盘运动学、云台 PID、发射状态机（放各自的 domain doc）
- TF 相关（放 transform.md）

**前置知识**：overview 中的"硬件抽象层"概念

---

### 4. transform.md（待写）

**职责**：讲清楚 TF 是什么、rm-controls 怎么处理 TF。

覆盖：
- TF 是什么（给零基础新人的最小介绍：坐标系树、父子变换）
- URDF 关节树怎么变成 TF 发布出来——`robot_state_controller` 的职责
- IMU 数据怎么变成 TF——`orientation_controller` 的职责
- rm-controls 为什么不用标准 `tf2_ros::Buffer`：实时线程安全问题 → `RobotStateInterface` 的设计
- 其他控制器怎么消费 TF（查坐标系变换）
- 配置项说明

**不覆盖**：
- 坐标变换的数学原理（旋转矩阵/四元数，需要的话单独讲）

**前置知识**：overview（理解三层架构）+ hardware.md（理解 joint）

---

### 5. chassis.md（待写）

**职责**：讲清楚"底盘是怎么动起来的"。

覆盖：
- 机器人有哪些轮系：麦轮、舵轮、全向轮
- 每种轮系的运动学解算（轮子转速 ↔ 底盘速度）
- 功率限制：裁判系统限制总功率，控制器怎么分配
- FOLLOW 模式：底盘自动跟随云台方向
- 里程计
- 配置项说明

**前置知识**：overview + hardware.md

---

### 6. gimbal.md（待写）

**职责**：讲清楚"云台是怎么瞄准的"。

覆盖：
- yaw/pitch 双轴结构
- 串级 PID（角度环 → 速度环 → 力矩）
- 重力补偿
- 弹道解算（子弹下落补偿）
- 自瞄模式：接收视觉数据 → 云台跟踪目标
- 底盘前馈：底盘运动时云台保持指向
- 配置项说明

**前置知识**：overview + hardware.md

---

### 7. shooter.md（待写）

**职责**：讲清楚"子弹是怎么打出去的"。

覆盖：
- 发射机构组成：摩擦轮 + 拨弹盘
- 发射状态机：STOP → READY → PUSH → BLOCK
- 摩擦轮转速控制
- 卡弹检测
- 热量限制（裁判系统）
- 配置项说明

**前置知识**：overview + hardware.md

---

### 8. manual.md（待写）

**职责**：讲清楚"上层怎么编排下层"。

覆盖：
- rm_manual 的定位：决策层，不参与 1kHz 控制环，跑在 100Hz
- 遥控器/键盘输入怎么变成事件：`InputEvent`、上升沿/下降沿
- 怎么切换控制器：`ControllerManager` 的 state/main/calibration 三类生命周期
- 标定流水线：`CalibrationQueue` 编排标定步骤
- 指令发布：`CommandSender` 构建并发布控制指令
- 兵种差异如何体现：Manual 继承体系（ChassisGimbalManual / EngineerManual / DroneManual……）
- 裁判系统交互：功率/热量管理
- 配置项说明

**不覆盖**：
- 各 Manual 子类的代码细节

**前置知识**：overview + hardware.md + chassis.md / gimbal.md / shooter.md

---

## 依赖关系

```
overview.md                ← 新人第一站，零前置知识
    │
    ├── communication.md   ← 需要 overview
    │
    ├── hardware.md        ← 需要 communication.md 理解通信方式
    │                        需要 overview 的"硬件抽象层"概念
    │
    ├── transform.md       ← 需要 hardware.md 理解 joint
    │
    ├── chassis.md         ← 需要 hardware.md + transform.md
    ├── gimbal.md           ← 同上
    ├── shooter.md         ← 同上
    │
    └── manual.md          ← 需要 chassis.md + gimbal.md + shooter.md
                              理解下层再讲上层编排
```

---

## 和旧文档（01-~09-）的关系

全部 deprecated。新的 `rm-controls-notes/` 逐步覆盖旧文档的内容，但：
- **组织形式不同**：旧文档按目录编号分（代码框架/EtherCAT/电机/控制理论…），新文档按业务领域分（底盘/云台/发射）
- **深度不同**：旧文档偏操作参考，新文档偏原理
- **部分内容可以直接搬运**：`software_framework.md` 中的 Transmission 三种类型、`why_rm-controls.md` 中的理念
