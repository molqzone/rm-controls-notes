# rm-controls-notes 文档结构

> **状态**：本目录的核心文档已经完成，并按 `D:\DynamicX\rm_ws` 当前实现复核。目标读者是没有调车经验的新人。

---

## 核心理念

- **按业务领域组织**，不按 controller 包名组织。
- 文档名对应读者要解决的问题：`chassis.md`、`gimbal.md`、`shooter.md`、`manual.md`。
- 从系统结构、通信和硬件基础出发，再进入控制、坐标变换和具体机构。
- 现有实现、配置快照和后续建议必须明确区分；不能把设计建议或外部固件行为写成当前源码功能。

---

## 文档清单

### 1. overview.md

**职责**：建立三层架构、机构与控制器的整体心智模型。

覆盖无下位机架构、决策层/控制层/硬件抽象层分工、底盘/云台/发射机构与常用控制器。

### 2. communication.md

**职责**：讲清电脑与设备间的 CAN、EtherCAT 链路，以及 `rm_hw` 和 `rm_ecat_hw` 的边界。

覆盖 CAN/CAN FD、EtherCAT 从站与分布式时钟、两种硬件路径的启动配置和通信数据流。

### 3. hardware.md

**职责**：讲清电机、编码器、Transmission 与标定。

覆盖 RoboMaster 电机与反馈、单圈角和机械零点、`SimpleTransmission`、`DifferentialTransmission`、`MultiActuatorTransmission`，以及 `need_calibration`、运行时 offset 和标定编排。

### 4. control.md

**职责**：讲清所有机构共享的闭环控制基础。

覆盖 PID、串级环、前馈、限幅与 anti-windup、状态切换、命令规划和可复现的调参流程。它不代替各机构文档中的具体控制器行为。

### 5. transform.md

**职责**：讲清 TF、URDF 关节树和控制器如何查询坐标变换。

覆盖坐标系树、`robot_state_controller`、IMU 姿态和控制器消费 TF。当前 `robot_state_controller` 使用标准 `tf2_ros::Buffer`；`RobotStateInterface` / `RobotStateHandle` 只是保存并转发该 Buffer 指针的硬件接口，不能因此宣称它无锁或具有硬实时保证。

### 6. chassis.md

**职责**：讲清底盘轮系、运动学、FOLLOW、里程计和功率约束。

### 7. gimbal.md

**职责**：讲清云台轴、串级控制、重力补偿、弹道解算和视觉跟踪。

### 8. shooter.md

**职责**：讲清摩擦轮、拨盘状态机、卡弹检测、射出检测和热量约束。

### 9. manual.md

**职责**：讲清 `rm_manual` 如何把输入事件、控制器切换、标定流水线、指令发布和裁判数据编排起来。

---

## 阅读依赖

```
overview.md
  ├── communication.md
  │     └── hardware.md
  │           ├── control.md
  │           └── transform.md
  │                 ├── chassis.md   （还需要 control）
  │                 └── gimbal.md    （还需要 control）
  │           └── shooter.md         （还需要 control）
  └── manual.md                       （需要 chassis / gimbal / shooter）
```

`manual.md` 同时依赖硬件和控制概念；阅读三个机构文档后再看它，能把上层的状态、指令和反馈放回具体机构中理解。

---

## 旧文档

`01-~09-` 目录中的旧文档已 deprecated。新文档不沿用其编号式组织，而按业务领域和学习依赖组织；可复用原理时仍需以当前源码、URDF 和配置为准。
