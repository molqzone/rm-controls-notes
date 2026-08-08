# rm-controls-notes

面向**零调车经验新人**的 rm-controls 文档集，按业务领域组织，按依赖顺序阅读即可建立完整心智模型。

## 阅读顺序

1. [Overview 大观](./overview.md) — 系统长什么样、分几层、每层管什么（零前置知识）
2. [Communication 通信](./communication.md) — 电脑和电机之间怎么说话：CAN、EtherCAT
3. [Hardware 硬件](./hardware.md) — 电机、编码器、Transmission、标定
4. [Control 控制基础](./control.md) — PID、串级环、前馈、保护与调参方法
5. [Transform 坐标变换](./transform.md) — TF 是什么、rm-controls 怎么处理 TF
6. [Chassis 底盘](./chassis.md) — 车是怎么动起来的：轮系、功率、FOLLOW、里程计
7. [Gimbal 云台](./gimbal.md) — 是怎么瞄准的：串级 PID、重力补偿、弹道、自瞄
8. [Shooter 发射](./shooter.md) — 子弹是怎么打出去的：摩擦轮、状态机、卡弹、热量
9. [Manual 决策层](./manual.md) — 上层怎么编排下层：事件、控制器切换、标定编排、指令发布

## 依赖关系

```
overview
  ├── communication          ← 需要 overview
  │     └── hardware         ← 需要 communication 理解通信方式
  │           ├── control    ← 需要 hardware 理解执行器与反馈
  │           └── transform  ← 需要 hardware 理解 joint
  │                 ├── chassis  ← 需要 control + transform
  │                 ├── gimbal   ← 同上
  │                 ├── shooter  ← 需要 control
  │                 └── manual   ← 需要 chassis + gimbal + shooter
```
