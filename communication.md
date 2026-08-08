# 电脑和电机之间怎么说话

> **前置知识**：[overview](./overview.md)（理解三层架构，尤其是硬件抽象层）

在 overview 里我们说过，rm-controls 是**无下位机**架构：控制算法全部跑在上位机（NUC 之类的小电脑）里，下位机退化成一个"透明转发层"。这句话听起来很干净，但它藏了一个问题——**电脑并没有一根线直接插在电机上**。电脑算出"这个轮子该出 2 牛·米的力矩"，这个数字要经过好几道手才能变成电机里真实流动的电流。这篇文档就讲清楚这中间发生了什么。

---

## 1. 为什么需要通信协议

先想一个最朴素的问题：上位机是一台普通电脑，它身上只有 USB、网口、HDMI 这些接口；而 RoboMaster 的电机（3508、6020 这些）接受的是 **CAN 报文**——一种和电脑原生接口完全不同的信号。两者语言不通。

所以中间必须有：

1. **一套约定好的数据格式**（协议）：多少个字节、哪几位表示力矩、哪几位表示电机 ID
2. **一个翻译/转发的硬件**（下位机 / 从站板）：把电脑发来的信号转成电机认识的 CAN 报文，再把电机的反馈转回去
3. **一条物理链路**：网线或者 CAN 线，负责真正把电信号传过去

通信协议就是第 1 件事——**在电脑和电机之间约定一套双方都遵守的"话术"**。没有它，电脑发出的每一个 bit 对电机来说都是噪声。

rm-controls 历史上用过两套方案：早期直接用 **CAN 总线**（对应 `rm_hw` 包），现在用 **EtherCAT**（对应 `rm_ecat_hw` 包）。要理解为什么迁移，得先分别看看这两种是什么。

---

## 2. CAN 总线：机器人里的老资格

### 2.1 它是什么

CAN（Controller Area Network）是 Bosch 在 1980 年代为汽车设计的一种总线协议。汽车里有几十个电子控制单元（发动机、刹车、车窗……），它们需要一条便宜、抗干扰、可靠的线互相通信——CAN 就是为这个场景生的。RoboMaster 的电调（C620、GM6020 内置电调等）原生就说 CAN，所以整个生态天然建立在 CAN 之上。

CAN 的几个关键特点：

- **总线型拓扑**：所有设备挂在同一对差分线（CAN_H / CAN_L）上，像串在一根绳上的一串灯，而不是一个设备一根线
- **广播 + 仲裁**：没有点对点地址阶段，每条报文带一个 **CAN 仲裁 ID**。所有设备都能听到报文并按 ID 过滤。总线空闲、多个节点同时开始发送时，数值更小的 ID 赢得本次仲裁；它仍要等待已经在发送的帧。仲裁失败的节点自动等待重试，不等于报文损坏；持续过载时，低优先级 ID 可能长期饥饿
- **报文短小**：经典 CAN 2.0 每条报文的数据段**最多 8 字节**，波特率通常 1 Mbit/s

### 2.2 CAN ID：电机的"门牌号"

在 RoboMaster 的用法里要区分三种“ID”：

- **逻辑电机 ID**：电调拨码或配置中的 1、2、3……，表示组内第几个电机；
- **反馈帧仲裁 ID**：例如某类电机逻辑 ID 1 对应 `0x201`，另一类可能对应 `0x205`；
- **命令帧仲裁 ID**：一帧常同时携带 4 个电机的目标值，ID 由电机类型和分组规则决定。

所以“两个电机都配置成逻辑 ID 1”不必然产生同一个仲裁 ID，“逻辑 ID 不同”也不保证跨协议绝不冲突。配置和调试时必须查对应固件协议，核对**总线 + 实际命令 ID + 实际反馈 ID + 分组槽位**，不能只看一个数字。

> 具体每种电机的报文格式、力矩怎么编码成整数，属于**电机控制协议**，放在 [hardware](./hardware.md) 里讲。这里只需要记住：总线上真正参与过滤和仲裁的是帧的仲裁 ID，逻辑电机 ID 只是生成它与选择分组槽位的协议参数。

### 2.3 CAN FD

CAN FD（Flexible Data-rate）是 CAN 的升级版，把单条报文的数据段从 8 字节扩到最多 64 字节，数据段还能临时切到更高的波特率。它缓解了"报文太短"的痛点，但没有改变 CAN 总线型、单总线带宽有限的本质。rm-controls 的电机生态仍以经典 CAN 为主。FD-CAN 控制器可以配置成经典 CAN 模式连接 RM 电调，但经典 CAN 节点不能解析 CAN FD 或 BRS 帧；“MCU 支持 FDCAN”不代表可以直接在现有电机总线上发送 FD 帧。

### 2.4 CAN 方案的三个痛点

`rm_hw` 用单线程把 CAN 方案跑通了，逻辑非常直白——一个 1kHz 的循环里顺序做四件事：

```
ROS control 循环（1kHz）
  read()               ← 从 SocketCAN 读电机反馈
  controllerMgr::update()  ← 各控制器算命令
  write()              ← 把命令发到 SocketCAN
        │
        ▼
  SocketCAN（Linux 内核）→ CAN 总线（1Mbit/s）→ 电调 → 电机
```

简单是它最大的优点，但一台完整的机器人跑起来就暴露出三个问题：

1. **带宽有限**。CAN 2.0 只有 1 Mbit/s、每帧 8 字节。一台车要控多个电机，再加上 IMU、GPIO、遥控器数据，总线利用率很快到达极限。
2. **没有同步机制**。读和写都在一个线程里串行完成，各个电机的反馈时刻、命令时刻天然是错开的、异步的，谈不上"同一时刻的快照"。
3. **扩展性差**。常见 RM 电机协议给出若干逻辑 ID 和分组槽位，但“最多编址 8 个”只是协议上限，不是带宽承诺。要提高刷新率或继续加设备，通常得拆分 CAN 总线。

总线负载应先做预算：

$$
U\approx\frac{\sum_i f_i B_i}{R_{bit}}
$$

$f_i$ 是第 $i$ 类帧频率，$B_i$ 是包含仲裁、控制、CRC、ACK、帧间隔和位填充后的总位数。经典 CAN、11 位 ID、8 字节数据帧在**未计位填充**时就约 111 bit。若 8 个电机各以 1kHz 回传，再以 1kHz 发两帧 4 电机分组命令，固定长度已经约为：

$$
(8+2)\times1000\times111=1.11\ \text{Mbit/s}
$$

这在 1M CAN 上物理不可行，还没给重传和突发留余量。实际方案必须降低反馈/命令率、减少设备或拆分总线，并把目标利用率留在 100% 以下足够远的位置。

物理层也要守规矩：使用线性主干、只在总线两端各接一个 $120\ \Omega$ 终端，避免星形拓扑和长支线。断电后测 CAN_H 与 CAN_L，两个 120Ω 并联应接近 60Ω；明显偏离时先修接线和终端，再查软件协议。

---

## 3. EtherCAT：把总线搬到网线上

### 3.1 它是什么

EtherCAT（Ethernet for Control Automation Technology）是 Beckhoff 提出的一种**工业实时以太网**协议。常规 EtherCAT 链路使用 100BASE-TX（100 Mbit/s）和普通双绞线；主机可以是千兆网卡，但 EtherCAT 端口协商/工作在 100 Mbit/s。它的工作方式和普通 TCP/IP 以太网很不一样：

- **主站—从站结构**：上位机是唯一的**主站**（master），所有设备是**从站**（slave）。主站发一帧数据，这帧数据像地铁一样依次穿过每一个从站
- **"飞速处理"（on the fly）**：每个从站在这帧数据经过自己的一瞬间，就地读走属于自己的部分、写入自己的反馈，几乎不产生延迟。一帧数据跑一圈回来，所有从站的命令都发下去了、所有反馈也都收上来了
- **分布式时钟（Distributed Clock, DC）**：EtherCAT 有一套让从站本地时钟对齐的机制，可把从站同步误差压得很小。它提供的是统一时间基准；板后 CAN 电机是否同步采样，还取决于从站固件有没有按该时钟触发采样、缓存并携带时间戳

### 3.2 从站拓扑

在 rm-controls 里，EtherCAT 从站是一块**转接板**：它一头是网口（连上位机），另一头是 CAN 控制器（连电机）。也就是说——

> **CAN 并没有消失，而是被"下沉"到了从站内部。** 电机和电调之间仍然说 CAN，只不过这段 CAN 变得很短、就在从站板上；从站板对外则统一用 EtherCAT 和上位机通信。

一块从站板通常带两路 CAN（CAN0 + CAN1），协议编址上可接约 16 个电机，但每路实际刷新率仍受 §2.4 的 CAN 带宽预算限制。电机命令、IMU、遥控器（DBus）和 GPIO 可装进同一组 EtherCAT PDO 一起收发，主站侧因此得到统一周期的数据；从站板内部仍要决定各 CAN 帧的发送、采样和时间戳策略。

### 3.3 分布式时钟解决了什么

回想 CAN 的痛点之二——没有跨设备时间基准。EtherCAT DC 让各从站能按约定时刻触发采样和输出，闭环时序因此更容易定义。但 DC **不会自动保证**从站后面的 CAN 电机反馈就是同一时刻快照：只有从站固件同步触发/缓存，或每份反馈带可比较时间戳时，主站才能这样解释。没有这些保证时，PDO 只代表“本周期收集到的最新一组值”。

---

## 4. 为什么从 CAN 迁移到 EtherCAT

把上面的对比拉成一张表，迁移的动机就一目了然：

| 维度 | CAN 方案（rm_hw） | EtherCAT 方案（rm_ecat_hw） |
| --- | --- | --- |
| 线程数 | 1 | 3（通信、控制、降频发布） |
| 通信介质 | CAN 总线（1 Mbit/s） | EtherCAT / 100BASE-TX（100 Mbit/s） |
| 协议编址能力（刷新率另算） | 常见 8 个 / CAN | 约 16 个 / 从站（CAN0 + CAN1） |
| 每周期数据量 | ~30 条 CAN 帧 | 1 帧 EtherCAT（约 100 字节） |
| 同步性 | 无跨节点公共时基 | DC 对齐从站时钟；板后设备是否同步取决于从站实现 |
| 反馈内容 | 只有电机位置 / 速度 / 电流 | 电机 + IMU + GPIO + 遥控器同在一帧 |
| 扩展方式 | 加 CAN 转接卡 | 加 EtherCAT 从站 |

一句话：**EtherCAT 用大得多的带宽、统一的从站时基和一组 PDO 承载多类外设，改善了 CAN 的带宽、跨板同步和扩展问题。** 板后 CAN 的刷新与同步仍需从站固件保证。本文后续以当前默认的 `rm_ecat_hw` 启动路径为主；工作区也保留 `rm_hw` / `rm_can_hw.launch` 的兼容路径，不能把两套配置当成同一条链路。

不过 EtherCAT 也不是白拿好处——它引入了一个新矛盾：**ROS 的分配、锁和发布延迟不能进入 EtherCAT 收发关键路径**。这就是下一节线程隔离设计的由来。

---

## 5. 线程隔离：保护 EtherCAT 收发时序

### 5.1 矛盾

EtherCAT 主站（rm-controls 用的是 SOEM，Simple Open EtherCAT Master 库）要求"发一帧 → 等接收 → 解析反馈 → 组装下一帧"需要在**固定的 1kHz 周期**内完成。但如果把控制计算也塞进这个周期，问题就来了：问题不在控制计算本身（算一个 PID 其实只要几微秒），而在**ROS 操作不可预测**。

`controllerManager::update()` 和 `write()` 所在的控制路径里，仍可能出现三类不适合塞进 EtherCAT 收发截止时间的工作：

1. **ROS 消息的内存分配与序列化**：`write()` 里会 `publish()` GPIO 状态、总线状态等 ROS 消息。尽管用了 `ThreadedPublisher` 把网络发送卸到后台，但 `publish()` 本身仍涉及消息内存分配、序列化、加锁写入 publisher 队列——都可能有几十到几百微秒的突发延迟。
2. **控制器插件的非确定性工作**：`controllerManager_->update()` 本身不会执行 load/unload 服务回调；这些回调由 ROS spinner 线程处理，控制器切换在后续 update 边界生效。但它会调用所有活跃 controller 的 `update()`，而插件仍可能发布调试话题、访问参数或做其他非确定性工作。
3. **动态调度不可控**：`std::thread` 和 ROS 的线程池共享 CPU 时间。即使 `SCHED_FIFO` 设了高优先级，一次 page fault（ROS 消息首次分配）、一次系统调用（`ros::Time::now()`）、一次内核抢占——都可能在毫秒级路径上插入不可控的延迟。

如果像 CAN 方案那样塞进一个线程里顺序执行：

```
readAllBuses() → updateProcessReadings() → controllerManager::update() → updateSendStagedCommands() → writeToAllBuses()
│                    ← 控制器/ROS 工作可能抖动 ──→              │              │
└────────── 过程数据收发被拖迟 ──────────────────→       周期抖动、监测告警或（取决于超时配置）总线状态变化
```

关键是：**这不是"控制器算太久"的问题**——控制器本身的纯数学运算通常很短，问题是控制路径的最坏耗时不受同样的上界约束。EtherCAT 的过程数据周期仍要满足设备和主站配置的时限；错过周期会增加抖动、丢过程数据或触发监测恢复，但不会必然在一次迟到后立刻掉线。

所以矛盾的本质是：**CAN 和 EtherCAT 都有时序约束，但 EtherCAT 的过程数据周期、从站状态和恢复策略让通信关键路径更需要可预测的最坏耗时。** 允许的抖动不是固定的“几十微秒”，应以当前周期、从站和 watchdog 配置实测为准。

### 5.2 解法：两条关键线程 + 一条降频发布线程

解法不是"让控制器算快点"——它本来就够快。解法是把**一段代码拆成两条路径，把 ROS 关在 EtherCAT 的门外**。

先看直接参与控制的两条关键线程，它们通过双缓冲隔离：

```
EtherCAT 线程（updateWorker，目标 1kHz）       控制线程（controlWorker，有独立时限）
───────────────────────────────              ────────────────────────
只做最干净的四步，不同步发布 ROS：               执行 ROS control 与命令 staging：
1. readAllBuses()（SOEM 收帧）                1. read()（从 reading_ 拷反馈，纯 double 赋值）
2. updateProcessReadings()（解析 TxPDO）      2. controllerManager::update()（可能含 ROS）
3. updateSendStagedCommands()（组装 RxPDO）   3. write()（propagate + 限幅 + stageCommand，可能 publish）
4. writeToAllBuses()（SOEM 发帧）
         ▲                              ▲
         │        双缓冲队列（加锁）        │
         │    ┌──────────────────┐        │
         ├────┤  reading_（反馈）  ├────────┤
         │    │  ← EtherCAT 写入  │  控制线 │
         │    │  → 控制线程读取    │  程读取 │
         │    ├──────────────────┤        │
         │    │ stagedCommand_    │        │
         │    │  ← 控制线程写入    │  控制线 │
         └────┤  → EtherCAT 读取  ├────────┘
              └──────────────────┘
```

- **EtherCAT 线程**只做数据搬运与 SOEM 收发，不同步做 ROS 发布和日志 I/O，目标是让最坏耗时稳定在 1ms 截止时间内。
- **控制线程**管 ROS control 的 read-update-write 流程。它的抖动不会直接拖迟 EtherCAT 帧，但仍必须监测 deadline；卡住时，通信线程只会重复旧命令。
- **publishWorker** 以约 100Hz 从快照发布 readings、IMU、joint state 和诊断，避免序列化、网络发送进入上面两条关键路径。

两个线程通过每块从站内部的两块加锁缓冲区交接数据（`stagedCommandMutex_` / `readingMutex_`，见 `RmEcatSlave.h`）。

代价是引入**最多约一个周期的交接延迟**（若两个线程相位近似均匀，平均约半个周期），还要处理旧命令。命令应带时间戳或递增序号；超过新鲜度阈值后，通信/硬件层必须进入预定义的清零、失能或受控保持状态，恢复时从当前关节状态平滑接回，不能永远重放 `stagedCommand_`。

---

## 6. 通信链路全景

把一条命令从控制器一路走到电机、再把反馈走回来，全景是这样的：

```
上位机（NUC）
  控制器 update() 算出关节力矩
        │  hardware_interface（C++ 指针直通，见 overview 控制层↔硬件层）
        ▼
  rm_ecat_hw
    ├─ Transmission：关节力矩 → 电机力矩（换算，详见 hardware.md）
    ├─ 限幅
    └─ 力矩(N·m) → 整数 raw，存入命令缓冲
        │
        ▼  EtherCAT 线程按 1kHz 组帧
  网卡（常见为千兆网卡，EtherCAT 链路工作在 100BASE-TX）
        │  一帧 EtherCAT，网线
        ▼
  EtherCAT 从站板
    ├─ 解析出属于各电机的命令
    └─ 通过板载 CAN 控制器发 CAN 报文
        │  短程 CAN
        ▼
  电调（C620 等）→ 电机
```

反馈走的是同一条路，方向相反：

```
电机编码器 / 电流采样
  → 电调 → CAN 报文 → 从站板组装进 EtherCAT 帧
  → 网线 → 网卡 → EtherCAT 线程写入反馈缓冲
  → 控制线程 read() 取出 → Transmission 换算回关节值
  → 控制器 getPosition()/getVelocity() 读到，用于闭环
```

注意全景里出现了 **Transmission（传动换算）** 和**限幅**——它们是硬件抽象层的另一半职责，这里只是标出它们在链路中的位置，具体机制留给 [hardware](./hardware.md)。

---

## 7. 硬件抽象层怎么封装通信差异

overview 里讲过，三层架构的意图是"每层只关心自己的事"。通信方式的差异——是走 CAN 还是走 EtherCAT——就被**完全封在硬件抽象层里**，上面的控制层和决策层一无所知。

这是怎么做到的？关键在于 ROS control 提供的一个抽象基类 `RobotHW`：它规定了任何硬件都要实现 `read()`（把硬件反馈读进内存）和 `write()`（把内存里的命令发给硬件）两个方法。控制器永远只跟内存里那块共享数据打交道，从不关心数据是怎么来、怎么走的。

于是 rm-controls 提供了两个 `RobotHW` 的实现：

| 包 | 通信方式 | 线程模型 |
| --- | --- | --- |
| `rm_hw` | CAN 总线（SocketCAN） | 单线程 |
| `rm_ecat_hw` | EtherCAT（SOEM） | 两条控制关键线程 + 一条降频发布线程 |

对控制器来说，换掉哪一个都一样——它写的还是 `joint.setCommand(effort)`，读的还是 `joint.getPosition()`。通信介质从 CAN 换成 EtherCAT，是把 `rm_hw` 这个插件换成 `rm_ecat_hw`，控制层一行代码都不用改。这正是 overview 里说的"换一个通信方式不影响控制层"在通信这一层的具体兑现。

`rm_ecat_hw` 内部为了做好这层封装，落到代码是**四层类**层层委托完成的——每一层解决一个特定问题、只跟相邻层打交道。理解这几个类的分工，是读懂（或改动）硬件层的前提。自上而下四层：

```
┌── 第1层：ROS control 标准接口 ─────────────────────────────┐
│  controller::update() → joint.setCommand(effort)          │
│  → Transmission 传播 → EffortActuatorInterface            │
│  不依赖任何硬件细节                                        │
└──────────────────────┬─────────────────────────────────────┘
                       ▼
┌── 第2层：RmEcatHardwareInterface（继承 RobotHW）───────────┐
│  桥接 ROS control 标准 ↔ EtherCAT 从站管理                 │
│  read() / write() / setupActuators() / setupTransmission() │
└──────────────────────┬─────────────────────────────────────┘
                       ▼
┌── 第3层：SlaveManager + Slave（从站管理）──────────────────┐
│  管理各从站的 PDO 收发、力矩单位换算、线程安全缓冲          │
│  rmStandardSlaveManager / RmEcatStandardSlave / MitSlave   │
└──────────────────────┬─────────────────────────────────────┘
                       ▼
┌── 第4层：EcatBusManager（SOEM 之上的总线管理）─────────────┐
│  通过网卡真正收发 EtherCAT 帧、监控总线状态                │
│  startupCommunication() / readAllBuses() / writeToAllBuses()│
└─────────────────────────────────────────────────────────────┘
```

### 7.1 第2层 RmEcatHardwareInterface —— 桥

它是 `RobotHW` 的实现，是控制层唯一直接接触的类。职责是把 ROS control 那套标准（接口、Transmission、限位）和下面的从站管理**对接起来**：

| 方法 | 干什么 |
| --- | --- |
| `setupActuators()` | 创建每个电机的 `ActData`，把它的指针注册进各硬件接口 |
| `setupTransmission()` | 从 URDF 加载传动比（见 [hardware](./hardware.md)） |
| `setupJointLimit()` | 从 URDF 加载关节限位 |
| `setupImus()` / `setupGpios()` | 注册 IMU / GPIO 接口 |
| `read()` | 从从站读反馈 → 写进 `ActData` |
| `write()` | 从 `ActData` 读命令 → 交给从站 |

**关键设计是 `ActData` 的指针被注册到各接口**——控制器通过接口读写的，其实就是这块 `ActData` 内存，零拷贝。这正是 overview 说的"控制层↔硬件层走 C++ 指针直通"的落点。

### 7.2 第3层 SlaveManager + Slave —— 从站管理

这一层管"从站的 PDO（过程数据对象）怎么收发"。它区分了两个概念：

- **SlaveManager**（`rmStandardSlaveManager`）：管一批从站的整体收发与线程安全缓冲
- **Slave**：单个从站的抽象，负责把自己那块 PDO 解析/组装

SlaveManager 的主要方法：

| 方法 | 干什么 |
| --- | --- |
| `updateProcessReadings()` | 遍历各从站 `slave->updateRead()`，解析 TxPdo（反馈） |
| `updateSendStagedCommands()` | 遍历各从站 `slave->updateWrite()`，组装 RxPdo（命令） |
| `stageMotorCommands(cmds)` | 线程安全地把命令存进 `stagedCommand_` 缓冲 |
| `getMotorPositions()` 等 | 读取解析后的反馈 |

**力矩的单位换算就发生在这一层**：控制器算的是 `double`（N·m），但电机命令是 `int16_t` 整数。下行 `double → int16_t`（乘 `torqueFactorNmToInteger` 再 clamp），上行 `int16_t → double`（除回去）。

Slave 按电机协议分两种：

| Slave 类型 | 协议 | 控制方式 | 典型电机 |
| --- | --- | --- | --- |
| `RmEcatStandardSlave` | RM 协议 | 力矩控制 | 3508 / 6020 / 2006 |
| `RmEcatMitSlave` | MIT 协议 | 位置 / 速度 / 力矩 | 达妙（DM） |

### 7.3 第4层 EcatBusManager —— 总线

最底层，直接坐在 SOEM（Simple Open EtherCAT Master 库）之上，负责通过网卡真正收发帧、监控总线：

| 方法 | 干什么 |
| --- | --- |
| `startupCommunication()` | 初始化网卡、枚举从站 |
| `readAllBuses()` | `ec_send_processdata()` 发请求、`ec_receive_processdata()` 收响应 |
| `writeToAllBuses()` | `ec_send_processdata()` 发命令 |
| `busMonitoring()` | 监测总线状态 |
| `onActivate()` / `onDeactivate()` | 总线恢复 / 隔离 |

它只管 EtherCAT 帧的物理收发和总线健康，完全不知道上面跑的是电机、IMU 还是 GPIO——业务无关。

### 7.4 三个 worker 线程怎么用这几个类

第 5 节讲的拆线程设计，落到类上就是三个 worker 函数，各自调用上面的类：

```
EtherCAT 线程 updateWorker（严格 1kHz）    控制线程 controlWorker（可抖动）     发布线程 publishWorker（~100Hz）
──────────────────────────────           ──────────────────────           ──────────────────────────────
1. EcatBusManager::readAllBuses()        1. RmEcatHW::read()               1. SlaveManager::sendRos()
     SOEM 发帧+收帧                          从 reading_ 取反馈                 把 readings / imu / jointState
2. SlaveManager::updateProcessReadings() 2. controllerManager.update()        publish 成 ROS topic
     slave->updateRead()                     各控制器算命令
     解析 TxPdo → reading_               3. RmEcatHW::write()
3. SlaveManager::updateSendStagedCommands()   → stageMotorCommands()
     slave->updateWrite()                     写进 stagedCommand_
     从 stagedCommand_ 组 RxPdo
4. EcatBusManager::writeToAllBuses()
     发帧
```

两块加锁缓冲区把通信线程和控制线程解耦：

| 缓冲区 | 锁 | 控制线程 | EtherCAT 线程 |
| --- | --- | --- | --- |
| `stagedCommand_`（命令） | `stagedCommandMutex_` | 写 | 读 |
| `reading_`（反馈） | `readingMutex_` | 读 | 写 |
| `act_data_list_` | 无需锁 | 读+写（私有） | 不访问 |

`act_data_list_` 为什么不用锁？因为它是**控制线程的私有数据**——控制线程通过 `read()` 从加锁的 `reading_` 拷贝进来、通过 `write()` 拷贝到加锁的 `stagedCommand_`，EtherCAT 线程从头到尾不碰它。加锁只发生在两块交接缓冲上，`act_data_list_` 独享无争用。

一句话总结这四层：**BusManager 管帧、SlaveManager+Slave 管 PDO 和单位换算、RmEcatHardwareInterface 管接口对接、ROS control 管算法**——每一层只跟相邻层说话，往上暴露的都是干净接口。这就是"通信差异被封在硬件层"这句话在代码里的真实样子。

### 7.5 分层排障顺序

电机不动时从下往上查，能最快缩小范围：

1. **物理链路**：EtherCAT slave 是否 OP、工作计数是否正确；CAN 断电电阻是否约 60Ω，线序、终端和总线电压是否正常。
2. **反馈新鲜度**：只看每个 slave/actuator 最后更新时间、序号和故障码，先确认“数据在持续变新”，不要先调 PID。
3. **协议映射**：核对 bus、逻辑电机 ID、实际命令/反馈仲裁 ID、PDO 槽位和固件模式。
4. **控制链**：同时画 `target / measurement / error / pre-limit output / final output / saturation`，定位是没有目标、没有反馈、输出被限幅还是闭环方向错误。
5. **机械与单位**：最后查正方向、减速比、offset、卡滞和负载。

诊断数据应由 `publishWorker` 从一致快照降频发布到 ROS topic，再用 PlotJuggler 等工具观察。不要在 EtherCAT/update 实时路径里同步 `printf`、写文件或高频 publish；“为了看时序而打印”本身就可能破坏时序。

---

## 8. 小结

- 电脑和电机语言不通，中间必须有**协议 + 转发硬件 + 物理链路**三样东西，通信协议约定了双方的"话术"。
- **CAN** 是 RoboMaster 电机原生的总线，简单可靠，但带宽（1 Mbit/s、8 字节）、同步、扩展三方面都吃紧。
- **EtherCAT** 用 100BASE-TX 跑工业实时以太网，带宽大得多，DC 给从站提供公共时间基准，PDO 可承载电机、IMU、遥控器和 GPIO；板后 CAN 是否同步采样仍取决于从站固件。
- 迁移到 EtherCAT 的代价是通信路径上不允许抖动，`rm_ecat_hw` 用**三线程（通信 + 控制 + 发布）+ 双缓冲**把 ROS 的不可预测性隔离出 EtherCAT 关键路径来应对，代价是约一个周期的延迟。
- 无论走 CAN 还是 EtherCAT，差异都被封在硬件抽象层的 `RobotHW` 实现里，控制层和决策层完全无感。

下一站 [hardware](./hardware.md)：链路终点的电机长什么样、编码器怎么知道自己转到哪了、Transmission 怎么在关节和电机之间做翻译，以及每次上电为什么都要标定。
