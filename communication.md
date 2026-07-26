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
- **广播 + 仲裁**：没有"地址"的概念，每条报文带一个 **CAN ID**。所有设备都能听到每一条报文，各自根据 ID 决定"这条是不是发给我的"。当多个设备同时想说话时，ID 数值小的优先（仲裁），保证高优先级报文不被延误
- **报文短小**：经典 CAN 2.0 每条报文的数据段**最多 8 字节**，波特率通常 1 Mbit/s

### 2.2 CAN ID：电机的"门牌号"

在 RoboMaster 的用法里，CAN ID 同时承担了两件事：**区分是哪个电机**、**区分是命令还是反馈**。比如 C620 电调，一条 ID 的报文里塞了 4 个电机的目标电流，电机则用另一组 ID（0x201、0x202……）把自己的角度、转速、电流反馈回来。所以配置里给每个电机写的那个 ID/bus，本质就是它在总线上的门牌号——发命令、收反馈都靠它对号入座。

> 具体每种电机的报文格式、力矩怎么编码成整数，属于**电机控制协议**，放在 [hardware](./hardware.md) 里讲。这里只需要记住：CAN ID = 总线上区分设备的编号。

### 2.3 CAN FD

CAN FD（Flexible Data-rate）是 CAN 的升级版，把单条报文的数据段从 8 字节扩到最多 64 字节，数据段还能临时切到更高的波特率。它缓解了"报文太短"的痛点，但没有改变 CAN 总线型、单总线带宽有限的本质。rm-controls 的电机生态仍以经典 CAN 为主，CAN FD 更多是了解性知识。

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

1. **带宽有限**。CAN 2.0 只有 1 Mbit/s、每帧 8 字节。一台车要控 16 个电机，再加上 IMU、GPIO、遥控器数据，每个控制周期得发大约 30 条 CAN 帧，总线利用率被压到极限。
2. **没有同步机制**。读和写都在一个线程里串行完成，各个电机的反馈时刻、命令时刻天然是错开的、异步的，谈不上"同一时刻的快照"。
3. **扩展性差**。一条 CAN 总线最多挂 8 个电机，想再多就得加 CAN 转接卡，一路一路地堆。

---

## 3. EtherCAT：把总线搬到网线上

### 3.1 它是什么

EtherCAT（Ethernet for Control Automation Technology）是 Beckhoff 提出的一种**工业实时以太网**协议。它用的是普通千兆网卡、普通网线，但工作方式和普通以太网很不一样：

- **主站—从站结构**：上位机是唯一的**主站**（master），所有设备是**从站**（slave）。主站发一帧数据，这帧数据像地铁一样依次穿过每一个从站
- **"飞速处理"（on the fly）**：每个从站在这帧数据经过自己的一瞬间，就地读走属于自己的部分、写入自己的反馈，几乎不产生延迟。一帧数据跑一圈回来，所有从站的命令都发下去了、所有反馈也都收上来了
- **分布式时钟（Distributed Clock, DC）**：EtherCAT 有一套让所有从站对齐时间的机制，可以把各从站之间的时间抖动压到 **1 微秒以内**。这意味着"所有电机在同一时刻动作"从愿望变成了现实——这正是 CAN 方案给不了的同步性

### 3.2 从站拓扑

在 rm-controls 里，EtherCAT 从站是一块**转接板**：它一头是网口（连上位机），另一头是 CAN 控制器（连电机）。也就是说——

> **CAN 并没有消失，而是被"下沉"到了从站内部。** 电机和电调之间仍然说 CAN，只不过这段 CAN 变得很短、就在从站板上；从站板对外则统一用 EtherCAT 和上位机通信。

一块从站板通常带两路 CAN（CAN0 + CAN1），于是单块从站就能接约 16 个电机。更重要的是：电机命令、IMU 采集、遥控器（DBus）数据、GPIO 控制——这些原本要挤在多路 CAN 上的东西，现在**全部打包进同一帧 EtherCAT 数据**里一起收发，彻底避开了多路总线之间"各走各的、时间对不齐"的问题。

### 3.3 分布式时钟解决了什么

回想 CAN 的痛点之二——没有同步。EtherCAT 的分布式时钟正是对症下药：所有从站共享一个亚微秒精度的公共时钟，主站可以让它们在约定的同一时刻采样、同一时刻输出。对控制来说，这意味着你拿到的一整套电机反馈是**同一时刻的快照**，算出的命令也是**同一时刻下发**，闭环的时序基础干净得多。

---

## 4. 为什么从 CAN 迁移到 EtherCAT

把上面的对比拉成一张表，迁移的动机就一目了然：

| 维度 | CAN 方案（rm_hw） | EtherCAT 方案（rm_ecat_hw） |
| --- | --- | --- |
| 线程数 | 1 | 2 |
| 通信介质 | CAN 总线（1 Mbit/s） | 以太网（1000 Mbit/s） |
| 电机数 / 总线 | 8 个 / CAN | 约 16 个 / 从站（CAN0 + CAN1） |
| 每周期数据量 | ~30 条 CAN 帧 | 1 帧 EtherCAT（约 100 字节） |
| 同步性 | 无显式同步 | DC 时钟同步（抖动 < 1μs） |
| 反馈内容 | 只有电机位置 / 速度 / 电流 | 电机 + IMU + GPIO + 遥控器同在一帧 |
| 扩展方式 | 加 CAN 转接卡 | 加 EtherCAT 从站 |

一句话：**EtherCAT 用大得多的带宽、亚微秒的同步、以及"一帧承载所有外设"的能力，一次性解决了 CAN 的带宽、同步、扩展三个痛点。** 队内现在已经不再走 Linux-to-CAN 的老路，只用 `rm_ecat_hw`。

不过 EtherCAT 也不是白拿好处——它引入了一个新麻烦：**通信时序变得很严格**。这就是下一节要讲的双线程设计的由来。

---

## 5. 双线程：EtherCAT 带来的新问题与解法

### 5.1 矛盾

EtherCAT 主站（rm-controls 用的是 SOEM，Simple Open EtherCAT Master 库）要求"发一帧 → 等接收 → 解析反馈 → 组装下一帧"必须在**固定的 1kHz 周期**内严丝合缝地完成，不能等控制器慢慢算。但控制器的计算时间是**不确定**的——大多数时候几十微秒，偶尔碰上弹道解算这种重活可能冲到几百微秒。

如果像 CAN 方案那样塞进一个线程里顺序执行：

```
read() → controllerA::update() → controllerB::update() → write()
│         ├── 可能 200~800μs 抖动 ──┤                        │
└──────────────── 总时间不固定，write() 可能迟到 ────────────┘
```

控制器这一拍算久了，`write()` 就迟到，EtherCAT 帧错过了它该发的时刻，总线同步就丢了。

### 5.2 解法：拆成两个线程，用缓冲区解耦

`rm_ecat_hw` 的核心设计决策是把循环拆成两个线程：

```
控制线程（controlWorker）          双缓冲队列（加锁保护）        EtherCAT 线程（updateWorker）
────────────────────              ──────────────────         ────────────────────────
计算时间可抖动                                                 严格固定 1kHz
1. read()  从缓冲读上一拍反馈  ◄──────  reading_  ◄──────────  收到帧后写入反馈
2. controllerManager.update()                                
3. write() 把命令写入缓冲       ──────►  stagedCommand_  ────►  下一拍取走、组帧发送
```

- **控制线程**不受严格时序约束，允许偶尔超时；它只管从缓冲区拿最新反馈、算、把命令丢进缓冲区
- **EtherCAT 线程**雷打不动地按 1kHz 收发帧；它从缓冲区取走上一拍的命令去发，把收到的反馈写进缓冲区

两个线程通过两块加锁的缓冲区（`stagedCommand_` 命令缓冲、`reading_` 反馈缓冲）交接数据。代价是多引入了**一个周期的延迟**（平均约 500μs）——这是用一点点延迟换取通信时序的绝对稳定，非常划算。

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
  网卡（普通千兆网口）
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
| `rm_ecat_hw` | EtherCAT（SOEM） | 双线程 |

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

### 7.4 两个 worker 线程怎么用这几个类

第 5 节讲的双线程，落到类上就是两个 worker 函数，各自调用上面的类：

```
控制线程 controlWorker（计算可抖动）      EtherCAT 线程 updateWorker（严格 1kHz）
──────────────────────────              ──────────────────────────────
1. RmEcatHW::read()                      1. EcatBusManager::readAllBuses()
     从 reading_ 缓冲取上一拍反馈              SOEM 发帧+收帧
2. controllerManager.update()            2. SlaveManager::updateProcessReadings()
     各控制器算命令                            slave->updateRead() 解析 TxPdo → reading_
3. RmEcatHW::write()                      3. SlaveManager::updateSendStagedCommands()
     → SlaveManager::stageMotorCommands()      slave->updateWrite() 从 stagedCommand_ 组 RxPdo
       写进 stagedCommand_ 缓冲            4. EcatBusManager::writeToAllBuses() 发帧
```

两块加锁缓冲区把两个线程解耦：

| 缓冲区 | 锁 | 控制线程 | EtherCAT 线程 |
| --- | --- | --- | --- |
| `stagedCommand_`（命令） | `stagedCommandMutex_` | 写 | 读 |
| `reading_`（反馈） | `readingMutex_` | 读 | 写 |
| `act_data_list_` | 无需锁 | 读+写（私有） | 不访问 |

`act_data_list_` 为什么不用锁？因为它是**控制线程的私有数据**——控制线程通过 `read()` 从加锁的 `reading_` 拷贝进来、通过 `write()` 拷贝到加锁的 `stagedCommand_`，EtherCAT 线程从头到尾不碰它。加锁只发生在两块交接缓冲上，`act_data_list_` 独享无争用。

一句话总结这四层：**BusManager 管帧、SlaveManager+Slave 管 PDO 和单位换算、RmEcatHardwareInterface 管接口对接、ROS control 管算法**——每一层只跟相邻层说话，往上暴露的都是干净接口。这就是"通信差异被封在硬件层"这句话在代码里的真实样子。

---

## 8. 小结

- 电脑和电机语言不通，中间必须有**协议 + 转发硬件 + 物理链路**三样东西，通信协议约定了双方的"话术"。
- **CAN** 是 RoboMaster 电机原生的总线，简单可靠，但带宽（1 Mbit/s、8 字节）、同步、扩展三方面都吃紧。
- **EtherCAT** 用普通网线跑工业实时以太网，带宽大得多，靠分布式时钟做到亚微秒同步，还能把电机、IMU、遥控器、GPIO 全塞进一帧——CAN 被下沉到从站板内部。
- 迁移到 EtherCAT 的代价是通信时序变严格，`rm_ecat_hw` 用**双线程 + 缓冲区**解耦通信与计算来应对，代价是约一个周期的延迟。
- 无论走 CAN 还是 EtherCAT，差异都被封在硬件抽象层的 `RobotHW` 实现里，控制层和决策层完全无感。

下一站 [hardware](./hardware.md)：链路终点的电机长什么样、编码器怎么知道自己转到哪了、Transmission 怎么在关节和电机之间做翻译，以及每次上电为什么都要标定。
