# OpenMANIPULATOR-X

ROBOTIS 出品的桌面级四自由度机械臂，末端带平行夹爪。本仓库中 `open_manipulator_x` 是其 ROS 2 软件栈所对应的机器人型号；与 OMY、OMX 等其它 OpenMANIPULATOR 变体并列，但 kinematic 结构、关节命名与预设姿态各自独立。

## 机械结构

**OpenMANIPULATOR-X**:
本上下文中指代该四轴臂 + 平行夹爪这一完整机器人，而非某个软件包或 launch 文件。
_Avoid_: omx, Open Manipulator X（混用大小写时）

**Arm（机械臂）**:
由 `joint1`–`joint4` 四个旋转关节串联而成的四自由度运动链，从基座 `link1` 延伸到腕部 `link5`。
_Avoid_: manipulator chain（泛指时）、4-DOF arm（口语化描述）

**Gripper（夹爪）**:
安装在 `link5` 上的平行两指末端执行器，通过开合抓取物体。
_Avoid_: tool, end tool（与 End effector 混淆时）

**Master finger（主指）**:
夹爪中受独立控制的指部，对应 `gripper_left_joint`；所有夹爪开合命令只作用于主指。
_Avoid_: left gripper, gripper joint（未说明左右/master 关系时）

**Mimic finger（从指）**:
镜像主指运动的指部，对应 `gripper_right_joint`；不单独接收控制指令，始终跟随主指。
_Avoid_: right gripper, slave finger

**End effector（末端执行器参考点）**:
固定在 `link5` 上的工具中心点（`end_effector_link`），代表任务空间运动的规划与测量基准；不是指夹爪手指本身。
_Avoid_: TCP（未在团队内约定时）、tool tip

**Link / Joint**:
Link 是刚性连杆；Joint 是连接两根 link 的可动轴。OpenMANIPULATOR-X 的 arm 含 `link1`–`link5` 与 `joint1`–`joint4`；夹爪另含 `gripper_left_link`、`gripper_right_link` 及对应关节。
_Avoid_: segment, axis（未明确是 link 还是 joint 时）

## 运动与姿态

**Joint-space motion（关节空间运动）**:
以各关节目标角度描述的运动，仅涉及 `joint1`–`joint4`（arm）或主指关节（gripper）。
_Avoid_: joint trajectory（实现层术语）

**Task-space motion（任务空间运动）**:
以 End effector 在 `world` 坐标系下的位姿（位置 + 姿态）描述的运动。
_Avoid_: Cartesian motion（未区分参考 frame 时）

**Named pose（命名姿态）**:
一组有固定名称、对应已知关节值的预设构型。OpenMANIPULATOR-X 的 arm 侧有 `init`、`home`；gripper 侧有 `open`、`close`。
_Avoid_: preset, waypoint（未强调是命名构型时）

**Init pose**:
Arm 的全零构型：`joint1`–`joint4` 均为 0。通常作为规划组的默认初始状态。
_Avoid_: zero pose, origin

**Home pose**:
Arm 的折叠/收拢构型（SRDF 中 `joint2=-1, joint3=0.7, joint4=0.3`），区别于 Init pose。
_Avoid_: rest pose（与 Ready pose 混淆时）

**Ready pose**:
启动序列中的工作预备构型（`joint2=-1, joint3=1, joint4=0`），由 bringup 在控制器就绪后执行，用于将 arm 从 Home/Init 过渡到可操作状态。
_Avoid_: standby, start pose

**Open / Close（夹爪状态）**:
Gripper 的两个命名姿态：Open 为主指张开（约 0.019 m），Close 为主指闭合（约 -0.01 m）。
_Avoid_: grasp / release（描述动作意图而非构型时）

## 控制语义

**Arm group**:
运动规划时将 `joint1`–`joint4` 与 End effector 固定关节视为一个整体的分组；Task-space 与 Joint-space 的 arm 运动均在此分组内规划。
_Avoid_: arm chain, move group（实现层名称）

**Gripper group**:
运动规划时仅包含夹爪关节（`gripper_left_joint`、`gripper_right_joint`）的分组；与 Arm group 分开规划与执行。
_Avoid_: hand group, tool group

**Operation mode（运行模式）**:
机器人所处的三类互斥运行情境之一：**Hardware**（经串口驱动真实 Dynamixel 舵机）、**Simulation**（Gazebo 仿真，无物理连接）、**Mock hardware**（软件回环，命令被镜像为状态，无真实或仿真物理）。
_Avoid_: real vs fake, sim flag

**Standard control launch（标准控制 launch）**:
通过 `open_manipulator_x.launch.py` 启动的默认配置：`open_manipulator_x_position` ros2_control、`arm_controller` + `gripper_controller` 位置控制，用于 MoveIt / 轨迹执行。
_Avoid_: normal mode, position launch（未与 GC launch 区分时）

**Gravity compensation control mode（重力补偿控制模式）**:
通过独立 launch 启动的互斥配置：Arm 四关节（`joint1`–`joint4`）走 **effort** 接口，控制器前馈 τ≈g(q)，使 Arm 近似反驱动、可用手示教；**不含**轨迹录制；退出时 effort 清零并 **Torque disable**。与 Standard control launch 二选一，不可同时加载。操作说明见 [docs/open-manipulator-x-gravity-compensation.md](docs/open-manipulator-x-gravity-compensation.md)。
_Avoid_: GC mode（未限定型号时）、leader mode（OMX 无 follower 同步）

**Torque enable（力矩使能）**:
Dynamixel 舵机是否输出 holding torque 的安全开关；禁用时关节可手动拖动，启用时执行位置/电流控制。
_Avoid_: motor on/off, power

## 执行器

**Dynamixel actuator（Dynamixel 执行器）**:
OpenMANIPULATOR-X 的五个受控轴各对应一颗 Dynamixel 舵机，ID 依次为 11–15，分别驱动 `joint1`–`joint4` 与 Master finger。
_Avoid_: servo, motor（未特指 Dynamixel 协议栈时）

**Position control mode（位置控制模式）**:
Arm 四关节（ID 11–14）的工作模式：以目标关节角为控制量。
_Avoid_: mode 3（寄存器编号）

**Current-based position control mode（电流限制位置控制模式）**:
Master finger（ID 15）的工作模式：在位置控制基础上以 Goal Current 限制夹持力，用于抓取时防止过夹或滑脱。
_Avoid_: mode 5, force control（本机并非纯力控）

## 参数辨识

**System identification（系统辨识）**:
通过受控激励与测量，从实测数据估计机器人动力学模型参数的过程；本仓库 Phase 1 仅覆盖 **Arm** 四关节（`joint1`–`joint4`），不含 **Gripper**。
_Avoid_: calibration（未区分 kinematic/dynamic 时）、parameter tuning（指控制器增益时）

**Nominal dynamics model（名义动力学模型）**:
由官方 URDF 惯性参数与运动学链构成的理想模型，作为辨识对照基准与下游控制的默认来源。
_Avoid_: CAD model, ideal model（未强调是 URDF 来源时）

**Calibrated dynamics model（标定动力学模型）**:
在名义模型基础上，经辨识或单关节实验修正后的模型；参数存放在独立配置文件中，不覆盖 URDF 或现有 GC yaml。
_Avoid_: identified URDF, tuned yaml

**Base inertial parameters（BIP，最小惯性参数集）**:
经符号重组后线性独立、可辨识的等效惯性参数组合（如 `m_i c_{ix}`、`I_{xx,i}-I_{yy,i}`），而非各连杆独立的 10 个惯性参数。
_Avoid_: link inertia, URDF inertial（指原始 per-link 参数时）

**Excitation trajectory（激励轨迹）**:
为充分激励动力学而设计的关节空间参考轨迹，常用多频傅里叶级数以保证各参数可观测。
_Avoid_: test motion, random move

**Identification dataset（辨识数据集）**:
单次或多次激励实验同步记录的 `(q, q̇, q̈, τ)` 时间序列及元数据（采样率、关节范围、实验编号）。
_Avoid_: bag, log（未强调是辨识用途时）

**Viscous friction coefficient（粘滞摩擦系数 Fv）**:
与关节角速度成正比的摩擦项系数，线性模型中为 `τ_f = Fv · q̇`。
_Avoid_: damping, B（未区分 URDF `<dynamics>` 阻尼时）

**Coulomb friction coefficient（库仑摩擦系数 Fc）**:
与运动方向有关的常值干摩擦系数，线性模型中为 `τ_f = Fc · sign(q̇)`。
_Avoid_: static friction（未区分 stiction 模型时）
