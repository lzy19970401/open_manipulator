# 机器人运动学 · 空间向量 Glossary

本教学 workspace 的规范术语。与项目根目录 `CONTEXT.md`（OpenMANIPULATOR-X 软件/domain 词汇）互补。

## 运动学

**Twist（空间速度）**:
描述刚体运动的 6 维向量 V=(ω, v)，ω 为角速度、v 为参考系原点处线速度。
_Avoid_: 6D velocity（未强调共轭结构时）

**Forward kinematics（正运动学）**:
由关节变量 q 计算 End effector 位姿 T 的映射。
_Avoid_: FK 缩写（首次出现未定义时）

**Inverse kinematics（逆运动学）**:
由目标位姿 T 求关节变量 q 的映射；可能无解或多解。
_Avoid_: IK solver（实现层名称）

## 空间六维量

**Wrench（空间力）**:
作用在刚体上的广义力 6 维向量 F=(m, f)，m 为力矩、f 为力；与 Twist 共轭，功率 P=F^T V。
_Avoid_: 6D force, 广义力矩（仅指 m 时）

**Spatial momentum（空间动量）**:
刚体动量的 6 维向量 P=(n, h)，n 为角动量、h 为线动量。
_Avoid_: 6D momentum（未区分 n/h 时）

**Spatial acceleration（空间加速度）**:
刚体加速度的 6 维向量 A=(α, a)；body frame 下 a=dv/dt+ω×v。
_Avoid_: 6D acceleration, 对 Twist 直接求导（缺 ω×v 时）

## 动力学 · 重力补偿

**Analytical dynamics library（解析动力学库）**:
给定 q,q̇,q̈ 用公式直接算 τ、g(q)、M(q) 的库（Pinocchio、KDL 等），不做物理时间积分。
_Avoid_: 仿真器（MuJoCo/Gazebo 属于另一类）

**Physics simulator（物理仿真引擎）**:
对接触、摩擦、约束做时间步进，模拟系统随时间演化；可含动力学但目的在 sim 而非控制器内嵌 g(q)。
_Avoid_: 动力学库（混称「仿真算重力补偿」时）

**Gravity compensation（重力补偿）**:
在关节施加 τ_g = g(q)，抵消重力 Wrench 使 Arm 在静止构型下近似平衡、可手动拖动或低力持位。
_Avoid_: anti-gravity mode（未说明是力矩前馈时）

**Generalized gravity g(q)**:
拉格朗日动力学 Mq̈ + c(q,q̇) + g(q) = τ 中，仅含重力项的广义力向量；静止时 τ = g(q) 即可平衡。
_Avoid_: gravity torque（单关节口语，未强调是向量 g）

**Effort interface（力矩接口）**:
ros2_control / ros_control 中向关节发送力矩/电流命令的接口；重力补偿控制器必须写 effort，不能只写 position。
_Avoid_: torque topic（实现层路径名）
