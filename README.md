# 轮腿平衡机器人建模

轮腿平衡机器人的物理建模、VMC 映射、LQR 增益拟合与上层控制文档。整体遵循 [01_introduction.md](01_introduction.md) 中的流程：**抽象 → 建模 → 简化 → 求解 → 映射**。

## 文档索引

### 基础

| 文档 | 内容 | 状态 |
|------|------|------|
| [01_introduction.md](01_introduction.md) | 系统概述、控制层级、坐标与正方向约定 | 已完成 |
| [02_theoretical_mechanics.md](02_theoretical_mechanics.md) | 约束、广义坐标、拉格朗日方程等理论力学基础 | 已完成 |
| [03_control_theory.md](03_control_theory.md) | LQR 相关控制理论基础 | 已完成 |
| [04_vmc.md](04_vmc.md) | 五连杆 / 并联 / 偏置并联 VMC，足端力到关节力矩映射 | 已完成 |

### 物理建模（牛顿-欧拉）

符号与坐标系在 [05_newton_euler_single.md](05_newton_euler_single.md) 开头统一约定。三种建模由简到繁，受力分析与转动方程大部分可复用。

| 文档 | 内容 | 配套代码 |
|------|------|----------|
| [05_newton_euler_single.md](05_newton_euler_single.md) | 单腿建模（双腿合并） | [`code/modeling/single/`](code/modeling/single/) |
| [06_newton_euler_dual.md](06_newton_euler_dual.md) | 双腿建模（含 yaw 状态） | [`code/modeling/dual/`](code/modeling/dual/) |
| [07_newton_euler_dual_with_offset.md](07_newton_euler_dual_with_offset.md) | 双腿质心偏移建模（参考转轴、显式平衡点） | [`code/modeling/dual_offset/`](code/modeling/dual_offset/) |
| [16_leg_length_and_roll.md](16_leg_length_and_roll.md) | 腿长与高度 / Roll 的近似关系、支撑力约定 | 待验证 |

### 物理建模（拉格朗日）

| 文档 | 内容 | 状态 |
|------|------|------|
| [08_lagrangian_single.md](08_lagrangian_single.md) | 单腿拉格朗日建模 | 待补充 |
| [09_lagrangian_dual.md](09_lagrangian_dual.md) | 双腿拉格朗日建模 | 待补充 |
| [10_lagrangian_dual_with_offset.md](10_lagrangian_dual_with_offset.md) | 双腿质心偏移拉格朗日建模 | 待补充 |

### 控制与应用

| 文档 | 内容 | 状态 |
|------|------|------|
| [11_offground_and_jump.md](11_offground_and_jump.md) | 支持力解算、离地检测与跳跃控制 | 已完成 |
| [12_odometry.md](12_odometry.md) | 卡尔曼滤波里程计（IMU + 轮速融合） | 已完成 |
| [13_disturbance_observer.md](13_disturbance_observer.md) | 扰动观测器 | 待补充 |
| [14_state_machine.md](14_state_machine.md) | 状态机 | 待补充 |


## 代码结构

```
code/
├── modeling/
│   ├── params.m                 # 共享物理参数
│   ├── comparison.html          # 三套牛顿-欧拉模型矩阵系数对比（浏览器打开）
│   ├── single/                  # → 05_newton_euler_single.md
│   │   ├── model.m
│   │   ├── matrices.m
│   │   └── fit.m
│   ├── dual/                    # → 06_newton_euler_dual.md
│   │   ├── model.m
│   │   ├── matrices.m
│   │   └── fit.m
│   └── dual_offset/             # → 07_newton_euler_dual_with_offset.md
│       ├── model.m
│       ├── matrices.m
│       └── fit.m
└── vmc/                         # → 04_vmc.md
    ├── vmc_serial.hpp           # class VMCParallel（偏置并联）
    └── vmc_parallel.hpp         # class VMCLink（五连杆）
```

> **注意**：VMC 头文件名与类名交叉对应——`vmc_serial.hpp` 实现偏置并联 `VMCParallel`，`vmc_parallel.hpp` 实现五连杆 `VMCLink`。阅读代码时以类注释与 [04_vmc.md](04_vmc.md) 章节为准。

## 牛顿-欧拉建模（已实现）

三套建模共用同一接口与同一推导流程。

### 调用接口

```matlab
[A, B, thetall_eq, thetalr_eq] = matrices(ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr)
```

| 参数 | 含义 |
|------|------|
| `ll`, `lr` | 左右腿髋轴距离（轮轴 → 腿转轴，对应 md 中 $l$ / $l_{w,l}$） |
| `thetall0`, `thetalr0` | 腿质心偏置角（**仅** `dual_offset` 使用） |
| `dll`, `dlr` | 髋轴 → 腿质心距离（对应 md 中 $l_b$） |
| `Ill`, `Ilr` | 腿绕髋轴转动惯量 |

共享机体/轮子常数见 [`code/modeling/params.m`](code/modeling/params.m)（质量、惯量、$R_w$、$R_b$、$d_b$、$\theta_b^0$ 等）。

### 推导流程（`model.m`）

1. 建立 **完整非线性** 动力学方程（md 中消力后的式子，不做小角度线性化）
2. 计算平衡点  
   - `single` / `dual`：竖直平衡 $\theta = 0$  
   - `dual_offset`：md 中的 `atan2` 显式解 + $\theta_b = \pi/4 - \theta_b^0/2$
3. 在平衡点对 $q,\dot q,\ddot q,u$ 求 Jacobian → $M,D,K,H$
4. 组装 $\dot x = Ax + Bu$：$A$ 含 $-M^{-1}K$、$-M^{-1}D$，$B$ 为 $-M^{-1}H$

| 目录 | 状态维 | 控制输入 | 平衡点 |
|------|--------|----------|--------|
| `single` | 6：`[x,\dot x,\theta_l,\dot\theta_l,\theta_b,\dot\theta_b]` | `[τ_w, τ_l]` | $\theta_l=\theta_b=0$ |
| `dual` | 10：含 $\phi$ 与左右腿 | `[τ_wl, τ_wr, τ_ll, τ_lr]` | $\theta_{ll}=\theta_{lr}=\theta_b=0$ |
| `dual_offset` | 10：同 `dual` | 同 `dual` | 非零，随腿长与 $\theta_l^0$ 变化 |

### LQR 增益拟合（`fit.m`）

- **`single`**：单变量多项式 $K(l) = a_1 + a_2 l + a_3 l^2$
- **`dual` / `dual_offset`**：双变量 $K(L,R) = a_1 + a_2 L + a_3 R + a_4 L^2 + a_5 LR + a_6 R^2$，并拟合 `thetall_eq` / `thetalr_eq`

腿几何标定数据格式（每行）：`[l, θ_l⁰, d_l, I_l]`，范围约 0.13–0.40 m。

### 模型对比

[`code/modeling/comparison.html`](code/modeling/comparison.html) 在浏览器中打开，可对比三套牛顿-欧拉模型在相同参数下的矩阵系数差异（KaTeX 渲染）。

## VMC（已实现）

将 LQR 虚拟摆力/力矩映射到髋关节电机，见 [04_vmc.md](04_vmc.md)。

| 文件 | 类 | 对应文档章节 |
|------|-----|-------------|
| [`vmc_parallel.hpp`](code/vmc/vmc_parallel.hpp) | `VMCLink` | §五连杆 VMC |
| [`vmc_serial.hpp`](code/vmc/vmc_serial.hpp) | `VMCParallel` | §偏置并联 VMC |

依赖工程侧 `config_chassis.hpp`（本仓库未包含）。

## 使用方式

需要 **MATLAB**（Symbolic Math Toolbox）与 **Control System Toolbox**（`lqr`）。

**重新生成 `matrices.m`：**

```matlab
cd code/modeling/single       % 或 dual / dual_offset
run('model.m')
```

**LQR 增益拟合（输出 C 数组格式系数）：**

```matlab
cd code/modeling/single       % 或 dual / dual_offset
run('fit.m')
```

## 阅读路径建议

```
01 概论
  ↓
02 理论力学、03 控制理论（按需）
  ↓
04 VMC  ←→  code/vmc/*.hpp
  ↓
05 → 06 → 07  牛顿-欧拉建模  ←→  code/modeling/*/model.m
  ↓
fit.m 生成部署用增益表
  ↓
11 离地/跳跃  ·  12 里程计  ·  16 腿长与 Roll  ·  13/14（待补充）
```

