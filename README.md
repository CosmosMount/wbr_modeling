# 轮腿平衡机器人建模

轮腿平衡机器人的物理建模、VMC 映射与 LQR 增益拟合。文档按「理论 → 建模 → 控制」组织，代码与文档一一对应。

## 文档

按阅读顺序：

| 文档 | 内容 |
|------|------|
| [01_introduction.md](01_introduction.md) | 系统概述、控制层级、坐标与正方向约定 |
| [02_theoretical_mechanics.md](02_theoretical_mechanics.md) | 约束、广义坐标、拉格朗日方程等理论力学基础 |
| [03_control_theory.md](03_control_theory.md) | LQR 相关控制理论基础 |
| [04_vmc.md](04_vmc.md) | 五连杆 / 并联 / 偏置并联 VMC，足端力到关节力矩映射 |
| [05_newton_euler_single.md](05_newton_euler_single.md) | **单腿**牛顿-欧拉建模（双腿合并） |
| [06_newton_euler_dual.md](06_newton_euler_dual.md) | **双腿**牛顿-欧拉建模（含 yaw 状态） |
| [07_newton_euler_dual_with_offset.md](07_newton_euler_dual_with_offset.md) | **双腿质心偏移**建模（参考转轴、显式平衡点） |

三种牛顿-欧拉建模由简到繁，受力分析与转动方程大部分可复用，符号与坐标系在 [05](05_newton_euler_single.md) 开头统一约定。

## 代码结构

```
code/
├── modeling/
│   ├── params.m              # 共享物理参数（质量、惯量、几何常数）
│   ├── single/               # 对应 05_newton_euler_single.md
│   │   ├── model.m           # 符号推导 → 生成 matrices.m
│   │   ├── matrices.m        # [A,B,thetall_eq,thetalr_eq] = f(腿参数)
│   │   └── fit.m             # LQR 增益多项式拟合
│   ├── dual/                 # 对应 06_newton_euler_dual.md
│   │   ├── model.m
│   │   ├── matrices.m
│   │   └── fit.m
│   └── dual_offset/          # 对应 07_newton_euler_dual_with_offset.md
│       ├── model.m
│       ├── matrices.m
│       └── fit.m
└── vmc/
    ├── vmc_serial.hpp        # 串联腿 VMC（见 04_vmc.md）
    └── vmc_parallel.hpp      # 偏置并联 VMC（见 04_vmc.md §偏置并联VMC）
```

### 建模代码（`code/modeling/`）

三套建模共用同一接口与同一套流程：

```matlab
[A, B, thetall_eq, thetalr_eq] = matrices(ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr)
```

| 参数 | 含义 |
|------|------|
| `ll`, `lr` | 左右腿髋轴距离（轮轴→腿转轴） |
| `thetall0`, `thetalr0` | 腿质心偏置角（仅 `dual_offset` 使用） |
| `dll`, `dlr` | 腿质心到髋轴距离 |
| `Ill`, `Ilr` | 腿转动惯量 |

**统一推导流程**（`model.m`）：

1. 建立完整非线性动力学方程（md 中消力后的式子，非小角度线性化）
2. 计算平衡点（`single` / `dual` 为竖直平衡 `θ=0`；`dual_offset` 为 md 中的 `atan2` 显式解）
3. 在平衡点对 `q`, `dq`, `ddq`, `u` 求 Jacobian，得到 `M, D, K, H`
4. 组装状态空间：`ẋ = Ax + Bu`，其中 `A` 含 `-M⁻¹K`、`-M⁻¹D`，`B` 为 `-M⁻¹H`

| 模型 | 状态维 | 输入 | 文档 |
|------|--------|------|------|
| `single` | 6：`[x, ẋ, θ_l, θ̇_l, θ_b, θ̇_b]` | `[τ_w, τ_l]` | [05](05_newton_euler_single.md) |
| `dual` | 10：含 `φ` 与左右腿 | `[τ_wl, τ_wr, τ_ll, τ_lr]` | [06](06_newton_euler_dual.md) |
| `dual_offset` | 10：同 `dual` | 同 `dual` | [07](07_newton_euler_dual_with_offset.md) |

共享物理常数见 [`code/modeling/params.m`](code/modeling/params.m)。

### VMC 代码（`code/vmc/`）

将 LQR 输出的虚拟摆力矩映射到髋关节电机，实现见 [04_vmc.md](04_vmc.md)：

| 文件 | 说明 |
|------|------|
| [`vmc_serial.hpp`](code/vmc/vmc_serial.hpp) | 串联腿雅可比与力矩映射 |
| [`vmc_parallel.hpp`](code/vmc/vmc_parallel.hpp) | 偏置并联腿 `(l, φ)` 运动学与 `Jᵀ` 映射 |

## 使用方式

需要 MATLAB（Symbolic Math Toolbox）与 Control System Toolbox（`lqr`）。

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

`fit.m` 在腿长采样网格上调用 `matrices`，对 `K(L,R)` 及平衡角做多项式最小二乘拟合，供嵌入式查表使用。

## 阅读路径建议

```
01 概论 → 02 理论力学（按需） → 03 控制理论（按需）
    ↓
05 单腿 → 06 双腿 → 07 质心偏移
    ↓
code/modeling/*/model.m 对照文档公式
    ↓
04 VMC → code/vmc/*.hpp
    ↓
fit.m 生成部署用增益表
```

## 依赖与约定

- 力：竖直向上、水平向左为正（见 [05](05_newton_euler_single.md) 约定表）
- 腿几何数据格式（`fit.m`）：`[l, θ_l⁰, d_l, I_l]` 每行一条腿长标定
- `dual_offset` 的机体质心偏置角 `θ_b⁰` 固定在 `params.m` 的 `thetab0`
