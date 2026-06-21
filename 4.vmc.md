# VMC

| **Note** · VMC |
| :-- |

Virtual Model Control，虚拟模型控制，在机器人的**任务空间**（对于腿来说，就是足端）上创造虚拟的机械元件（如弹簧、阻尼器），然后通过**雅可比矩阵**将这些虚拟元件产生的虚拟力映射到机器人的关节力矩上

我们可以将腿模型

![wbr-spring](assets/wbr-spring.png)

抽象成

![pendulum](assets/pendulum.png)

对于BFGH，具有约束

```math
\frac{AC}{AI}=\frac{AB}{AH}=\frac{BC}{HI}=k
```

对于所谓并联和偏置并联，在几何建模上是等效的，偏置并联甚至有更好的几何性质，可以大幅度简化计算，但是因为我们是从并联开始到偏置并联，所以先使用通用的五连杆VMC计算。

## 五连杆VMC

通过VMC的思想，我们希望把五连杆简化为一个绕O点旋转的、长度可变的杆OC，即只考虑足端的运动，说杆其实并不准确，是弹簧阻尼系统更为合理，但是视为弹簧对后续分析太过复杂，因此选择解出长度后，视为长度不变的杆进行物理建模，再针对每个长度进行控制。

同时，在串腿上，我们希望能够消除气弹簧的影响，因此也需要将气弹簧等效到我们建模的方向上，同时产生一个力和一个力矩。

### F_L, τ_p

| **Note** · 坐标系 |
| :-- |

以 $`O`$ 为原点，向左为 $`x`$ 轴正方向，向下为 $`y`$ 轴正方向建立坐标系

![pendulum-1](assets/pendulum-1.png)

我们的电机放在 $`\phi_{1},\phi_{4}`$ ，分别对应力矩 $`\tau_{1},\tau_{4}`$ ，我们能够得到电机的角度和速度反馈，那么为了求解完整的五连杆，我们需要通过 $`\phi_{1},\phi_{4}`$ 求得 $`\phi_{2},\phi_{3}`$ ，再进一步求出 $`\phi`$ 和 $`L`$ 

分析点 $`C`$ 坐标，我们有

```math
\begin{equation}
\left\{
\begin{aligned}
x_{B}+l_{2}\cos \phi_{2} &= x_{D}+l_{3}\cos \phi_{3} \\
y_{B}+l_{2}\sin \phi_{2} &= y_{D}+l_{3}\sin \phi_{3}
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
x_{B}-x_{D}+l_{2}\cos \phi_{2} &= l_{3}\cos \phi_{3} \\
y_{B}-y_{D}+l_{2}\sin \phi_{2} &= l_{3}\sin \phi_{3}
\end{aligned}
\right.
\end{equation}
```

平方后相加消去 $`\phi_{3}`$

```math
(x_{B}-x_{D})^{2}+(y_{B}-y_{D})^{2}+l_{2}^{2}-l_{3}^{2}+2(x_{B}-x_{D})l_{2}\cos \phi_{2}+2(y_{B}-y_{D})l_{2}\sin \phi_{2}=0
```

由于

```math
l_{2}=l_{3}=L_{2}
```

可以简化为

```math
(x_{B}-x_{D})^{2}+(y_{B}-y_{D})^{2}+2(x_{B}-x_{D})L_{2}\cos \phi_{2}+2(y_{B}-y_{D})L_{2}\sin \phi_{2}=0
```

令

```math
A=2L_{2}(x_{D}-x_{B}),B=2L_{2}(y_{D}-y_{B}),C=(x_{B}-x_{D})^{2}+(y_{B}-y_{D})^{2}
```

有

```math
C-A\cos \phi_{2}-B\sin \phi_{2}=0
```

```math
C-A\left(  \frac{1-\tan^2\left( \frac{\phi_{2}}{2} \right)}{1+\tan^2\left( \frac{\phi_{2}}{2} \right)} \right)-B\left( \frac{2\tan\left( \frac{\phi_{2}}{2} \right)}{1+\tan^2\left( \frac{\phi_{2}}{2} \right)}  \right)=0
```

那么

```math
C\left( 1+\tan^2\left( \frac{\phi_{2}}{2} \right) \right)-A\left( 1-\tan^2\left( \frac{\phi_{2}}{2} \right) \right)-2B\left( \tan\left( \frac{\phi_{2}}{2} \right) \right)=0
```

```math
(C+A)\tan^2\left( \frac{\phi_{2}}{2} \right)-2B\tan\left( \frac{\phi_{2}}{2} \right)+C-A=0
```

解得

```math
\tan\left( \frac{\phi_{2}}{2} \right)= \frac{2B-\sqrt{ 4B^{2}-4(C+A)(C-A) }}{2(C+A)}= \frac{B-\sqrt{ A^{2}+B^{2}-C^{2} }}{A+C}
```

那么

```math
\phi_{2}=2\arctan \left(  \frac{B-\sqrt{ A^{2}+B^{2}-C^{2} }}{A+C} \right)
```

得到 $`\phi_{2}`$ 后我们能够通过 $`CD`$ 点的坐标得到 $`\phi_{3}`$ 

对于雅可比矩阵，我们还需要用  $`\dot{\phi}_{1},\dot{\phi}_{4}`$ 表示 $`\dot{\phi}_{2},\dot{\phi}_{3}`$ 

为了直接求解到可以直接代入物理量，展开 $`x_{B},x_{D}`$

```math
\begin{equation}
\left\{
\begin{aligned}
L_{1}\cos \phi_{1}-d+L_{2}\cos \phi_{2} &= L_{1}\cos \phi_{4}+d+L_{2}\cos \phi_{3} \\
L_{1}\sin \phi_{1}+L_{2}\sin \phi_{2} &= L_{1}\sin \phi_{4}+L_{2}\sin \phi_{3}
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
L_{1}\dot{\phi}_{1}\sin \phi_{1}+L_{2}\dot{\phi}_{2}\sin \phi_{2} &= L_{1}\dot{\phi}_{4}\sin \phi_{4}+L_{2}\dot{\phi}_{3}\sin \phi_{3} \\
L_{1}\dot{\phi}_{1}\cos \phi_{1}+L_{2}\dot{\phi}_{2}\cos \phi_{2} &= L_{1}\dot{\phi}_{4}\cos \phi_{4}+L_{2}\dot{\phi}_{3}\cos \phi_{3}
\end{aligned}
\right.
\end{equation}
```

解得

```math
\begin{equation}
\left\{
\begin{aligned}
\dot{\phi}_{2}= \frac{L_{1}(\dot{\phi}_{1}\sin(\phi_{3}-\phi_{1})+\dot{\phi}_{4}(\phi_{4}-\phi_{3}))}{L_{2}\sin(\phi_{2}-\phi_{3})} \\
\dot{\phi}_{3}= \frac{L_{1}(\dot{\phi}_{1}\sin(\phi_{2}-\phi_{1})+\dot{\phi}_{4}(\phi_{4}-\phi_{2}))}{L_{2}\sin(\phi_{2}-\phi_{3})}
\end{aligned}
\right.
\end{equation}
```

接下来，我们终于可以求解足端和两个关节关系了，

即**工作空间** 

```math
x=\left[\begin{matrix}
x_{C} \\
y_{C}
\end{matrix}\right],F=\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]
```

与**关节空间** 

```math
q=\left[\begin{matrix}
\dot{\phi}_{1} \\
\dot{\phi}_{4}
\end{matrix}\right],\tau=\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]
```

对于 $`C`$ 点坐标

```math
\begin{equation}
\left\{
\begin{aligned}
x_{C}&=L_{1}\cos \phi_{1}+L_{2}\cos \phi_{2}-d \\
y_{C}&=L_{1}\sin \phi_{1}+L_{2}\sin \phi_{2}
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
\dot{x}_{C}&=-(L_{1}\dot{\phi}_{1}\sin \phi_{1}+L_{2}\dot{\phi}_{2}\sin \phi_{2}) \\
\dot{y}_{C}&=L_{1}\dot{\phi}_{1}\cos \phi_{1}+L_{2}\dot{\phi}_{2}\cos \phi_{2}
\end{aligned}
\right.
\end{equation}
```

代入上面求解的结果并转化为矩阵形式

```math
\left[\begin{matrix}
\dot{x}_{C} \\
\dot{y}_{C}
\end{matrix}\right]=\frac{L_{1}}{\sin(\phi_{2}-\phi_{3})}\left[\begin{matrix}
\sin(\phi_{1}-\phi_{2})\sin \phi_{3} & \sin(\phi_{3}-\phi_{4})\sin \phi_{2} \\
-\sin(\phi_{1}-\phi_{2})\cos \phi_{3} & -\sin(\phi_{3}-\phi_{4})\cos \phi_{2}
\end{matrix}\right]\left[\begin{matrix}
\dot{\phi}_{1} \\
\dot{\phi}_{4}
\end{matrix}\right]
```

获得雅可比矩阵

```math
J=\frac{L_{1}}{\sin(\phi_{2}-\phi_{3})}\left[\begin{matrix}
\sin(\phi_{1}-\phi_{2})\sin \phi_{3} & \sin(\phi_{3}-\phi_{4})\sin \phi_{2} \\
-\sin(\phi_{1}-\phi_{2})\cos \phi_{3} & -\sin(\phi_{3}-\phi_{4})\cos \phi_{2}
\end{matrix}\right]
```

根据虚功原理，我们得到

```math
\tau^{T}\delta q -F^{T}\delta x=0
```

| **Tip** |
| :-- |

我们约定关节力矩 $`\tau`$ 做正功，外部虚拟力 $`F`$ 做负功

因此

```math
\tau^T\delta q=F^{T}\delta x=F^{T}J\delta q\implies (\tau^{T}-F^{T}J)\delta q=0
```

由于 $`\delta q`$ 是任意的，则必然有

```math
\tau ^{T}-F^{T}J=0\implies \tau^{T}=F^{T}J\implies \tau=J^{T}F
```

最终，我们就得到了

```math
\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]=J^{T}\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]
```

![pendulum-1](assets/pendulum-1.png)

然而，我们需要的是倒立摆的力和力矩，将 $`F_{x},F_{y}`$ 旋转到 $`F_{L},F_{T}`$

```math
\left[ \begin{matrix}
F_{L} \\
F_{T}
\end{matrix} \right]=\left[\begin{matrix}
\cos \phi & \sin \phi \\
-\sin \phi & \cos \phi
\end{matrix}\right]\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]
```

在转化到 $`F_{L},\tau_{P}`$

```math
\left[ \begin{matrix}
F_{L} \\
\tau_{P}
\end{matrix} \right]
=\left[  \begin{matrix}
1 & 0 \\
0 & L
\end{matrix} \right]\left[ \begin{matrix}
F_{L} \\
F_{T}
\end{matrix} \right]
=\left[  \begin{matrix}
1 & 0 \\
0 & L
\end{matrix} \right]\left[\begin{matrix}
\cos \phi & \sin \phi \\
-\sin \phi & \cos \phi
\end{matrix}\right]\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]
```

那么记

```math
M=\left[  \begin{matrix}
1 & 0 \\
0 & L
\end{matrix} \right],R=\left[\begin{matrix}
\cos \phi & \sin \phi \\
-\sin \phi & \cos \phi
\end{matrix}\right]
```

有

```math
\left[ \begin{matrix}
F_{L} \\
\tau_{p}
\end{matrix} \right]=MR\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]\implies\left[\begin{matrix}
F_{x} \\
F_{y}
\end{matrix} \right]=R^{-1}M^{-1}\left[ \begin{matrix}
F_{L} \\
\tau_{P}
\end{matrix} \right]
```

那么

```math
\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]=J^{T}R^{-1}M^{-1}\left[ \begin{matrix}
F_{L} \\
\tau_{p}
\end{matrix} \right]
```

```math
\left[ \begin{matrix}
\dot{L} \\
\dot{\phi}
\end{matrix} \right]=\left[  \begin{matrix}
1 & 0 \\
0 & \frac{1}{L}
\end{matrix} \right]\left[\begin{matrix}
\cos \phi & \sin \phi \\
-\sin \phi & \cos \phi
\end{matrix}\right]\left[\begin{matrix}
\dot{x}_{C} \\
\dot{y}_{C}
\end{matrix} \right]=\left[  \begin{matrix}
1 & 0 \\
0 & \frac{1}{L}
\end{matrix} \right]RJ\left[\begin{matrix}
\dot{\phi}_{1} \\
\dot{\phi}_{4}
\end{matrix}\right]
```

### F_S

![wbr-spring](assets/wbr-spring.png)

![wbr-spring-2](assets/wbr-spring-2.png)

那么，我们可以用类似的方法对气弹簧的等效力和力矩做解算，观察车体发现气弹簧固定在 $`L_{1}`$ 上，可以计算到和竖直方向夹角 $`\alpha`$ 和两个偏置距离 $`d_{1},d_{2}`$ 

```math
\begin{equation}
\left\{
\begin{aligned}
x_{J}&=d_{1}\cos \left( \phi_{1}+\alpha-\frac{\pi}{2} \right)= d_{1}\sin ( \phi_{1}+\alpha)\\
y_{J}&=d_{1}\sin \left( \phi_{1}+\alpha-\frac{\pi}{2} \right)=-d_{1}\cos(\phi_{1}+\alpha)
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
x_{K}&=L_{1}\cos \phi_{1}+d_{2}\cos \phi_{2} \\
y_{K}&=L_{1}\sin \phi_{1}+d_{2}\sin \phi_{2}
\end{aligned}
\right.
\end{equation}
```

对于 $`K`$ 点而言，由于点 $`J`$ 固定，实际上 $`F_{S}`$ 只提供了推力，而不会产生扭转的力，我们只需分析 $`l_{s}`$ 与 $`\phi_{1},\phi_{4}`$ 的关系，分解到 $`x,y`$ 方向上等价于

```math
\begin{equation}
\left\{
\begin{aligned}
x_{K}-x_{J}&=L_{1}\cos \phi_{1}+d_{2}\cos \phi_{2} -d_{1}\sin ( \phi_{1}+\alpha) \\
y_{K}-y_{J}&=L_{1}\sin \phi_{1}+d_{2}\sin \phi_{2}+d_{1}\cos ( \phi_{1}+\alpha)
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
\dot{x}_{K}-\dot{x}_{J}&=-L_{1}\dot{\phi}_{1}\sin \phi_{1}-d_{2}\dot{\phi}_{2}\sin \phi_{2} -d_{1}\dot{\phi}_{1}\cos (\phi_{1}+\alpha) \\
\dot{y}_{K}-\dot{y}_{J}&=L_{1}\dot{\phi}_{1}\cos \phi_{1}+d_{2}\dot{\phi}_{2}\cos \phi_{2}-d_{1}\dot{\phi}_{1}\sin(\phi_{1}+\alpha)
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
F_{S_{x}}&=F_{S} \frac{x_{K}-x_{J}}{l_{S}} \\
F_{S_{y}}&=F_{S} \frac{y_{K}-y_{J}}{l_{S}}
\end{aligned}
\right.
\end{equation}
```

此时可以看作以下问题

**工作空间** 

```math
x=\left[\begin{matrix}
x_{K}-x_{J} \\
y_{K}-y_{J}
\end{matrix}\right],F=\left[\begin{matrix}
F_{S_{x}} \\
F_{S_{y}}
\end{matrix} \right]
```

**关节空间** 

```math
q=\left[\begin{matrix}
\dot{\phi}_{1} \\
\dot{\phi}_{4}
\end{matrix}\right],\tau=\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]
```

## 偏置并联VMC

### F_L, τ_l

其实不难想到，我们能够利用偏置并联构型的特殊几何性质大幅度简化上面的运算

![pendulum-3](assets/pendulum-3.png)

先前，我们为了运算的简便性采用了坐标再转到长度、角度，现在可以直接对长度、角度的几何关系进行计算，那么我们转为研究下面的问题

**工作空间** 

```math
x=\left[\begin{matrix}
L \\
\phi
\end{matrix}\right],F=\left[\begin{matrix}
F_{L} \\
\tau_{p}
\end{matrix} \right]
```

**关节空间** 

```math
q=\left[\begin{matrix}
\phi_{1} \\
\phi_{4}
\end{matrix}\right],\tau=\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]
```

由高度对称性，得到

```math
\phi=\frac{\phi_{1}-\phi_{4}}{2}+\phi_{4}=\frac{\phi_{1}+\phi_{4}}{2}\implies \dot{\phi}=\frac{\dot{\phi}_{1}+\dot{\phi}_{4}}{2}
```

记

```math
\alpha=\frac{\phi_{1}-\phi_{4}}{2}\implies \dot{\alpha}=\frac{\dot{\phi}_{1}-\dot{\phi}_{4}}{2}
```

余弦定理，规避 $`\phi_{2},\phi_{3}`$

```math
L_{1}^{2}+l^{2}-2lL_{1}\cos \alpha=L_{2}^2\implies l^2-2lL_{1}\cos \alpha+L_{1}^{2}-L_{2}^{2}=0
```

保留正数解得到

```math
l= L_{1}\cos \alpha+\sqrt{ L_{2}^{2}-L_{1}^2\sin^{2}\alpha }
```

同时

```math
2l\dot{l}-2\dot{l}L_{1}\cos \alpha+2lL_{1}\dot{\alpha}\sin \alpha=0
```

解得

```math
\dot{l}=\frac{lL_{1}\sin \alpha}{L_{1}\cos \alpha-1}\dot{\alpha}=\frac{lL_{1}\sin \alpha}{2(L_{1}\cos \alpha-1)}(\dot{\phi}_{1}-\dot{\phi}_{4})
```

有

```math
J=\left[ \begin{matrix}
\frac{lL_{1}\sin \alpha}{2(L_{1}\cos \alpha-1)} & -\frac{lL_{1}\sin \alpha}{2(L_{1}\cos \alpha-1)} \\
\frac{1}{2}  & \frac{1}{2}
\end{matrix} \right]
```

使得

```math
\left[\begin{matrix}
L \\
\phi
\end{matrix}\right]=J\left[\begin{matrix}
\phi_{1} \\
\phi_{4}
\end{matrix}\right]
```

那么

```math
\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]=J^{T}\left[ \begin{matrix}
F_{L} \\
\tau_{p}
\end{matrix} \right]
```

### F_s

![wbr-spring-3](assets/wbr-spring-3.png)

同样对于气弹簧，也可以用几何关系进行求解， $`d_{1},d_{2},d_{3}`$ 在图纸上均为已知量

```math
\alpha_{s}=\arccos\left( \frac{d_{3}^{2}+L_{1}^{2}-d_{1}^{2}}{2d_{3}L_{1}} \right),\theta=\arccos\left( \frac{L_{1}^{2}+L_{2}^2-l^{2}}{2L_{1}L_{2}} \right),\beta_{s}=\theta-\alpha_{s}
```

```math
\begin{equation}
\left\{
\begin{aligned}
l_{s}^{2}&=d_{2}^2+d_{3}^2-2d_{2}d_{3}\cos \beta_{s} \\
L^2&=L_{1}^2+L_{2}^2-2L_{1}L_{2}\cos \theta
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
2l_{s}\delta l_{s}&=2d_{2}d_{3}\delta s\sin\beta_{s}=2d_{2}d_{3}\delta \theta\sin \beta_{s} \\
2L\delta L&=2L_{1}L_{2}\delta \theta\sin \theta
\end{aligned}
\right.
\end{equation}
```

```math
\begin{equation}
\left\{
\begin{aligned}
\delta l_{s}&=\frac{d_{2}d_{3}\delta \theta\sin \beta_{s}}{l_{s}} \\
\delta L&=\frac{L_{1}L_{2}\delta \theta\sin \theta}{L}
\end{aligned}
\right.
\end{equation}
```

假设气弹簧只提供了推力，由虚功原理

```math
F_{Ls}\delta L=F_{s}\delta l_{s}
```

等效力

```math
F_{Ls}=\frac{\delta l_{s}}{\delta L}F_{s}=\frac{d_{2}d_{3}L\sin \beta_{s}}{L_{1}L_{2}l_{s}\sin \theta}F_{s}
```

这里最后分解到 $`\phi_{1},\phi_{4}`$ 的力矩作用效果可以视为是相同的，但是无论是从安装方式还是非对称性上讲，对 $`\phi_{1},\phi_{4}`$ 的作用效果不应该是一样的，且对等效腿关节的旋转也可能无法忽略。
