# 物理建模与控制律

物理建模的方式直接决定了控制器的设计和控制上限，对于建模而言，并不是一定是越精确越细致就越好，模型精确依赖数据精确，模型粗糙依赖更多处理，下面的物理建模按照从易到难，从简单到复杂，展现了三个发展阶段。

在三种建模的发展历程中，受力分析，转动分析和加速度分析均大部分可复用，那么这就很好地简化了我们在切换模型时的分析成本，按照顺序阅读三种建模，会对整个系统有更好地认识。下面的建模均是从原来学校的开源发展修改而来，统一并优化了符号定义和坐标系约定，使各种建模具有更好的一致性和连贯性。

## 约定

一切力以竖直向上、水平向左为正方向，一切旋转以左视图、逆时针为正方向

| 物理意义      | 数学符号                                            |
| --------- | ----------------------------------------------- |
| 轮主动力矩     | $`\tau_{w,l},\tau_{w,r}`$                         |
| 髋主动力矩     | $`\tau_{l,l},\tau_{l,r}`$                         |
| 腿对机身水平作用力 | $`F_{l,l}^{h},F_{l,r}^{h}`$                       |
| 腿对机身垂直作用力 | $`F_{l,l}^{v},F_{l,r}^{v}`$                       |
| 轮对腿水平作用力  | $`F_{w,l}^{h},F_{w,r}^{h}`$                       |
| 轮对腿垂直作用力  | $`F_{w,l}^{v},F_{w,r}^{v}`$                       |
| 轮质量       | $`m_{w}`$                                         |
| 腿质量       | $`m_{l}`$                                         |
| 机体质量      | $`m_{b}`$                                         |
| 位移相关      | $`x,\dot{x},\ddot{x}`$                            |
| yaw       | $`\phi,\dot{\phi}, \ddot{\phi}`$                  |
| pitch     | $`\theta_{b},\dot{\theta}_{b},\ddot{\theta}_{b}`$ |
| 摆角        | $`\theta_{l},\dot{\theta}_{l},\ddot{\theta}_{l}`$ |
| 轮角        | $`\theta_{w},\dot{\theta}_{w},\ddot{\theta}_{w}`$ |
| 轮转轴到腿转轴距离 | $`l,l_{l},l_{r}`$                                 |
| 轮转轴到腿质心距离 | $`l_{w},l_{w,l},l_{w,r}`$                         |
| 腿转轴到腿质心距离 | $`l_{b},l_{b,l},l_{b,r}`$                         |
| 轮加速度      | $`a_{w}^h,a_{w}^v`$                               |
| 腿加速度      | $`a_{l}^h,a_{l}^v`$                               |
| 机体加速度     | $`a_{b}^h,a_{b}^v`$                               |
| 机体质心偏移    | $`d_{b},\theta_{b}^0`$                            |
| 腿质心偏移     | $`d_{l},\theta_{l}^0`$                            |

我们的分析过程遵循自下而上的顺序，保证连贯性和前后的依赖。

## 单腿建模

将双腿等价为一条腿进行统一建模，极大程度地降低了建模复杂度，但同时会带来较多层级的控制器设计，由哈工程王洪玺首次提出。

### 轮质心

受力分析

```math
-F_{w}^{h}+f=m_{w}a_{w}^{h}
```

```math
-F_{w}^{v}+F_{N}-m_{w}g=m_{w}a_{w}^{v}
```

实际上单腿建模下，第二条式子并不需要

加速度

```math
a_{w}^h=\ddot{x},a_{w}^{v}=0
```

转动分析

```math
I \ddot{\theta}_{w}=\tau_{w}-fR_{w}
```

### 腿质心

受力分析

```math
-F_{l}^{h}+F_{w}^{h}=m_{l}a_{l}^{h}
```

```math
-F_{l}^{v}+F_{w}^{v}-m_{l}g=m_{l}a_{l}^{v}
```

加速度

```math
a_{l}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l_{w}\sin \theta_{l})=\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}
```

```math
a_{l}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w}\cos \theta_{l})=-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}
```

转动分析

```math
I \ddot{\theta}_{l}=\tau_{l}-\tau_{w}+(F_{w}^{v}l_{w}+F_{l}^{v}l_{l})\sin \theta_{l}-(F_{w}^{h}l_{w}+F_{l}^{h}l_{l})\cos \theta_{l}
```

### 机体质心

受力分析

```math
F_{l}^{h}=m_{b}a_{b}^{h}
```

```math
F_{l}^{v}-m_{b}g=m_{b}a_{b}^v
```

加速度

```math
a_{b}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l\sin \theta_{l})=\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2}
```

```math
a_{b}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l\cos \theta_{l})=-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}
```

转动分析

```math
I \ddot{\theta}_{b}=-\tau_{l}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+F_{l}^{v}d_{b}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b}
```

### 控制律

我们需要

```math
x=\left[ \begin{matrix}
x \\
\dot{x} \\
\theta_{l} \\
\dot{\theta}_{l} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta}_{l} \\
\ddot{\theta}_{l} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w} \\
\tau_{l}
\end{matrix}
 \right]
```

最终期望整理出

```math
\dot{x}=Ax+Bu
```

消去力

#### 水平运动方程

```math
f=(m_{w}+m_{l}+m_{b})\ddot{x}+(m_{l}l_{w}+m_{b}l)\cos \theta_{l}\ddot{\theta}_{l}-(m_{l}l_{w}+m_{b}l)\sin \theta_{l}\dot{\theta}_{l}^{2}
```

代入 $`f`$ 

```math
\big(m_{w}+m_{l}+m_{b}+\tfrac{I}{R_{w}^{2}}\big)\ddot{x}+(m_{l}l_{w}+m_{b}l)\cos \theta_{l}\ddot{\theta}_{l}=\frac{\tau_{w}}{R_{w}}+(m_{l}l_{w}+m_{b}l)\sin \theta_{l}\dot{\theta}_{l}^{2}
```

小角度线性化

```math
\big(m_{w}+m_{l}+m_{b}+\tfrac{I}{R_{w}^{2}}\big)\ddot{x}+(m_{l}l_{w}+m_{b}l)\ddot{\theta}_{l}=\frac{\tau_{w}}{R_{w}}
```

#### 腿部转动方程

```math
\begin{aligned}

I\ddot{\theta}_{l}=&\,\tau_{l}-\tau_{w} \\

&+\Big[(m_{l}+m_{b})g+m_{l}\big(-l_{w}\sin \theta_{l}\ddot{\theta}_{l}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}\big(-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]l_{w}\sin \theta_{l} \\

&+m_{b}\Big(g-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}\Big)l_{b}\sin \theta_{l} \\

&-\Big[m_{l}l_{w}\big(\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}\big)+m_{b}(l_{w}+l_{b})\big(\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2}\big)\Big]\cos \theta_{l}

\end{aligned}
```

小角度线性化

```math
\big[I+m_{l}l_{w}^{2}+m_{b}l(l_{w}+l_{b})\big]\ddot{\theta}_{l}+(m_{l}l_{w}+m_{b}l)\ddot{x}=\tau_{l}-\tau_{w}+\big[(m_{l}+m_{b})gl_{w}+m_{b}gl_{b}\big]\theta_{l}
```

#### 机体转动方程

```math
\begin{aligned}

I\ddot{\theta}_{b}=&\,-\tau_{l}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\

&+m_{b}\big(l\sin\theta_{l}\ddot{\theta}_{l}+l\cos\theta_{l}\dot{\theta}_{l}^{2}\big)d_{b}\sin\theta_{b} \\

&-m_{b}\big(\ddot{x}+l\cos\theta_{l}\ddot{\theta}_{l}-l\sin\theta_{l}\dot{\theta}_{l}^{2}\big)d_{b}\cos\theta_{b}

\end{aligned}
```

小角度线性化

```math
I\ddot{\theta}_{b}+m_{b}ld_{b}\ddot{\theta}_{l}=-\tau_{l}-m_{b}gd_{b}+m_{b}gd_{b}\theta_{b}-m_{b}d_{b}\ddot{x}
```

### 标准动力学

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
```

$`C`$ 汇集 $`\dot{\theta}^{2}`$ 离心项，此处不展开 $`M,B`$ 元素，只给维数与 $`G`$ 。 
$`q=[x,\theta_{l},\theta_{b}]^{T}`$ ， $`\tau=[\tau_{w},\tau_{l}]^{T}`$ ， $`\ddot{\theta}_{w}=\ddot{x}/R_w`$ 。$`M`$： $`3\times 3`$ ；$`B`$： $`3\times 2`$

```math
G=\begin{bmatrix}

0 \\

\big[(m_{l}+m_{b})gl_{w}+m_{b}gl_{b}\big]\sin\theta_{l} \\

m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}gd_{b}\sin\theta_{b}

\end{bmatrix}
```

## 双腿建模

将双腿分开建模，并将yaw实际作为状态变量进行计算，能够对双腿实现更精细的控制，并消除了调节劈叉环的过程，由上交首次提出，和单腿建模均为当前最主流建模。

### 轮质心

左右轮对称

受力分析

```math
-F_{w,i}^{h}+f_{i}=m_{w}a_{w,i}^{h}
```

```math
-F_{w,i}^{v}+F_{N,i}-m_{w}g=m_{w}a_{w,i}^{v}
```

加速度

```math
a_{w,i}^h=\ddot{x},a_{w,i}^{v}=0
```

转动分析

```math
I \ddot{\theta}_{w,i}=\tau_{w,i}-f_{i}R_{w}
```

#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```

### 腿质心

左右腿对称

受力分析

```math
-F_{l,i}^{h}+F_{w,i}^{h}=m_{l}a_{l,i}^{h}
```

```math
-F_{l,i}^{v}+F_{w,i}^{v}-m_{l}g=m_{l}a_{l,i}^{v}
```

加速度

```math
a_{l,i}^{h}=a_{w,i}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,i}\sin \theta_{l,i})=\ddot{x}+l_{w,i}\cos \theta_{l,i}\ddot{\theta}_{l,i}-l_{w,i}\sin \theta_{l,i}\dot{\theta}_{l,i}^{2}
```

```math
a_{l,i}^{v}=a_{w,i}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,i}\cos \theta_{l,i})=-l_{w,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-l_{w,i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}
```

转动分析

```math
I_{l,i}\ddot{\theta}_{l,i}=\tau_{l,i}-\tau_{w,i}+(F_{w,i}^{v}l_{w,i}+F_{l,i}^{v}l_{b,i})\sin \theta_{l,i}-(F_{w,i}^{h}l_{w,i}+F_{l,i}^{h}l_{b,i})\cos \theta_{l,i}
```

### 机体质心

```math
F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^{h}
```

```math
F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}
```

加速度

```math
\begin{aligned}
a_{b}^{h} & =a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } \frac{1}{2}(l_{l}\sin \theta_{l,l}+l_{r}\sin \theta_{l,r}) \\
 & =\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{aligned}
```

```math
\begin{aligned}
a_{b}^{v} & =a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } \frac{1}{2}(l_{l} \cos \theta_{l,l}+l_{r}\cos \theta_{l,r}) \\
 & =-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{aligned}
```

转动分析

```math
I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
```

```math
I_{\phi}\ddot{\phi }=(f_{r}-f_{l})R_{b}
```

### 控制律

我们需要

```math
x=\left[ \begin{matrix}
x \\
\dot{x} \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w,l}  \\
\tau_{w,r} \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
```

最终期望整理出

```math
\dot{x}=Ax+Bu
```

消去力

#### 水平运动方程

```math
\begin{aligned}

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{w,l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big) \\
=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{w,l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{w,r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

\end{aligned}
```

小角度线性化

```math
\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{w,l}\ddot{\theta}_{l,l}+l_{w,r}\ddot{\theta}_{l,r}\big)=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}
```

#### yaw转动方程

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]
```

无需进一步线性化

#### 腿部转动方程

左腿

```math
\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}+\tfrac{m_b+2m_l}{2}gl_l\sin\theta_{l,l}-m_l l_{b,l}g\sin\theta_{l,l} \\

&+m_l l_{b,l}l_{w,l}\sin^2\theta_{l,l}\,\ddot{\theta}_{l,l}+m_l l_{b,l}l_{w,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2} \\

&-\tfrac{m_b}{4}l_l\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_l\sin\theta_{l,l}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,l}\cos\theta_{l,l}-(m_w-m_l)\ddot{x}\,l_{b,l}\cos\theta_{l,l}+m_l l_{w,l}l_{b,l}\cos^2\theta_{l,l}\,\ddot{\theta}_{l,l}-m_l l_{w,l}l_{b,l}\sin\theta_{l,l}\cos\theta_{l,l}\,\dot{\theta}_{l,l}^{2}

\end{aligned}
```

小角度线性化

```math
\begin{aligned}

\big[I_{l,l}+m_{l}l_{w,l}l_{b,l}\big]\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-(m_w l_l-m_l l_{b,l})\ddot{x}+\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,l}+\tfrac{m_{b}}{2}gl_{b,l}\Big]\theta_{l,l}

\end{aligned}
```

右腿

```math
\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}+\tfrac{m_b+2m_l}{2}gl_r\sin\theta_{l,r}-m_l l_{b,r}g\sin\theta_{l,r} \\

&+m_l l_{b,r}l_{w,r}\sin^2\theta_{l,r}\,\ddot{\theta}_{l,r}+m_l l_{b,r}l_{w,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2} \\

&-\tfrac{m_b}{4}l_r\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_r\sin\theta_{l,r}\big[l_{w,l}\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_{w,l}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_{w,r}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-m_w\ddot{x}\,l_{w,r}\cos\theta_{l,r}-(m_w-m_l)\ddot{x}\,l_{b,r}\cos\theta_{l,r}+m_l l_{w,r}l_{b,r}\cos^2\theta_{l,r}\,\ddot{\theta}_{l,r}-m_l l_{w,r}l_{b,r}\sin\theta_{l,r}\cos\theta_{l,r}\,\dot{\theta}_{l,r}^{2}

\end{aligned}
```

小角度线性化

```math
\big[I_{l,r}+m_{l}l_{w,r}l_{b,r}\big]\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-(m_w l_r-m_l l_{b,r})\ddot{x}+\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,r}+\tfrac{m_{b}}{2}gl_{b,r}\Big]\theta_{l,r}
```

#### 机体转动方程

```math
\begin{aligned}

I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\

&+m_{b}\Big[\frac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\frac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\frac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\

&-m_{b}\Big[\ddot{x}+\frac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\frac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\frac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}

\end{aligned}
```

小角度线性化

```math
I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}+m_{b}gd_{b}\theta_{b}-m_{b}d_{b}\ddot{x}-\frac{m_{b}d_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)
```

### 标准动力学

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
```

$`C`$ 汇集 $`\dot{\theta}^{2}`$ 离心项，此处不展开 $`M,B`$ 元素，只给维数与 $`G`$ 。
 
$`q=[\theta_{w,l},\theta_{w,r},\theta_{l,l},\theta_{l,r},\theta_{b}]^{T}`$ ， $`\tau=[\tau_{w,l},\tau_{w,r},\tau_{l,l},\tau_{l,r}]^{T}`$ 。$`M`$ ： $`5\times 5`$ ；$`B`$： $`5\times 4`$

```math
G=\begin{bmatrix}
0 \\
0 \\
\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,l}+\tfrac{m_{b}}{2}gl_{b,l}\Big]\sin\theta_{l,l} \\
\Big[\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{w,r}+\tfrac{m_{b}}{2}gl_{b,r}\Big]\sin\theta_{l,r} \\
m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}gd_{b}\sin\theta_{b}
\end{bmatrix}
```

## 双腿质心偏移建模

将偏置并联构型的腿部质心靠后问题纳入物理建模中，通过对平衡点的显式计算得到了更精确的平衡位置与控制效果，由港大首次提出，是新兴的建模方法。同时将原先的参考质心改为参考转轴，简化了对腿部的分析，也是最符合质心偏移的参考方式。

这里的小角度线性化实际不发挥作用，准确的计算应当将角度带入到平衡点进行计算。精确的平衡点强依赖对模型的标注，标注不好可能最终得到的结果并不如人意。

### 轮转轴

左右轮对称，轮转轴与轮质心相同

受力分析

```math
-F_{w,i}^{h}+f_{i}=m_{w}a_{w,i}^{h}
```

```math
-F_{w,i}^{v}+F_{N,i}-m_{w}g=m_{w}a_{w,i}^{v}
```

加速度

```math
a_{w,i}^h=\ddot{x},a_{w,i}^{v}=0
```

转动分析

```math
I \ddot{\theta}_{w,i}=\tau_{w,i}-f_{i}R_{w}
```

#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```

### 腿转轴

左右腿对称

受力分析

```math
-F_{l,i}^{h}+F_{w,i}^{h}=m_{l}a_{l,i}^{h}
```

```math
-F_{l,i}^{v}+F_{w,i}^{v}-m_{l}g=m_{l}a_{l,i}^{v}
```

加速度

```math
a_{l,i}^{h}=a_{w,i}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{i}\sin \theta_{l,i})=\ddot{x}+l_{i}\cos \theta_{l,i}\ddot{\theta}_{l,i}-l_{i}\sin \theta_{l,i}\dot{\theta}_{l,i}^{2}
```

```math
a_{l,i}^{v}=a_{w,i}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{i}\cos \theta_{l,i})=-l_{i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-l_{i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}
```

转动分析

```math
I_{l,i}\ddot{\theta}_{l,i}=\tau_{l,i}-\tau_{w,i}-m_{l}gd_{l}\sin(\theta_{l,i}+\theta_{l,i}^{0})-F_{w,i}^{h}l_{i}\cos \theta_{l,i}+F_{w,i}^{v}l_{i}\sin \theta_{l,i}
```

### 机体转轴

受力分析

```math
F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^h
```

```math
F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}
```

加速度

```math
a_{b}^{h}=\frac{1}{2}(a_{l,l}^{h}+a_{l,r}^{h})=\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```

```math
a_{b}^{v}=\frac{1}{2}(a_{l,l}^{v}+a_{l,r}^{v})=-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```

转动分析

```math
I_{b}\ddot{\theta}_{b}=-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
```

```math
I_{\phi}\ddot{\phi }=(f_{r}-f_{l})R_{b}
```

### 控制律

我们需要

```math
x=\left[ \begin{matrix}
x \\
\dot{x} \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w,l}  \\
\tau_{w,r} \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
```

最终期望整理出

```math
\dot{x}=Ax+Bu
```

消去力

#### 水平运动方程

```math
\begin{aligned}

\big(2m_{w}+2m_{l}+m_{b}+\tfrac{2I}{R_{w}^{2}}\big)\ddot{x}+\frac{m_{b}}{2}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big)+m_{l}\big(l_{l}\cos\theta_{l,l}\ddot{\theta}_{l,l}+l_{r}\cos\theta_{l,r}\ddot{\theta}_{l,r}\big) \\
=\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}+m_{b}\Big[-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]+m_{l}\Big[-l_{l}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]

\end{aligned}
```

#### yaw转动方程

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]
```

#### 腿部转动方程

左腿

```math
\begin{aligned}

I_{l,l}\ddot{\theta}_{l,l}=&\,\tau_{l,l}-\tau_{w,l}\big(1+\tfrac{l_l}{R_w}\big)+\tfrac{I}{R_w}l_l\ddot{\theta}_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-m_{w}\ddot{x}\,l_{l}\cos\theta_{l,l} \\

&+\tfrac{m_b+2m_l}{2}gl_{l}\sin\theta_{l,l}-\tfrac{m_b}{4}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{l}\sin\theta_{l,l}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}
```

右腿

```math
\begin{aligned}

I_{l,r}\ddot{\theta}_{l,r}=&\,\tau_{l,r}-\tau_{w,r}\big(1+\tfrac{l_r}{R_w}\big)+\tfrac{I}{R_w}l_r\ddot{\theta}_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-m_{w}\ddot{x}\,l_{r}\cos\theta_{l,r} \\

&+\tfrac{m_b+2m_l}{2}gl_{r}\sin\theta_{l,r}-\tfrac{m_b}{4}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big] \\

&-\tfrac{m_l}{2}l_{r}\sin\theta_{l,r}\big[l_l\sin\theta_{l,l}\ddot{\theta}_{l,l}+l_l\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+l_r\sin\theta_{l,r}\ddot{\theta}_{l,r}+l_r\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\big]

\end{aligned}
```

#### 机体转动方程

```math
\begin{aligned}
I_{b}\ddot{\theta}_{b}=&\,-\tau_{l,l}-\tau_{l,r}-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}g\sin\theta_{b} \\
&+m_{b}\Big[\tfrac{l_{l}}{2}\sin\theta_{l,l}\ddot{\theta}_{l,l}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\sin\theta_{l,r}\ddot{\theta}_{l,r}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]\sin\theta_{b} \\
&-m_{b}\Big[\ddot{x}+\tfrac{l_{l}}{2}\cos\theta_{l,l}\ddot{\theta}_{l,l}-\tfrac{l_{l}}{2}\sin\theta_{l,l}\dot{\theta}_{l,l}^{2}+\tfrac{l_{r}}{2}\cos\theta_{l,r}\ddot{\theta}_{l,r}-\tfrac{l_{r}}{2}\sin\theta_{l,r}\dot{\theta}_{l,r}^{2}\Big]d_{b}\cos\theta_{b}
\end{aligned}
```

### 标准动力学

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
```

$`C`$ 汇集 $\dot{\theta}^{2}$ 离心项，此处不展开 $`M,B`$ 元素，只给维数与 $`G`$ 。

$`q=[\theta_{w,l},\theta_{w,r},\theta_{l,l},\theta_{l,r},\theta_{b}]^{T}`$ ， $`\tau=[\tau_{w,l},\tau_{w,r},\tau_{l,l},\tau_{l,r}]^{T}`$ 。$`M`$： $`5\times 5`$ ；$`B`$： $`5\times 4`$

```math
G=\begin{bmatrix}
0 \\
0 \\
\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\sin\theta_{l,l}+m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}) \\
\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\sin\theta_{l,r}+m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0}) \\
m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}gd_{b}\sin\theta_{b}
\end{bmatrix}
```

#### 平衡点计算

对于 $`q`$ 在平衡点，满足 $`G(q)=0`$ ，解得

```math
\theta_{l,l}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,l}^0}{\left(m_l + \frac{m_b}{2}\right) l_l + m_l d_l \cos\theta_{l,l}^0} \right)
```

```math
\theta_{l,r}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,r}^0}{\left(m_l + \frac{m_b}{2}\right) l_r + m_l d_l \cos\theta_{l,r}^0} \right)
```

```math
\theta_{b} = \dfrac{\pi}{4} - \dfrac{\theta_{b}^0}{2}
```

测算之后将平衡点代入式子，计算得到 $`A,B`$ 矩阵
