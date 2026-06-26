# 物理建模与控制律

物理建模的方式直接决定了控制器的设计和控制上限，对于建模而言，并不是一定是越精确越细致就越好，模型精确依赖数据精确，模型粗糙依赖更多处理，下面的物理建模按照从易到难，从简单到复杂，展现了三个发展阶段。

在三种建模的发展历程中，受力分析，转动分析和加速度分析均大部分可复用，那么这就很好地简化了我们在切换模型时的分析成本，按照顺序阅读三种建模，会对整个系统有更好地认识。下面的建模均是从原来学校的开源发展修改而来，统一并优化了符号定义和坐标系约定，使各种建模具有更好的一致性和连贯性。
## 约定

一切力以竖直向上、水平向左为正方向

| 物理意义      | 数学符号                                            |
| --------- | ----------------------------------------------- |
| 轮主动力矩     | $`\tau_{w,l},\tau_{w,r}`$                         |
| 髋主动力矩     | $`\tau_{l,l},\tau_{l,r}`$                         |
| 腿对机身水平作用力 | $`F_{l,l}^{h},F_{l,r}^{h}`$                         |
| 腿对机身垂直作用力 | $`F_{l,l}^{v},F_{l,r}^{v}`$                         |
| 轮对腿水平作用力  | $`F_{w,l}^{h},F_{w,r}^{h}`$                         |
| 轮对腿垂直作用力  | $`F_{w,l}^{v},F_{w,r}^{v}`$                         |
| 轮质量与惯量    | $`m_{w},I_{w}`$                                   |
| 腿质量与惯量    | $`m_{l},I_{l}`$                                   |
| 机体质量与惯量   | $`m_{b},I_{b}`$                                   |
| 位移相关      | $`x,\dot{x},\ddot{x}`$                            |
| yaw       | $`\phi,\dot{\phi}, \ddot{\phi}`$                  |
| pitch     | $`\theta_{b},\dot{\theta}_{b},\ddot{\theta}_{b}`$ |
| 摆角        | $`\theta_{l},\dot{\theta}_{l},\ddot{\theta}_{l}`$ |
| 轮角        | $`\theta_{w},\dot{\theta}_{w},\ddot{\theta}_{w}`$ |
| 轮转轴到腿转轴距离 | $`l,l_{l},l_{r}`$                                 |
| 轮转轴到腿质心距离 | $`l_{w},l_{w,l},l_{w,r}`$                         |
| 腿转轴到腿质心距离 | $`l_{b},l_{b,l},l_{b,r}`$                         |
| 轮加速度      | $`a_{w}^{h},a_{w}^{v}`$                               |
| 腿加速度      | $`a_{l}^{h},a_{l}^{v}`$                               |
| 机体加速度     | $`a_{b}^{h},a_{b}^{v}`$                               |
| 机体质心偏移    | $`d_{b},\theta_{b}^{0}`$                            |
| 腿质心偏移     | $`d_{l},\theta_{l}^{0}`$                            |

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
a_{w}^{h}=\ddot{x},a_{w}^{v}=0
```
转动分析
```math
I_{w} \ddot{\theta}_{w}=\tau_{w}-fR_{w}
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
a_{l}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l_{w}\sin \theta_{l}) \\\\
=\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}
```
```math
a_{l}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w}\cos \theta_{l})=-l_{w}\sin \theta_{l}\ddot{\theta}_{l}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}
```
转动分析
```math
I_{l} \ddot{\theta}_{l}=\tau_{l}-\tau_{w}+(F_{w}^{v}l_{w}+F_{l}^{v}l_{b})\sin \theta_{l}-(F_{w}^{h}l_{w}+F_{l}^{h}l_{b})\cos \theta_{l}
```

### 机体质心

受力分析
```math
F_{l}^{h}=m_{b}a_{b}^{h}
```
```math
F_{l}^{v}-m_{b}g=m_{b}a_{b}^{v}
```
加速度
```math
a_{b}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l\sin \theta_{l})=\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2}
```
```math
a_{b}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l\cos \theta_{l})=-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}
```
转动分析
```math
I_{b} \ddot{\theta}_{b}=-\tau_{l}+F_{l}^{v}d_{b}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b}
```
### 控制律

我们需要
```math
x=\left[ \begin{matrix}
x \\\\
\dot{x} \\\\
\theta_{l} \\\\
\dot{\theta}_{l} \\\\
\theta_{b} \\\\
\dot{\theta}_{b} \\\\
\end{matrix}
 \right] \\\\
 , \\\\
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\\\
\ddot{x} \\\\
\dot{\theta}_{l} \\\\
\ddot{\theta}_{l} \\\\
\dot{\theta}_{b} \\\\
\ddot{\theta}_{b} \\\\
\end{matrix}
 \right] \\\\
 , \\\\
  u=\left[ \begin{matrix}
\tau_{w} \\\\
\tau_{l} \\\\
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
f & =m_{w}a_{w}^{h}+m_{l}a_{l}^{h}+m_{b}a_{b}^{h} \\\\
 & =m_{w}\ddot{x}+m_{l}(\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2}) \\\\
  & =(m_{w}+m_{l}+m_{b})\ddot{x}+(m_{l}+m_{b})(l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(l_{b}\cos \theta_{l}\ddot{\theta}_{l}-l_{b}\sin \theta_{l}\dot{\theta}_{l}^{2})
\end{aligned}
```
代入
```math
f=\frac{\tau_{w}-I_{w}\ddot{\theta}_{w}}{R_{w}}=\frac{\tau_{w}}{R_{w}}-\frac{I_{w}\ddot{x}}{R_{w}^{2}}
```
得到
```math
\left( m_{w}+m_{l}+m_{b}+\frac{I_{w}}{R_{w}^{2}} \right)\ddot{x}+(m_{l}+m_{b})(l_{w}\cos \theta_{l}\ddot{\theta}_{l}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(l_{b}\cos \theta_{l}\ddot{\theta}_{l}-l_{b}\sin \theta_{l}\dot{\theta}_{l}^{2})-\frac{\tau_{w}}{R_{w}}=0
```


#### 机体转动方程

```math
\begin{aligned}
I_{b} \ddot{\theta}_{b} & =-\tau_{l}+F_{l}^{v}d_{b}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b} \\\\
 & =-\tau_{l}+m_{b}(a_{l}^{v}+g)d_{b}\sin \theta_{b}-m_{b}a_{l}^{h}d_{b}\cos \theta_{b} \\\\
 & =-\tau_{l}+m_{b}(-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g)d_{b}\sin \theta_{b}-m_{b}(\ddot{x}+l\cos \theta_{l}\ddot{\theta}_{l}-l\sin \theta_{l}\dot{\theta}_{l}^{2})\cos \theta_{b}
\end{aligned}
```


#### 腿部转动方程

```math
\begin{aligned}
F_{w}^{v} & =F_{l}^{v}+m_{l}(a_{l}^{v}+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l}^{v}+g) \\\\
 & = m_{b}(-l\sin \theta_{l}\ddot{\theta}_{l}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g)+m_{l}(-l_{w}\sin \theta_{l}\ddot{\theta}_{l}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) 
\end{aligned}
```
```math
F_{w}^{h}=f-m_{w}a_{w}^{h}= \frac{\tau_{w}-I_{w} \ddot{\theta}_{w}}{R_{w}}-m_{w}\ddot{x}
```
得到
```math
\begin{aligned}
I_{l} \ddot{\theta}_{l} & =\tau_{l}-\tau_{w} \\
 & +m_{b}l\sin \theta_{l}(-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & +m_{l}l_{w}\sin \theta_{l}(-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & -l_{w}\cos \theta_{l}(\frac{\tau_{w}-I_{w} \ddot{\theta}_{w}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{w}\cos \theta_{l}(\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2})
\end{aligned}
```


