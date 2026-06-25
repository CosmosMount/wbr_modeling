1. [约定](#%E7%BA%A6%E5%AE%9A)
2. [单腿建模](#%E5%8D%95%E8%85%BF%E5%BB%BA%E6%A8%A1)
	1. [轮质心](#%E8%BD%AE%E8%B4%A8%E5%BF%83)
	2. [腿质心](#%E8%85%BF%E8%B4%A8%E5%BF%83)
	3. [机体质心](#%E6%9C%BA%E4%BD%93%E8%B4%A8%E5%BF%83)
	4. [控制律](#%E6%8E%A7%E5%88%B6%E5%BE%8B)
		1. [水平运动方程](#%E6%B0%B4%E5%B9%B3%E8%BF%90%E5%8A%A8%E6%96%B9%E7%A8%8B)
		2. [机体转动方程](#%E6%9C%BA%E4%BD%93%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		3. [腿部转动方程](#%E8%85%BF%E9%83%A8%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
3. [双腿建模](#%E5%8F%8C%E8%85%BF%E5%BB%BA%E6%A8%A1)
	1. [轮质心](#%E8%BD%AE%E8%B4%A8%E5%BF%83)
		1. [左轮](#%E5%B7%A6%E8%BD%AE)
		2. [右轮](#%E5%8F%B3%E8%BD%AE)
		3. [左右支持力](#%E5%B7%A6%E5%8F%B3%E6%94%AF%E6%8C%81%E5%8A%9B)
	2. [腿质心](#%E8%85%BF%E8%B4%A8%E5%BF%83)
		1. [左腿](#%E5%B7%A6%E8%85%BF)
		2. [右腿](#%E5%8F%B3%E8%85%BF)
	3. [机体质心](#%E6%9C%BA%E4%BD%93%E8%B4%A8%E5%BF%83)
	4. [控制律](#%E6%8E%A7%E5%88%B6%E5%BE%8B)
		1. [水平转动方程](#%E6%B0%B4%E5%B9%B3%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		2. [yaw转动方程](#yaw%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		3. [机体转动方程](#%E6%9C%BA%E4%BD%93%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		4. [腿部转动方程](#%E8%85%BF%E9%83%A8%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
4. [双腿质心偏移建模](#%E5%8F%8C%E8%85%BF%E8%B4%A8%E5%BF%83%E5%81%8F%E7%A7%BB%E5%BB%BA%E6%A8%A1)
	1. [轮转轴](#%E8%BD%AE%E8%BD%AC%E8%BD%B4)
		1. [左轮](#%E5%B7%A6%E8%BD%AE)
		2. [右轮](#%E5%8F%B3%E8%BD%AE)
		3. [左右支持力](#%E5%B7%A6%E5%8F%B3%E6%94%AF%E6%8C%81%E5%8A%9B)
	2. [腿转轴](#%E8%85%BF%E8%BD%AC%E8%BD%B4)
		1. [左腿](#%E5%B7%A6%E8%85%BF)
		2. [右腿](#%E5%8F%B3%E8%85%BF)
	3. [机体转轴](#%E6%9C%BA%E4%BD%93%E8%BD%AC%E8%BD%B4)
	4. [控制律](#%E6%8E%A7%E5%88%B6%E5%BE%8B)
		1. [水平运动方程](#%E6%B0%B4%E5%B9%B3%E8%BF%90%E5%8A%A8%E6%96%B9%E7%A8%8B)
		2. [yaw转动方程](#yaw%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		3. [机体转动方程](#%E6%9C%BA%E4%BD%93%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
		4. [腿部转动方程](#%E8%85%BF%E9%83%A8%E8%BD%AC%E5%8A%A8%E6%96%B9%E7%A8%8B)
	5. [标准动力学](#%E6%A0%87%E5%87%86%E5%8A%A8%E5%8A%9B%E5%AD%A6)
		1. [平衡点计算](#%E5%B9%B3%E8%A1%A1%E7%82%B9%E8%AE%A1%E7%AE%97)


物理建模的方式直接决定了控制器的设计和控制上限，对于建模而言，并不是一定是越精确越细致就越好，模型精确依赖数据精确，模型粗糙依赖更多处理，下面的物理建模按照从易到难，从简单到复杂，展现了三个发展阶段。

在三种建模的发展历程中，受力分析，转动分析和加速度分析均大部分可复用，那么这就很好地简化了我们在切换模型时的分析成本，按照顺序阅读三种建模，会对整个系统有更好地认识。下面的建模均是从原来学校的开源发展修改而来，统一并优化了符号定义和坐标系约定，使各种建模具有更好的一致性和连贯性。
## 约定

一切力以竖直向上、水平向左为正方向

| 物理意义      | 数学符号                                            |
| --------- | ----------------------------------------------- |
| 轮主动力矩     | $`\tau_{w,l},\tau_{w,r}`$                         |
| 髋主动力矩     | $`\tau_{l,l},\tau_{l,r}`$                         |
| 腿对机身水平作用力 | $`F_{l,l}^{h},F_{l,r}^h`$                         |
| 腿对机身垂直作用力 | $`F_{l,l}^{v},F_{l,r}^v`$                         |
| 轮对腿水平作用力  | $`F_{w,l}^{h},F_{w,r}^h`$                         |
| 轮对腿垂直作用力  | $`F_{w,l}^{v},F_{w,r}^v`$                         |
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
a_{l}^{h}=a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } (l_{w}\sin \theta_{l})
=\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2}
```
```math
a_{l}^{v}=a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w}\cos \theta_{l})=-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}
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
I_{b} \ddot{\theta}_{b}=-\tau_{l}+F_{l}^{v}d_{b}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b}
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
\begin{align}
f & =m_{w}a_{w}^{h}+m_{l}a_{l}^{h}+m_{b}a_{b}^{h} \\
 & =m_{w}\ddot{x}+m_{l}(\ddot{x}+l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2}) \\
  & =(m_{w}+m_{l}+m_{b})\ddot{x}+(m_{l}+m_{b})(l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(l_{b}\cos \theta_{l}\ddot{\theta}_{l}-l_{b}\sin \theta_{l}\dot{\theta}_{l}^2)
\end{align}
```
代入
```math
f=\frac{\tau_{w}-I_{w}\ddot{\theta}_{w}}{R_{w}}=\frac{\tau_{w}}{R_{w}}-\frac{I_{w}\ddot{x}}{R_{w}^{2}}
```
得到
```math
\left( m_{w}+m_{l}+m_{b}+\frac{I_{w}}{R_{w}^{2}} \right)\ddot{x}+(m_{l}+m_{b})(l_{w}\cos \theta_{l}\ddot{\theta_{l}}-l_{w}\sin \theta_{l}\dot{\theta}_{l}^{2})+m_{b}(l_{b}\cos \theta_{l}\ddot{\theta}_{l}-l_{b}\sin \theta_{l}\dot{\theta}_{l}^2)-\frac{\tau_{w}}{R_{w}}=0
```


小角度线性化（$`\sin\theta\approx\theta`$ ，$`\cos\theta\approx 1`$ ，忽略 $`\dot{\theta}^{2}`$ 项）：

```math
\left( m_{w}+m_{l}+m_{b}+\frac{I_{w}}{R_{w}^{2}} \right)\ddot{x}+\big((m_{l}+m_{b})l_{w}+m_{b}l_{b}\big)\ddot{\theta}_{l}-\frac{\tau_{w}}{R_{w}}=0
```

#### 机体转动方程

```math
\begin{align}
I_{b} \ddot{\theta}_{b} & =-\tau_{l}+F_{l}^{v}d_{b}\sin \theta_{b}-F_{l}^{h}d_{b}\cos \theta_{b} \\
 & =-\tau_{l}+m_{b}(a_{l}^{v}+g)d_{b}\sin \theta_{b}-m_{b}a_{l}^{h}d_{b}\cos \theta_{b} \\
 & =-\tau_{l}+m_{b}(-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g)d_{b}\sin \theta_{b}-m_{b}(\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2})\cos \theta_{b}
\end{align}
```


小角度线性化：

```math
I_{b}\ddot{\theta}_{b}=-\tau_{l}+m_{b}gd_{b}\theta_{b}-m_{b}\ddot{x}-m_{b}l\ddot{\theta}_{l}
```

#### 腿部转动方程

```math
\begin{align}
F_{w}^{v} & =F_{l}^{v}+m_{l}(a_{l}^v+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l}^{v}+g) \\
 & = m_{b}(-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g)+m_{l}(-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & =(m_{b}+m_{l})(-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}+g)+m_{b}(-l_{b}\sin \theta_{l}\ddot{\theta_{l}}-l_{b}\cos \theta_{l}\dot{\theta}_{l}^{2})
\end{align}
```
```math
F_{w}^{h}=f-m_{w}a_{w}^{h}= \frac{\tau_{w}-I_{w} \ddot{\theta}_{w}}{R_{w}}-m_{w}\ddot{x}
```
得到
```math
\begin{align}
I_{l} \ddot{\theta}_{l} & =\tau_{l}-\tau_{w} \\
 & +l_{w}\cos \theta_{l}((m_{b}+m_{l})(-l_{w}\sin \theta_{l}\ddot{\theta_{l}}-l_{w}\cos \theta_{l}\dot{\theta}_{l}^{2}+g)+m_{b}(-l_{b}\sin \theta_{l}\ddot{\theta_{l}}-l_{b}\cos \theta_{l}\dot{\theta}_{l}^{2}+g)) \\
 & +m_{b}l_{b}\sin \theta_{l}(-l\sin \theta_{l}\ddot{\theta_{l}}-l\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & -l_{w}\cos \theta_{l}(\frac{\tau_{w}-I_{w} \ddot{\theta}_{w}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{w}\cos \theta_{l}(\ddot{x}+l\cos \theta_{l}\ddot{\theta_{l}}-l\sin \theta_{l}\dot{\theta}_{l}^{2})
\end{align}
```


小角度线性化：

```math
\begin{align}
(I_{l}+m_{b}l\,l_{w})\ddot{\theta}_{l} & =\tau_{l}-\tau_{w}\Big(1+\frac{l_{w}}{R_{w}}\Big)+(m_{w}-m_{b})l_{w}\ddot{x}-m_{b}gl_{b}\theta_{l} \\
 & +gl_{w}(2m_{b}+m_{l})
\end{align}
```

其中 $`gl_{w}(2m_{b}+m_{l})`$ 为平衡常数项，在平衡点处由关节力矩平衡；整理 $`\dot{x}=Ax+Bu`$ 时消去即可。

## 双腿建模

将双腿分开建模，并将yaw实际作为状态变量进行计算，能够对双腿实现更精细的控制，并消除了调节劈叉环的过程，由上交首次提出，和单腿建模均为当前最主流建模。

### 轮质心

左右轮对称

#### 左轮

受力分析

```math
-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h}
```
```math
-F_{w,l}^{v}+F_{N,l}-m_{w}g=m_{w}a_{w,l}^{v}
```
加速度

```math
a_{w,l}^h=\ddot{x},a_{w,l}^{v}=0
```
转动分析
```math
I_{w} \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}
```

#### 右轮

```math
-F_{w,r}^{h}+f_{l}=m_{w}a_{w,r}^{h}
```
```math
-F_{w,r}^{v}+F_{N,r}-m_{w}g=m_{w}a_{w,r}^{v}
```
加速度

```math
a_{w,r}^h=\ddot{x},a_{w,r}^{v}=0
```
转动分析
```math
I_{w} \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}
```
#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```
### 腿质心

左右腿对称

#### 左腿

受力分析
```math
-F_{l,l}^{h}+F_{w,l}^{h}=m_{l}a_{l,l}^{h}
```
```math
-F_{l,l}^{v}+F_{w,l}^{v}-m_{l}g=m_{l}a_{l,l}^{v}
```
加速度
```math
a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\sin \theta_{l,l})=\ddot{x}+l_{w,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
```math
a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\cos \theta_{l,l})=-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
转动分析
```math
I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}+(F_{w,l}^{v}l_{w,l}+F_{l,l}^{v}l_{b,l})\sin \theta_{l,l}-(F_{w,l}^{h}l_{w,l}+F_{l,l}^{h}l_{b,l})\cos \theta_{l,l}
```
#### 右腿
```math
-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h}
```
```math
-F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}
```
加速度
```math
a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\sin \theta_{l,r})=\ddot{x}+l_{w,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
```math
a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\cos \theta_{l,r})=-l_{w,r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
转动分析
```math
I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}+(F_{w,r}^{v}l_{w,r}+F_{l,r}^{v}l_{b,r})\sin \theta_{l,r}-(F_{w,r}^{h}l_{w,r}+F_{l,r}^{h}l_{b,r})\cos \theta_{l,r}
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

\begin{align}
a_{b}^{h} & =a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } \frac{1}{2}(l_{l}\sin \theta_{l,l}+l_{r}\sin \theta_{l,r}) \\
 & =\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{align}
```
```math
\begin{align}
a_{b}^{v} & =a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } \frac{1}{2}(l_{l} \cos \theta_{l,l}+l_{r}\cos \theta_{l,r}) \\
 & =-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{align}
```
转动分析
```math
I_{b}\ddot{\theta}_{b}=-(\tau_{l,l}+\tau_{l,r})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
```
```math
I_{\phi}\ddot{\phi }=(f_{r}-f_{l})R_{b}
```
### 控制律

我们需要
```math
x=\left[ \begin{matrix}
x \\
\dot{x}  \\
\phi \\
\dot{\phi} \\
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
\ddot{x}  \\
\dot{\phi} \\
\ddot{\phi} \\
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

#### 水平转动方程

```math
f  =m_{w,l}a_{w,l}^{h}+m_{w,r}a_{w,r}^{h}+m_{l,l}a_{l,l}^{h}+m_{l,r}a_{l,r}^{h}+m_{b}a_{b}^{h} 
```
代入 $`f`$
```math
\begin{align}
\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}} = &  \left( 2m_{w}+2m_{l}+m_{b}+\frac{2I_{w}}{R_{w}^{2}} \right)\ddot{x}\\
 & +\left( m_{l}+\frac{m_{b}}{2} \right)(l_{w,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}) \\
 & +\frac{m_{b}}{2}(l_{b,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{b,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^2+l_{b,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{b,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^2)
\end{align}
```


小角度线性化：

```math
\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}=\left(2m_{w}+2m_{l}+m_{b}+\frac{2I_{w}}{R_{w}^{2}}\right)\ddot{x}+\left(m_{l}+\frac{m_{b}}{2}\right)\big(l_{w,l}\ddot{\theta}_{l,l}+l_{w,r}\ddot{\theta}_{l,r}\big)+\frac{m_{b}}{2}\big(l_{b,l}\ddot{\theta}_{l,l}+l_{b,r}\ddot{\theta}_{l,r}\big)
```

#### yaw转动方程

```math

I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]

```


小角度线性化（滚动约束 $`\ddot{\theta}_{w}=\ddot{x}/R_{w}`$ ，$`I\ddot{\theta}_{w}`$ 项相消）：

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\big(\tau_{w,r}-\tau_{w,l}\big)
```
#### 机体转动方程

```math
\begin{align}
I_{b} \ddot{\theta}_{b} = & -(\tau_{l,l}+\tau_{l,r})+m_{b}gd_{b}\sin \theta_{b}-m_{b}\ddot{x}\cos \theta_{b} \\
 & +\frac{m_{b}}{2}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})d_{b}\sin \theta_{b} \\
 & -\frac{m_{b}}{2}(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})\cos \theta_{b}
\end{align}
```


小角度线性化：

```math
I_{b}\ddot{\theta}_{b}=-(\tau_{l,l}+\tau_{l,r})+m_{b}gd_{b}\theta_{b}-m_{b}\ddot{x}-\frac{m_{b}}{2}\big(l_{l}\ddot{\theta}_{l,l}+l_{r}\ddot{\theta}_{l,r}\big)
```
#### 腿部转动方程

```math
\begin{align}
F_{w,l}^{v} & =F_{l,l}^{v}+m_{l}(a_{l,l}^v+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,l}^{v}+g) \\
 & = \frac{1}{2}m_{b}\left( -\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g \right)+m_{l}(-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l}\dot{\theta}_{l}^{2}+g)
\end{align}
```

```math
\begin{align}
I_{l,l} \ddot{\theta}_{l,l} = & \tau_{l,l}-\tau_{w,l} \\
 & +\frac{1}{2}m_{b}l_{w,l}\sin \theta_{l,l}(-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & + \frac{1}{2}m_{l}l_{w,l}\sin \theta_{l,l}(-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & + \frac{1}{2}m_{b}l_{b,l}\sin \theta_{l,l}(-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & -l_{w,l}\cos \theta_{l,l}(\frac{\tau_{w,l}-I_{w} \ddot{\theta}_{w,l}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{b,l}\cos \theta_{l,l}(\ddot{x}+l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2})
\end{align}
```


小角度线性化（左腿）：

```math
\begin{align}
(I_{l,l}+m_{b}l_{l}l_{b,l})\ddot{\theta}_{l,l} & =\tau_{l,l}-\tau_{w,l}\Big(1+\frac{l_{w,l}}{R_{w}}\Big)+(m_{w}-m_{b})l_{w,l}\ddot{x} \\
 & +\frac{m_{b}g}{2}\big(l_{b,l}+l_{w,l}\big)\theta_{l,l}+\frac{m_{l}g}{2}l_{w,l}\theta_{l,l}
\end{align}
```

```math
\begin{align}
I_{l,r} \ddot{\theta}_{l,r} = & \tau_{l,r}-\tau_{w,r} \\
 & +\frac{1}{2}m_{b}l_{w,r}\sin \theta_{l,r}(-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{l}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & + \frac{1}{2}m_{l}l_{w,r}\sin \theta_{l,r}(-l_{w,r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & + \frac{1}{2}m_{b}l_{b,r}\sin \theta_{l,r}(-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{l}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & -l_{w,r}\cos \theta_{l,r}(\frac{\tau_{w,r}-I_{w} \ddot{\theta}_{w,r}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{b,r}\cos \theta_{l,r}(\ddot{x}+l_{l}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{l}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{align}
```


小角度线性化（右腿）：

```math
\begin{align}
(I_{l,r}+m_{b}l_{l}l_{b,r})\ddot{\theta}_{l,r} & =\tau_{l,r}-\tau_{w,r}\Big(1+\frac{l_{w,r}}{R_{w}}\Big)+(m_{w}-m_{b})l_{w,r}\ddot{x} \\
 & +\frac{m_{b}g}{2}\big(l_{b,r}+l_{w,r}\big)\theta_{l,r}+\frac{m_{l}g}{2}l_{w,r}\theta_{l,r}
\end{align}
```
## 双腿质心偏移建模

将偏置并联构型的腿部质心靠后问题纳入物理建模中，通过对平衡点的显式计算得到了更精确的平衡位置与控制效果，由港大首次提出，是新兴的建模方法。同时将原先的参考质心改为参考转轴，简化了对腿部的分析，也是最符合质心偏移的参考方式。

这里的小角度线性化实际不发挥作用，准确的计算应当将角度带入到平衡点进行计算。精确的平衡点强依赖对模型的标注，标注不好可能最终得到的结果并不如人意。

### 轮转轴

左右轮对称，轮转轴与轮质心相同

#### 左轮

受力分析

```math
-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h}
```
```math
-F_{w,l}^{v}+F_{N,l}-m_{w}g=a_{w,l}^{v}
```
加速度
```math
a_{w,l}^h=\ddot{x},a_{w,l}^{v}=0
```
转动分析
```math
I \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}
```

#### 右轮

```math
-F_{w,r}^{h}+f_{l}=m_{w}a_{w,r}^{h}
```
```math
-F_{w,r}^{v}+F_{N,r}-m_{w}g=m_{w}a_{w,r}^{v}
```
加速度

```math
a_{w,r}^h=\ddot{x},a_{w,r}^{v}=0
```
转动分析
```math
I \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}
```
#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```
### 腿转轴

左右腿对称

#### 左腿

受力分析
```math
-F_{l,l}^{h}+F_{w,l}^{h}=m_{l}a_{l,l}^{h}
```
```math
-F_{l,l}^{v}+F_{w,l}^{v}-m_{l}g=m_{l}a_{l,l}^{v}
```
加速度
```math
a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\sin \theta_{l,l})=\ddot{x}+l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
```math
a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\cos \theta_{l,l})=-l_{l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}
```

转动分析
```math
I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-F_{w,l}^{h}l_{l}\cos \theta_{l,l}+F_{w,l}^{v}l_{l}\sin \theta_{l,l}
```
#### 右腿

受力分析
```math
-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h}
```
```math
-F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}
```
加速度
```math
a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\sin \theta_{l,r})=\ddot{x}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
```math
a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\cos \theta_{l,r})=-l_{r}\sin \theta_{l,r}\ddot{\theta_{l,r}}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
转动分析
```math
I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-F_{w,r}^{h}l_{r}\cos \theta_{l,r}+F_{w,r}^{v}l_{r}\sin \theta_{l,r}
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
I_{b}\ddot{\theta}_{b}=-(\tau_{l,l}+\tau_{l,r})-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
```
```math
I_{\phi}\ddot{\phi }=(f_{r}-f_{l})R_{b}
```
### 控制律

我们需要
```math
x=\left[ \begin{matrix}
x \\
\dot{x}  \\
\phi \\
\dot{\phi} \\
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
\ddot{x}  \\
\dot{\phi} \\
\ddot{\phi} \\
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
\begin{align}
f  &  = m_{w}a_{w,l}^{h}+m_{w}a_{w,r}^{h}+m_{l}a_{l}^{h}+m_{l}a_{l,r}^{h}+m_{b}a_{b}^{h}  \\
 & = m_{w}\ddot{x}+m_{w}\ddot{x}+m_{l}a_{l,l}^{h}+m_{l}a_{l,r}^{h}+m_{b} \frac{a_{l,l}^{h}+a_{l,r}^{h}}{2} \\
 & = (2m_{w}+2m_{l}+m_{b})\ddot{x} \\
  & +\frac{1}{2}(2m_{l}+m_{b})(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
   & + \frac{1}{2}(2m_{l}+m_{b})(l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l}\dot{\theta}_{l,r}^{2})
\end{align}
```
代入 $`f`$
```math

\begin{align}
\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}  =& \left( 2m_{w}+2m_{l}+m_{b}+\frac{2I_{w}}{R_{w}^{2}} \right)\ddot{x} \\
  & +\frac{1}{2}(2m_{l}+m_{b})(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
   & + \frac{1}{2}(2m_{l}+m_{b})(l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l}\dot{\theta}_{l,r}^{2})
\end{align}
```
#### yaw转动方程

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]
```
#### 机体转动方程

```math
\begin{align}
I_{b} \ddot{\theta}_{b} = & -(\tau_{l,l}+\tau_{l,r})-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}gd_{b}\sin \theta_{b}-m_{b}\ddot{x}\cos \theta_{b} \\
 & +\frac{m_{b}}{2}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})d_{b}\sin \theta_{b} \\
 & -\frac{m_{b}}{2}(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})\cos \theta_{b}
\end{align}
```

#### 腿部转动方程

```math
\begin{align}
F_{w,l}^{v} & =F_{l,l}^{v}+m_{l}(a_{l,l}^{v}+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,l}^{v}+g) = \frac{1}{2}(m_{b}+2m_{l})a_{l,l}^{v}+\frac{1}{2}m_{b}a_{l,r}^{v} \\
 & =\frac{1}{2}(m_{b}+2m_{l})(-l_{l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2})+\frac{1}{2}m_{b}(-l_{r}\sin \theta_{l,r}\ddot{\theta_{l,r}}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{align}
```
```math
\begin{align}
I_{l,l}\ddot{\theta}_{l,l}= & \tau_{l,l}-\tau_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}) \\
 & -l_{w,l}\cos \theta_{l,l}(\frac{\tau_{w,l}-I_{w} \ddot{\theta}_{w,l}}{R_{w}}-m_{w}\ddot{x}) \\
 & +\frac{1}{2}(m_{b}+2m_{l})l_{r}\sin \theta_{l,l}(-l_{l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
& +\frac{1}{2}m_{b}l_{r}\sin \theta_{l,l}(-l_{r}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{r}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2})
\end{align}
```

```math
\begin{align}
I_{l,r}\ddot{\theta}_{l,r}= & \tau_{l,r}-\tau_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0}) \\
 & -l_{w,r}\cos \theta_{l,r}(\frac{\tau_{w,r}-I_{w} \ddot{\theta}_{w,r}}{R_{w}}-m_{w}\ddot{x}) \\
 & +\frac{1}{2}(m_{b}+2m_{l})l_{r}\sin \theta_{l,r}(-l_{l}\sin \theta_{l,l}\ddot{\theta_{l,l}}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
& +\frac{1}{2}m_{b}l_{r}\sin \theta_{l,r}(-l_{r}\sin \theta_{l,r}\ddot{\theta_{l,r}}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{align}
```
### 标准动力学

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
```

$`C`$ 汇集 $`\dot{\theta}^{2}`$ 离心项，此处不展开 $`M,B`$ 元素，只给维数与 $`G`$ 。

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
