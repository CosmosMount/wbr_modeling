# 离地与跳跃控制

## 支持力解算

由 [vmc](04_vmc.md) 我们知道

```math
\left[ \begin{matrix}
F_{L} \\
\tau_{l}
\end{matrix} \right]=
(J^{T})^{-1}\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]=
(J^{T})^{-1}\left[ \begin{matrix}
k_{\tau}I_{q_1} \\
k_{\tau}I_{q_4}
\end{matrix} \right]
```

那么我们可以从电机反馈解得此时等效杆上的输出力、力矩值，进一步计算足端支持力，再通过足端支持力的大小判定机器人所处的状态。

对于牛顿-欧拉法分析

```math
-F_{w,i}^{v}+F_{N,i}-m_{w}g=m_{w}a_{w,i}^{v}\implies F_{N,i}=F_{w,i}^{v}+m_{w}(g+a_{w,i}^{v})
```

$`a_{b}^{v}`$ 可以由IMU测量与处理后得到，因此我们将原来的推导顺序反过来，同时考虑腿长变化，得到

```math
F_{w,i}^{v} =F_{l,i}^{v}+m_{l}(a_{l,i}^v+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,i}^{v}+g)
```

```math
a_{w,i}^{v}=a_{b}^{v}-\frac{ \partial ^{2} }{ \partial t } (l_{i}\cos \theta_{l})=a_{b}^{v}+l_{i}\sin \theta_{l,i}\ddot{\theta}_{l,i}+l_{i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}-\ddot{l}_{i}\cos\theta_{l,i}+2\dot{l}_{i}\sin \theta_{l,i}\dot{\theta}_{l,i}
```

```math
a_{l,i}^{v}=a_{b}^{v}-\frac{ \partial ^{2} }{ \partial t } (l_{b,i}\cos \theta_{l,i})=a_{b}^{v}+l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}+l_{b,i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}-\ddot{l}_{b,i}\cos\theta_{l,i}+2\dot{l}_{b,i}\sin \theta_{l,i}\dot{\theta}_{l,i}
```

这里，$`F_{w}^{v}`$ 的计算实际没有考虑腿伸长力与气弹簧力的影响，在 [vmc](04_vmc.md) 中，我们对气弹簧的力 $F_s$ 进行了估计，对气弹簧力进行补偿之后，理论上等价于无气弹簧模型。因此记实际竖直压力

```math
F_{v,i} = F_{w,i}^{v} + (F_{L,i}-F_{s,i})\cos\theta_{l,i}
```

为了方便计算，忽略 $`\dot{\theta}_{l,i}^{2}`$ 项和交叉项

```math
a_{w,i}^{v}=a_{b}^{v}+l_{i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-\ddot{l}_{i}\cos\theta_{l,i}
```

```math
a_{l,i}^{v}=a_{b}^{v}+l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-\ddot{l}_{b,i}\cos\theta_{l,i}
```

```math
F_{w,i}^{v} =m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,i}^{v}+g)=(m_{b}+m_{l})(a_{b}^{v}+g)+m_{l}l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}
```

那么，由于 $`l_{b,i}=\mu l_{i}`$，我们可以化简得到
```math
F_{N,i}=(m_{b}+m_{l}+m_{w})(a_{b}^{v}+g)+(\mu m_{l}+m_{w})(l_{l}\sin \theta_{l,i}\ddot{\theta}_{l,i}+\ddot{l}_{i}\cos\theta_{l,i})+(F_{L,i}-F_{s,i})\cos\theta_{l,i}
```

## 离地策略与阈值

计算出支持力表达式后，需要确定离地策略和离地阈值。

机器人离地时，轮子无法作为有效控制量参与控制，此时轮子需要完全失能避免空转带来的能量消耗与巨大电流，以及轮高速旋转对腿的反作用矩。

缺失轮子的控制，摆杆的摆动完全由髋转矩决定，那么对摆杆的大幅度控制必然引起机体的旋转，而机体在滞空过程中可能出现小幅度的俯仰角；同时落地缓冲依赖摆杆竖直向下；因此我们可以考虑在滞空过程中只对摆角进行控制，最终控制量简化为

```math
x=\left[ \begin{matrix}
0 \\
0  \\
0 \\
0 \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
0 \\
0
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
0 \\
0  \\
0 \\
0 \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
0 \\
0
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
0  \\
0 \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
```

同时，为了最大化实现腿部缓冲性能，我们选择在滞空时将腿长伸到最长，就像设计一个缓冲的弹簧一样，伸长腿长有两种思路：

1. 通过PID主动控制腿长伸长，落地时能够调节弹簧阻尼
2. 利用气弹簧直接伸长并在落地时依赖气弹簧自身阻尼

仍需要测试 **@TODO** 得到上述两种思路离地时支持力解算的差异

## 跳跃控制

实际上，对于跳跃而言，只需要保证足够的沿杆方向的力，加上合适的收腿时机，就能够实现对应的跳跃高度。跳跃的上限主要由

1. 起跳时机身姿态控制
2. 髋电机最大扭矩
3. 电路最大承载电流

仍需要测试定量数据 **@TODO**