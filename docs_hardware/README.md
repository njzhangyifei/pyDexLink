```
Title:        DexLink Servo Hardware Documentation (Simplified Chinese)
Date:         2016-08-29
Version:      0.0.1 (Subject to Change)
Prepared For: github.com/njzhangyifei/pyDexLink
```

# 采用 DexLink 的 RS485 舵机硬件部分细节文档

此文档将包括关于舵机硬件相关内容。
若要查询通讯协议，请见 [DexLink协议标准](http://vm.njzyf.info/static/dexlink/)

## RS485(TIA/EIA-485-A) 的正确使用

RS485 使用差分信号表示正负逻辑，可以有效控制共模噪声 Common Mode Noise 。

**以下列举一些常见问题的解决方案**

- 在**一主多从**的环境下，两条信号线需要利用终端电阻 Terminal Resistor 进行阻抗匹配，
此电阻在高速/多从机的环境下尤为重要。

该电阻的最佳摆放位置是在最远端的舵机的 RS485 A/B 两线之间，推荐电阻值是120Ohm。
此数值与线缆长短有关，若线缆较长，可能需要实验测定最佳电阻值。

- 若要增强信号的抗干扰性，比如舵机长时间工作在高电磁干扰环境下，推荐添加上下两个偏置电阻 Bias Resistor，
并使用双绞线作为 485 数据传输介质。

此情况应该较为少见，若线缆太长，也可能需要添加偏置电阻。

更多关于 RS485 接线的细节请阅读 
[Guidelines for Proper Wiring of an RS-485 (TIA/EIA-485-A) Network](https://www.maximintegrated.com/en/app-notes/index.mvp/id/763)


## 舵机的供电

推荐供电是 12V，电源需要能够供给足够的电源给线上的舵机。

在电压低于标准的情况下，舵机会自动禁用以保护 H 桥电路不受损。

一个舵机所级联的舵机在大负载的环境下是有限的。在满负载、最大电流的状态下，一个舵机所级联的舵机推荐不超过4-5个。

在许多舵机都需要大负载工作的情景下，推荐给舵机**分组走线、分组供电**。

在没有偏置电阻的情况下，舵机与舵机之间的地线不需要相连也可以进行通讯。


