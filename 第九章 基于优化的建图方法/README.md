# 第九章 基于优化的建图方法

代码下载：

https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%B9%9D%E7%AB%A0%20%E5%9F%BA%E4%BA%8E%E4%BC%98%E5%8C%96%E7%9A%84%E5%BB%BA%E5%9B%BE%E6%96%B9%E6%B3%95

[shenlan_ws 为按照课程ppt公式推导的代码](https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%B9%9D%E7%AB%A0%20%E5%9F%BA%E4%BA%8E%E4%BC%98%E5%8C%96%E7%9A%84%E5%BB%BA%E5%9B%BE%E6%96%B9%E6%B3%95/shenlan_ws)

[shenlan2_ws 为松鹏哥代码](https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%B9%9D%E7%AB%A0%20%E5%9F%BA%E4%BA%8E%E4%BC%98%E5%8C%96%E7%9A%84%E5%BB%BA%E5%9B%BE%E6%96%B9%E6%B3%95/shenlan2_ws)

native环境需要注意的是，g2o 版本需要进行更新，新的g2o版本已下载在shenlan_ws/third_party 中

## 1.IMU预积分 残差对部分变量的雅克比

### 预积分残差公式如下

分别为:位置残差，姿态残差，速度残差，accel_bias 残差， gyro_bias 残差
$$
\begin{bmatrix} r_p \\ r_q \\ r_v \\ r_{ba} \\ r_{bg} \end{bmatrix} = \begin{bmatrix}
q_{wb_i}^{*}(p_{wb_j} - p_{wb_i} - v_i^w \Delta t + \frac{1}{2}g^w \Delta t^2) - \alpha_{b_i b_j} \\
2[q_{b_i b_j}^{*} \bigotimes (q_{wb_i}^{*} \bigotimes q_{w b_j})]_{xyz} \\
q_{w b_i}^{*}(v_j^w - v_i^w + g^w \Delta t) - \beta_{b_i b_j} \\
b_j^a - b_i^a \\
b_j^g - b_i^g
\end{bmatrix}
$$

### 待优化变量

分别为：

 i时刻的相对位置，i时刻的相对姿态，i时刻的速度，i时刻的accel_bias，i时刻的gyto_bias

 j时刻的相对位置，j时刻的相对姿态，j时刻的速度，j时刻的accel_bias，j时刻的gyto_bias
$$
\left[\begin{array}{lllll}
\mathbf{p}_{w b_{i}} & \mathbf{q}_{w b_{i}} & \mathbf{v}_{i}^{w} & \mathbf{b}_{i}^{a} & \mathbf{b}_{i}^{g}
\end{array}\right]\left[\begin{array}{lllll}
\mathbf{p}_{w b_{j}} & \mathbf{q}_{w b_{j}} & \mathbf{V}_{j}^{w} & \mathbf{b}_{j}^{a} & \mathbf{b}_{j}^{g}
\end{array}\right]
$$
但在实际使用中，往往都是使用扰动量，因此实际是以下变量求雅克比
$$
\begin{aligned}
&{\left[\begin{array}{lllll}
\delta \mathbf{p}_{w b_{i}} & \delta \theta_{w b_{i}} & \delta \mathbf{v}_{i}^{w} & \delta \mathbf{b}_{i}^{a} & \delta \mathbf{b}_{i}^{g}
\end{array}\right]} \\
&{\left[\begin{array}{lllll}
\delta \mathbf{p}_{w b_{j}} & \delta \theta_{w b_{j}} & \delta \mathbf{v}_{j}^{w} & \delta \mathbf{b}_{j}^{a} & \delta \mathbf{b}_{j}^{g}
\end{array}\right]}
\end{aligned}
$$

### 1.1 位置残差的雅克比

1)对i时刻位置误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{p}_{w b_{i}}} &=\frac{\partial-\mathbf{q}_{w b_{i}}^{*}\left(\mathbf{p}_{w b_{i}}+\delta \mathbf{p}_{w b_{i}}\right)}{\partial \delta \mathbf{p}_{w b_{i}}} \\
&=-\mathbf{R}_{b_{i} w}
\end{aligned}
$$
2)对i时刻姿态误差的雅克比 

推导套路：a.添加右扰动 b.把扰动移动到右边，然后消去
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{p}}{\partial \delta \theta_{w b_{i}}} &=\frac{\partial\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{w b_{i}}
\end{array}\right]\right)^{*}\left(\mathbf{p}_{w b_{j}}-\mathbf{p}_{w b_{i}}-\mathbf{v}_{i}^{w} \Delta t+\frac{1}{2} \mathbf{g}^{w} \Delta t^{2}\right)}{\partial \delta \theta_{w b_{i}}} \\
&=\frac{\partial\left(\mathbf{R}_{w b_{i}} \exp \left(\delta \theta_{w b_{i}}^{\wedge}\right)\right)^{-1}\left(\mathbf{p}_{w b_{j}}-\mathbf{p}_{w b_{i}}-\mathbf{v}_{i}^{w} \Delta t+\frac{1}{2} \mathbf{g}^{w} \Delta t^{2}\right)}{\partial \delta \theta_{w b_{i}}} \\
&=\frac{\partial \exp \left(\left(-\delta \theta_{w b_{i}}\right)^{\wedge}\right) \mathbf{R}_{b_{i} w}\left(\mathbf{p}_{w b_{j}}-\mathbf{p}_{w b_{i}}-\mathbf{v}_{i}^{w} \Delta t+\frac{1}{2} \mathbf{g}^{w} \Delta t^{2}\right)}{\partial \delta \theta_{w b_{i}}} \\
& \approx \frac{\partial\left(\mathbf{I}-\delta \theta_{w b_{i}}^{\wedge}\right) \mathbf{R}_{b_{i} w}\left(\mathbf{p}_{w b_{j}}-\mathbf{p}_{w b_{i}}-\mathbf{v}_{i}^{w} \Delta t+\frac{1}{2} \mathbf{g}^{w} \Delta t^{2}\right)}{\partial \delta \theta_{w b_{i}}} \\
&=\left(\mathbf{R}_{b_{i} w}\left(\mathbf{p}_{w b_{j}}-\mathbf{p}_{w b_{i}}-\mathbf{v}_{i}^{w} \Delta t+\frac{1}{2} \mathbf{g}^{w} \Delta t^{2}\right)\right)^{\wedge}
\end{aligned}
$$


3)对i时刻速度误差的雅克比
$$
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{v}_{i}^{w}}=-\mathbf{R}_{b_{i} w} \Delta t
$$
4)对i时刻accel_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{b}_{i}^{a}} &=\frac{\partial-\left(\bar{\alpha}_{b_{i} b_{j}}+\mathbf{J}_{b_{i}^{a}}^{\alpha} \delta \mathbf{b}_{i}^{a}+\mathbf{J}_{b_{i}^{g}}^{\alpha} \delta \mathbf{b}_{i}^{g}\right)}{\partial \delta \mathbf{b}_{i}^{a}} \\
&=-\mathbf{J}_{b_{i}^{a}}^{\alpha}
\end{aligned}
$$
5)对i时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{b}_{i}^{g}}=-\mathbf{J}_{b_{i}^{g}}^{\alpha}
$$
6)对j时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{p}_{wb_{j}}}=\mathbf{R}_{b_{i} w}
$$
7)对j时刻姿态误差的雅克比 
$$
\frac{\partial \mathbf{r}_{p}}{\partial \delta \theta_{w b_{j}}} =0
$$
8)对j时刻速度误差的雅克比
$$
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{v}_{j}^{w}}=0
$$
9)对j时刻accel_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{b}_{j}^{a}} &=0
\end{aligned}
$$
10)对j时刻gyro_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{p}}{\partial \delta \mathbf{b}_{j}^{g}} &=0
\end{aligned}
$$

### 1.2 姿态残差的雅克比

1)对i时刻位置误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{p}_{w b_{i}}} &=0
\end{aligned}
$$
2)对i时刻姿态误差的雅克比 (课程已推)
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} &=\frac{\partial 2\left[\mathbf{q}_{b_{j} b_{i}} \otimes\left(\mathbf{q}_{b_{i} w} \otimes \mathbf{q}_{w b_{j}}\right)\right]_{x y z}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial 2\left[\mathbf{q}_{b_{i} b_{j}}^{*} \otimes\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\left.\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]
\end{array}\right]\right)^{*} \otimes \mathbf{q}_{w b_{j}}\right]_{x y z}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial-2\left[\left(\mathbf{q}_{b_{i} b_{j}}^{*} \otimes\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\left.\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]
\end{array}\right]\right)^{*} \otimes \mathbf{q}_{w b_{j}}\right)^{*}\right]_{x y z}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial-2\left[\mathbf{q}_{w b_{j}}^{*} \otimes\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}
\end{array}\right]\right) \otimes \mathbf{q}_{b_{i} b_{j}}\right]_{x y z}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}}
\end{aligned}
$$

$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} &=-2\left[\begin{array}{ll}
\mathbf{0} & \mathbf{I}
\end{array}\right] \frac{\partial \mathbf{q}_{w b_{j}}^{*} \otimes\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}
\end{array}\right]\right) \otimes \mathbf{q}_{b_{i} b_{j}}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=-2\left[\begin{array}{ll}
\mathbf{0} & \mathbf{I}
\end{array}\right] \frac{\partial\left[\mathbf{q}_{w b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}}\right]_{L}\left[\mathbf{q}_{b_{i} b_{j}}\right]_{R}\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}
\end{array}\right]}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=-2\left[\begin{array}{ll}
\mathbf{0} & \mathbf{I}
\end{array}\right]\left[\mathbf{q}_{w b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}}\right]_{L}\left[\mathbf{q}_{b_{i} b_{j}}\right]_{R}\left[\begin{array}{c}
\mathbf{0} \\
\frac{1}{2} \mathbf{I}
\end{array}\right]
\end{aligned}
$$

3)对i时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{v}_{i}^{w}} &=0
\end{aligned}
$$
4)对i时刻accel_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{b}_{i}^{a}} &=0
\end{aligned}
$$
5)对i时刻gyro_bias误差的雅克比 (课程已推)


$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{b}_{i}^{g}} &=\frac{\partial 2\left[\left(\mathbf{q}_{b_{i} b_{j}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \mathbf{J}_{b_{i}^{q}}^{q} \delta \mathbf{b}_{i}^{g}
\end{array}\right]\right)^{*} \otimes \mathbf{q}_{w b_{i}}^{*} \otimes \mathbf{q}_{w b_{j}}\right]_{x y z}}{\partial \delta \mathbf{b}_{i}^{g}} \\
&=\frac{\partial-2\left[\left(\left(\mathbf{q}_{b_{i} b_{j}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q} \delta \mathbf{b}_{i}^{g}
\end{array}\right]\right)^{*} \otimes \mathbf{q}_{w b_{i}}^{*} \otimes \mathbf{q}_{w b_{j}}\right)^{*}\right]_{x y z}}{\partial \delta \mathbf{b}_{i}^{g}} \\
&=\frac{\partial-2\left[\mathbf{q}_{w b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}} \otimes\left(\mathbf{q}_{b_{i} b_{j}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \mathbf{J}_{b_{i}^{g}} \delta \mathbf{b}_{i}^{g}
\end{array}\right]\right)\right]_{x y z}}{\partial \delta \mathbf{b}_{i}^{g}} \\
&=-2[\mathbf{0} \mathbf{I}]\left[\mathbf{q}_{w b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}} \otimes \mathbf{q}_{b_{i} b_{j}}\right]_{L}\left[\begin{array}{c}
0 \\
\frac{1}{2} \mathbf{J}_{b_{i}^{g}}^{q}
\end{array}\right]
\end{aligned}
$$
6)对j时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{p}_{wb_{j}}}=0
$$
7)对j时刻姿态误差的雅克比 
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \theta_{w b_{j}}} &=\frac{\partial 2\left[\mathbf{q}_{b_{i} b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}}^{*} \otimes \mathbf{q}_{w b_{j}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \theta_{w b_{j}}
\end{array}\right]\right]_{x y z}}{\partial \delta \theta_{w b_{j}}} \\
&=2\left[\begin{array}{ll}
0 & \mathbf{I}]\left[\mathbf{q}_{b_{i} b_{j}}^{*} \otimes \mathbf{q}_{w b_{i}}^{*} \otimes \mathbf{q}_{w b_{j}}\right]_{L}\left[\begin{array}{c}
1 \\
\frac{1}{2} \mathbf{I}
\end{array}\right]
\end{array}\right.
\end{aligned}
$$
8)对j时刻速度误差的雅克比
$$
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{v}_{j}^{w}}=0
$$
9)对j时刻accel_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{b}_{j}^{a}} &=0
\end{aligned}
$$
10)对j时刻gyro_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{q}}{\partial \delta \mathbf{b}_{j}^{g}} &=0
\end{aligned}
$$

### 1.3 速度残差的雅克比

1)对i时刻位置误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{p}_{w b_{i}}} &=0
\end{aligned}
$$
2)对i时刻姿态误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} &=\frac{\partial\left(\mathbf{q}_{w b_{i}} \otimes\left[\begin{array}{c}
1 \\
\frac{1}{2} \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}
\end{array}\right]\right)^{-1}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial\left(\mathbf{R}_{w b_{i}} \exp \left(\left[\delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]_{\times}\right)\right)^{-1}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial \exp \left(\left[-\delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]_{\times}\right) \mathbf{R}_{b_{i} w}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial\left(\mathbf{I}-\left[\delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]_{\times}\right) \mathbf{R}_{b_{i} w}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\frac{\partial-\left[\delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}\right]_{\times} \mathbf{R}_{b_{i} w}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)}{\partial \delta \boldsymbol{\theta}_{b_{i} b_{i}^{\prime}}} \\
&=\left[\mathbf{R}_{b_{i} w}\left(\mathbf{v}_{j}^{w}-\mathbf{v}_{i}^{w}+\mathbf{g}^{w} \Delta t\right)\right]_{\times}
\end{aligned}
$$
3)对i时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{v}_{i}^{w}} &= -\mathbf{R}_{b_{i} w}
\end{aligned}
$$
4)对i时刻accel_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{b}_{i}^{a}}=-\frac{\partial \boldsymbol{\beta}_{b_{i} b_{j}}}{\partial \delta \mathbf{b}_{i}^{a}}=-\mathbf{J}_{b_{i}^{a}}^{\beta}
$$
5)对i时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{b}_{i}^{g}}=-\frac{\partial \boldsymbol{\beta}_{b_{i} b_{j}}}{\partial \delta \mathbf{b}_{i}^{a}}=-\mathbf{J}_{b_{i}^{g}}^{\beta}
$$
6)对j时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{p}_{wb_{j}}}=0
$$
7)对j时刻姿态误差的雅克比 
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \theta_{w b_{j}}} &=0
\end{aligned}
$$
8)对j时刻速度误差的雅克比
$$
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{v}_{j}^{w}} = \mathbf{R}_{b_{i} w}
$$
9)对j时刻accel_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{b}_{j}^{a}} &=0
\end{aligned}
$$
10)对j时刻gyro_bias误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{v}}{\partial \delta \mathbf{b}_{j}^{g}} &=0
\end{aligned}
$$

### 1.4 加速度零偏残差的雅克比

1)对i时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{p}_{w b_{i}}}=\mathbf{0}
$$
2)对i时刻姿态误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \boldsymbol{\theta}_{wb_{i} {}}} &=0
\end{aligned}
$$
3)对i时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{v}_{i}^{w}} &= 0
\end{aligned}
$$
4)对i时刻accel_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{b}_{i}^{a}}=-I
$$
5)对i时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{b}_{i}^{g}}=0
$$
6)对j时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{p}_{w b_{j}}}=\mathbf{0}
$$
7)对j时刻姿态误差的雅克比 
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \boldsymbol{\theta}_{wb_{j} {}}} &=0
\end{aligned}
$$
8)对j时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{v}_{j}^{w}} &= 0
\end{aligned}
$$
9)对j时刻accel_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{b}_{j}^{a}}=I
$$
10)对j时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{a}}}{\partial \delta \mathbf{b}_{j}^{g}}=0
$$


### 1.5 角速度零偏残差雅克比

1)对i时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{p}_{w b_{i}}}=\mathbf{0}
$$
2)对i时刻姿态误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \boldsymbol{\theta}_{wb_{i} {}}} &=0
\end{aligned}
$$
3)对i时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{v}_{i}^{w}} &= 0
\end{aligned}
$$
4)对i时刻accel_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{b}_{i}^{a}}=0
$$
5)对i时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{b}_{i}^{g}}=-I
$$
6)对j时刻位置误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{p}_{w b_{j}}}=\mathbf{0}
$$
7)对j时刻姿态误差的雅克比 
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \boldsymbol{\theta}_{wb_{j} {}}} &=0
\end{aligned}
$$
8)对j时刻速度误差的雅克比
$$
\begin{aligned}
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{v}_{j}^{w}} &= 0
\end{aligned}
$$
9)对j时刻accel_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{b}_{j}^{a}}=0
$$
10)对j时刻gyro_bias误差的雅克比
$$
\frac{\partial \mathbf{r}_{b^{g}}}{\partial \delta \mathbf{b}_{j}^{g}}=I
$$

## 2.补全预积分部分代码

预积分代码填充部分，参考作业的示例里为松鹏大哥的代码，此部分代码，在求解残差雅克比时，使用的是vins的写法，所以与课程的ppt公式会有所不同。本节的作业补全代码提供 1.作业讲评的代码参考  2. 按照作业ppt公式推导的代码参考

### 2.1 作业讲评的代码参考

FILE : lidar_localization/src/models/pre_integrator/imu_pre_integrator.cpp  UpdateState()

**注意**：在误差雅克比推导过程中

a.F12   PPT中的推导为( I - w.vee()*delta_t )  ,  而 dR_inv  =  d_theta_ij.inverse().matrix() ， 两者是近似的，因为(I - w.vee()*delta_t )   近似与 exp(-theta) ,exp(-theta)为-theta进行so3的指数映射，所以exp(-theta) = (exp(theta))^-1 = R.inverse()

b.F 和 B阵的写法按照vins的写法 ，例如 F_k+1 = I + F_k * T ; B_k+1 = B_k * T

```cpp
    // TODO: a. update mean:
    //
    // 1. get w_mid:
    w_mid = 0.5 * (prev_w + curr_w) ;
    // 2. update relative orientation, so3:     更新新姿态角
    prev_theta_ij = state.theta_ij_;
    d_theta_ij = Sophus::SO3d::exp(w_mid * T) ;      //  ij 时刻的相对姿态
    state.theta_ij_ = state.theta_ij_ * d_theta_ij ;      //  当前时刻姿态更新
    curr_theta_ij =  state.theta_ij_ ;
    // 3. get a_mid:        
    a_mid = 0.5 * (prev_theta_ij * prev_a  +  curr_theta_ij * curr_a);   //  aceel  world系下
    // 4. update relative translation:      更新平移
    state.alpha_ij_ +=  state.beta_ij_ * T + 0.5 * a_mid * T * T ;      //  p_k+1 =  v_k * T + 0.5*a_k+1*t*t
    // 5. update relative velocity:
    state.beta_ij_ +=  a_mid * T;               //  vel  world系下
```

```cpp
    // TODO: b. update covariance:
    //
    // 1. intermediate results:
    dR_inv = d_theta_ij.inverse().matrix();
    prev_R = prev_theta_ij.matrix();
    curr_R  = curr_theta_ij.matrix();
    prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a) ;
    curr_R_a_hat  = curr_R  * Sophus::SO3d::hat(curr_a) ; 
```

```cpp
    // TODO: 2. set up F:
    //
    // F12 & F22 & F32:  F_k 和  G_k 是离散时间下的状态传递方程中的矩阵，一般是在连续时间下推导微分方程，再用它计算离散时间下的传递方程
    F_.block<3,  3>(INDEX_ALPHA,  INDEX_THETA)  =  -0.25 * T  * (prev_R_a_hat  +  curr_R_a_hat * dR_inv) ;   //  F12
    F_.block<3,  3>(INDEX_BETA, INDEX_THETA)  =  - 0.5 * T *(prev_R_a_hat  + curr_R_a_hat * dR_inv ) ;     //  F32
    // F14 & F34:
    F_.block<3,3>(INDEX_ALPHA, INDEX_B_A) =  -0.25 * T * (prev_R + curr_R);    //  F14
    F_.block<3,3>(INDEX_BETA,INDEX_B_A)  =  -0.5 *  (prev_R + curr_R);       //  F34 
    // F15 & F25 & F35:
    F_.block<3,3>(INDEX_ALPHA, INDEX_B_G) = 0.25 * T * T * curr_R_a_hat  ;   //  F15
    F_.block<3,3>(INDEX_BETA, INDEX_B_G) = 0.5 * T  * curr_R_a_hat;  // F35 
    //  F22
    F_.block<3,  3>(INDEX_THETA, INDEX_THETA) =  - Sophus::SO3d::hat(w_mid) ;  //  F22 
```

```cpp
    // TODO: 3. set up B:
    //
    // G11 & G31:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = 0.25 * prev_R * T ;            //  G11
    B_.block<3,3>(INDEX_BETA, INDEX_M_ACC_PREV) = 0.5 * prev_R ;           // G31
    // G12 & G22 & G32:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = -0.125 * T * T * curr_R_a_hat ; // G12
    B_.block<3,3>(INDEX_BETA, INDEX_M_GYR_PREV) = -0.25 * T *curr_R_a_hat ;     // G32
    // G13 & G33:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = 0.25 * curr_R * T ;    //  G13
    B_.block<3,3>(INDEX_BETA, INDEX_M_ACC_CURR) = 0.5 * curr_R ;    //  G33
    // G14 & G24 & G34:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = -0.125 * T  * T * curr_R_a_hat;  // G14
    B_.block<3,3>(INDEX_BETA,INDEX_M_GYR_CURR ) =  -0.25 * T *curr_R_a_hat;  // G34
```

```cpp
    // TODO: 4. update P_:
    MatrixF  F = MatrixF::Identity() + T *F_ ;
    MatrixB B = T * B_;
    P_ = F * P_ * F.transpose()  + B * Q_ * B.transpose() ;                     //   Q  imu噪声的方差
```

```cpp
    // TODO: 5. update Jacobian:
    //
    J_ = F * J_ ;
```

FILE : lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag
_imu_pre_integration.hpp	computeError()

**注意**：残差推导的R项中，课程参考代码推导方式使用SO3 旋转矩阵推导。而ppt中的推导方式使用的是四元数，当theta为微小量是，使用四元数进行推导的残差四元数的虚部 近似于  2/theta， 所以如果是用四元数的虚部表示残差r_q时，是需要在四元数虚部的基础上乘2。

```cpp
		// TODO: update pre-integration measurement caused by bias change:
		// 
		if (v0 -> isUpdated() ) {
			Eigen::Vector3d  d_b_a_i , d_b_g_i ;
			v0->getDeltaBiases(d_b_a_i, d_b_g_i);
			updateMeasurement(d_b_a_i,d_b_g_i);
		}
```

```cpp
		// TODO: compute error:
		//
		const Eigen::Vector3d &alpha_ij =  _measurement.block<3,  1>(INDEX_P,  0);			//   获取观测值
		const Eigen::Vector3d &theta_ij  = _measurement.block<3,   1>(INDEX_R, 0);			 
		const Eigen::Vector3d &beta_ij  = _measurement.block<3, 1>(INDEX_V, 0);		
		_error.block<3, 1>(INDEX_P, 0)  = ori_i.inverse().matrix() * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
		_error.block<3,1>(INDEX_R, 0)	= (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() * ori_j).log();
		_error.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
		_error.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
		_error.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;
```

lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prva
g.hpp  oplusImpl()

```cpp
    virtual void oplusImpl(const double *update) override {
        //
        // TODO: do update
        //
        _estimate.pos  +=  Eigen::Vector3d(
            update[PRVAG::INDEX_POS + 0],  update[PRVAG::INDEX_POS + 1],  update[PRVAG::INDEX_POS + 2]  
        );
        _estimate.ori    =  _estimate.ori * Sophus::SO3d::exp(
            Eigen::Vector3d(
                update[PRVAG::INDEX_ORI + 0],  update[PRVAG::INDEX_ORI + 1],  update[PRVAG::INDEX_ORI + 2]  
            )
        );
        _estimate.vel += Eigen::Vector3d(
                update[PRVAG::INDEX_VEL + 0],  update[PRVAG::INDEX_VEL + 1],  update[PRVAG::INDEX_VEL + 2]  
        );
        Eigen::Vector3d d_b_a_i(
                update[PRVAG::INDEX_B_A + 0],  update[PRVAG::INDEX_B_A + 1],  update[PRVAG::INDEX_B_A + 2]  
        );
        Eigen::Vector3d d_b_g_i(
                update[PRVAG::INDEX_B_G + 0],  update[PRVAG::INDEX_B_G + 1],  update[PRVAG::INDEX_B_G + 2]  
        );

        _estimate.b_a += d_b_a_i ;
        _estimate.b_g += d_b_g_i;               

        updateDeltaBiases(d_b_a_i, d_b_g_i);
    }
```

### 2.2 按照课程ppt公式推导的代码参考

FILE : lidar_localization/src/models/pre_integrator/imu_pre_integrator.cpp	UpdateState()

```cpp
    //
    // TODO: a. update mean:        名义值更新：中值积分
    //
    // 1. get w_mid:
    w_mid = 0.5 * (prev_w + curr_w) ;
    // 2. update relative orientation, so3:     更新新姿态角
    prev_theta_ij = state.theta_ij_;
    d_theta_ij = Sophus::SO3d::exp(w_mid * T) ;      //  ij 时刻的相对姿态
    state.theta_ij_ = state.theta_ij_ * d_theta_ij ;      //  当前时刻姿态更新
    curr_theta_ij =  state.theta_ij_ ;
    // 3. get a_mid:        
    a_mid = 0.5 * (prev_theta_ij * prev_a  +  curr_theta_ij * curr_a);   //  aceel  world系下
    // 4. update relative translation:      更新平移
    state.alpha_ij_ +=  state.beta_ij_ * T + 0.5 * a_mid * T * T ;      //  p_k+1 =  v_k * T + 0.5*a_k+1*t*t
    // 5. update relative velocity:
    state.beta_ij_ +=  a_mid * T;               //  vel  world系下
```

```cpp
    // TODO: b. update covariance:  误差值更新: 中间值
    //
    // 1. intermediate results:
    dR_inv = d_theta_ij.inverse().matrix();
    prev_R = prev_theta_ij.matrix();
    curr_R  = curr_theta_ij.matrix();
    prev_R_a_hat = prev_R * Sophus::SO3d::hat(prev_a) ;
    curr_R_a_hat  = curr_R  * Sophus::SO3d::hat(curr_a) ; 
```

```cpp
    // TODO: 2. set up F:  误差更新：F矩阵
    //
    // F12 & F22 & F32:  F_k 和  G_k 是离散时间下的状态传递方程中的矩阵，一般是在连续时间下推导微分方程，再用它计算离散时间下的传递方程
    F_ = MatrixF::Identity();
    F_.block<3,  3>(INDEX_ALPHA,  INDEX_THETA)  =  -0.25 * T * T * (prev_R_a_hat  +  curr_R_a_hat * (Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid)*T ) ) ;   //  F12
    F_.block<3,  3>(INDEX_THETA, INDEX_THETA) = Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid)*T ;  //  F22 
    F_.block<3,  3>(INDEX_BETA, INDEX_THETA)  =  - 0.5 * T *(prev_R_a_hat  + curr_R_a_hat * (Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid)*T ) ) ;     //  F32
    //  F13
    F_.block<3, 3>(INDEX_ALPHA, INDEX_BETA) = Eigen::Matrix3d::Identity() * T ;   //  F13
    // F14 & F34:
    F_.block<3,3>(INDEX_ALPHA, INDEX_B_A) =  -0.25 * T * T * (prev_R + curr_R);    //  F14
    F_.block<3,3>(INDEX_BETA,INDEX_B_A)  =  -0.5 * T * (prev_R + curr_R);       //  F34 
    // F15 & F25 & F35:
    F_.block<3,3>(INDEX_ALPHA, INDEX_B_G) = 0.25 * T * T * T * curr_R_a_hat  ;   //  F15
    F_.block<3,3>(INDEX_THETA, INDEX_B_G) = - Eigen::Matrix3d::Identity() * T ;  //  F25
    F_.block<3,3>(INDEX_BETA, INDEX_B_G) = 0.5 * T * T * curr_R_a_hat;  // F35 
```

```cpp
    // TODO: 3. set up B:  误差更新：B矩阵
    //
    B_ = MatrixB::Zero();
    // G11 & G31:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_ACC_PREV) = 0.25 * prev_R * T *T;            //  G11
    B_.block<3,3>(INDEX_BETA, INDEX_M_ACC_PREV) = 0.5 * prev_R * T ;           // G31
    // G12 & G22 & G32:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_GYR_PREV) = -0.125 * T * T * T * curr_R_a_hat ; // G12
    B_.block<3,3>(INDEX_THETA, INDEX_M_GYR_PREV) = 0.5 * T * Eigen::Matrix3d::Identity()  ; // G22
    B_.block<3,3>(INDEX_BETA, INDEX_M_GYR_PREV) = -0.25 * T * T *curr_R_a_hat ;     // G32
    // G13 & G33:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_ACC_CURR) = 0.25 * curr_R * T * T;    //  G13
    B_.block<3,3>(INDEX_BETA, INDEX_M_ACC_CURR) = 0.5 * curr_R * T;    //  G33
    // G14 & G24 & G34:
    B_.block<3,3>(INDEX_ALPHA, INDEX_M_GYR_CURR) = -0.125 * T * T * T * curr_R_a_hat;  // G14
    B_.block<3,3>(INDEX_THETA, INDEX_M_GYR_CURR) = 0.5 * Eigen::Matrix3d::Identity() * T ; // G24
    B_.block<3,3>(INDEX_BETA,INDEX_M_GYR_CURR ) =  -0.25 * T * T *curr_R_a_hat;  // G34
    //G45   
    B_.block<3,3>(INDEX_B_A, INDEX_R_ACC_PREV) = Eigen::Matrix3d::Identity() * T;     //G45
    //G56
    B_.block<3,3>(INDEX_B_G, INDEX_R_GYR_PREV) = Eigen::Matrix3d::Identity() * T;    //G56
```

```cpp
   // TODO: 4. update P_:	误差更新 P 矩阵
    P_ = F_ * P_ * F_.transpose()  + B_ * Q_ * B_.transpose() ;                     //   Q  imu噪声的方差
```

```cpp
    // TODO: 5. update Jacobian:  误差更新 J 矩阵
    //
    J_ = F_ * J_ ;
```

FILE : lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/edge/edge_prvag
_imu_pre_integration.hpp	computeError()

```cpp
		// TODO: update pre-integration measurement caused by bias change:  更新预积分部分(由于bias的改变，产生的变化)
		// 
		if (v0 -> isUpdated() ) {
			Eigen::Vector3d  d_b_a_i , d_b_g_i ;
			v0->getDeltaBiases(d_b_a_i, d_b_g_i);
			updateMeasurement(d_b_a_i,d_b_g_i);
		}
```

```cpp
		// TODO: compute error:
		//
		const Eigen::Vector3d &alpha_ij =  _measurement.block<3,  1>(INDEX_P,  0);			//   获取观测值
		const Eigen::Vector3d &theta_ij  = _measurement.block<3,   1>(INDEX_R, 0);		
		const Eigen::Vector3d &beta_ij  = _measurement.block<3, 1>(INDEX_V, 0);		
		Eigen::Quaterniond q_ij = Eigen::Quaterniond(Sophus::SO3d::exp(theta_ij).matrix());
		Eigen::Quaterniond q_wbi =  Eigen::Quaterniond(ori_i.matrix());
		Eigen::Quaterniond q_wbj =  Eigen::Quaterniond(ori_j.matrix());
		_error.block<3, 1>(INDEX_P, 0)  = ori_i.inverse().matrix() * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
		_error.block<3, 1>(INDEX_R, 0) = 2*(q_ij.conjugate() * (q_wbi.conjugate() * q_wbj)).vec();				//  当theta为小量时，theta 近似于 四元数的虚部的两倍
		_error.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
		_error.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
		_error.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;
```

lidar_localization/include/lidar_localization/models/graph_optimizer/g2o/vertex/vertex_prva
g.hpp  oplusImpl()

```cpp
    virtual void oplusImpl(const double *update) override {         //  顶点更新
        //
        // TODO: do update
        //
        _estimate.pos  +=  Eigen::Vector3d(
            update[PRVAG::INDEX_POS + 0],  update[PRVAG::INDEX_POS + 1],  update[PRVAG::INDEX_POS + 2]  
        );
        _estimate.ori    =  _estimate.ori * Sophus::SO3d::exp(
            Eigen::Vector3d(
                update[PRVAG::INDEX_ORI + 0],  update[PRVAG::INDEX_ORI + 1],  update[PRVAG::INDEX_ORI + 2]  
            )
        );
        _estimate.vel += Eigen::Vector3d(
                update[PRVAG::INDEX_VEL + 0],  update[PRVAG::INDEX_VEL + 1],  update[PRVAG::INDEX_VEL + 2]  
        );
        Eigen::Vector3d d_b_a_i(
                update[PRVAG::INDEX_B_A + 0],  update[PRVAG::INDEX_B_A + 1],  update[PRVAG::INDEX_B_A + 2]  
        );
        Eigen::Vector3d d_b_g_i(
                update[PRVAG::INDEX_B_G + 0],  update[PRVAG::INDEX_B_G + 1],  update[PRVAG::INDEX_B_G + 2]  
        );

        _estimate.b_a += d_b_a_i ;
        _estimate.b_g += d_b_g_i;               

        updateDeltaBiases(d_b_a_i, d_b_g_i);
    }
```



## 3.evo 评估  used imu_pre  and   no uded imu_pre

### 3.1 选择是否融合imu_pre

gnss  、 loop_close、imu_pre_integration、odo-pre_integration 开关

FILE : lidar_localiation/config/mapping/lio_back_end.yaml

```shell
#     1. g2o
graph_optimizer_type: g2o

# config measurement used:
# a. GNSS
use_gnss: true
# b. loop closure
use_loop_close: true
# c. IMU pre-integration		使用预积分：true    不使用预积分：false
use_imu_pre_integration: false			
# c. odo pre-integration
use_odo_pre_integration: true
```

```
evo_rpe kitti ground_truth.txt optimized.txt -r trans_part --delta 100 --plot --plot_mode xyz
```

```
evo_ape kitti ground_truth.txt optimized.txt -r full --plot --plot_mode xyz
```

### 3.2 使用作业讲评代码

|         | no used  imu_pre                                             | used imu_pre                                                 |
| ------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|         | ![no_used_imu_pre1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre1.png) | ![imu_pre_ref1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11imu_pre_ref1.png) |
|         | <img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre2.png" alt="no_used_imu_pre2"  /> | ![imu_pre_ref2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11imu_pre_ref2.png) |
| evo_ape | ![no_used_imu_pre4](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre4.png) | ![imu_pre_ref3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11imu_pre_ref3.png) |
|         | ![no_used_imu_pre3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre3.png) | ![imu_pre_ref4](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11imu_pre_ref4.png) |
|         | max	1.375001<br/>      mean	0.358914<br/>    median	0.332925<br/>       min	0.023321<br/>      rmse	0.413774<br/>       sse	327.693883<br/>       std	0.205887 | max	2.338323<br/>      mean	0.807955<br/>    median	0.710029<br/>       min	0.025905<br/>      rmse	0.935937<br/>       sse	1676.620677<br/>       std	0.472425 |

### 3.3 使用课程推导的PPT公式

|         | no used  imu_pre                                             | used imu_pre                                                 |
| ------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
|         | ![no_used_imu_pre1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre1.png) | ![use_manual_jaco_1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11use_manual_jaco_1.png) |
|         | <img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre2.png" alt="no_used_imu_pre2"  /> | ![use_manual_jaco_2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11use_manual_jaco_2.png) |
| evo_ape | ![no_used_imu_pre4](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre4.png) | ![use_manual_jaco_3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11use_manual_jaco_3.png) |
|         | ![no_used_imu_pre3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11no_used_imu_pre3.png) | ![use_manual_jaco_4](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11use_manual_jaco_4.png) |
|         | max	1.375001<br/>      mean	0.358914<br/>    median	0.332925<br/>       min	0.023321<br/>      rmse	0.413774<br/>       sse	327.693883<br/>       std	0.205887 | max	2.314255<br/>      mean	0.803588<br/>    median	0.704937<br/>       min	0.025532<br/>      rmse	0.930225<br/>       sse	1656.221045<br/>       std	0.468578 |

### 3.4 结论

加入imu_pre 后，在精度上并没有太大的优化，推测可能是kitti数据自身存在误差。但使用因子图融合的方法，比laser_odom 精度要高

## 4.推导融合编码器时预积分公式的推导(方差递推、残差对状态量雅克比、bias更新)

此部分推导，主要参考张松鹏的推导过程 

注意：

1.为什么imu需要做预积分，而雷达不需要做，编码器需要做预积分吗？

​		imu、编码器获取都是载体系(body)下的测量值，每次积分都需要转换到w系下，需要重新积分，使用预积分的方法，可以大大降低不必要的重复计算。雷达获取的是两帧点云间的位姿，不需要做转换。

2.编码器提供的是二维测量，能够反馈较为准确的位置信息。imu能反馈角速度和线加速度信息，通过对角速度进行积分可获相对旋转角度，通过对线加速度积分可获得相对位移，但使用imu积分得到的相对位移发散问题比较严重。故在imu+编码器融合的方法，可选取 imu提供p，imu提供q、v的方法进行融合。

​                                                                                                                                                                                                                    edited	by  kaho 2022.3.10																							                                                                                       																									

​		
