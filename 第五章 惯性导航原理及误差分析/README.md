# 多传感器融合定位 第五章 惯性导航原理及误差分析-不需要转台的IMU内参标定

参考博客：[[深蓝学院-多传感器融合定位-第5章作业]](https://blog.csdn.net/weixin_42113967/article/details/114006797?spm=1001.2014.3001.5502)

代码下载：[https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%83%AF%E6%80%A7%E5%AF%BC%E8%88%AA%E5%8E%9F%E7%90%86%E5%8F%8A%E8%AF%AF%E5%B7%AE%E5%88%86%E6%9E%90/imu_tk](https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%83%AF%E6%80%A7%E5%AF%BC%E8%88%AA%E5%8E%9F%E7%90%86%E5%8F%8A%E8%AF%AF%E5%B7%AE%E5%88%86%E6%9E%90/imu_tk)

## 1. 内参误差模型

### ideas

1.有别于基于转台的内参标定(分立级标定)，通过自定义坐标轴的方法，也可以离线标定IMU的内参。

2.安装误差使用是下三角模型，坐标轴定义为：IMU的坐标轴 Xb(自定义)Accel坐标轴Xa(器件坐标轴)重合,Zb 垂直于平面XbOYb，使用这样的方法，加速度计可以少估计三个内参。课件中，并没有对陀螺仪的bias进行内参标定，原因是，IMU的零偏具有上电不重复性，只能上电时进行校准，并且不像accel加速度计一样具有重力加速度g这样的绝对基准。

3.对gyro进行内参标定，办法是，先通过对accel的内参标定，对修正过后的加速度进行积分，计算旋转角度，进而估计gyro的内参。



![PPT1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT1.png)

![PPT2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT2.png)

![PPT3](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/深蓝多传感器融合定位/第四章点云地图构建及基于点云地图定位%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT3.png)

![PPT4](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT4.png)

![PPT5](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT5.png)

![PPT6](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8DPPT6.png)



## 2.修改imu_tk  安装误差模型为下三角模型

imu_tk 中使用的模型为上三角模型

FILE :    IMU_TK/src/calibration.cpp

### TODO 1

```cpp
bool operator() (
    const _T2* const params, 
    _T2* residuals 
  ) const {
    Eigen::Matrix<_T2, 3, 1> raw_samp( 
      _T2(sample_(0)), 
      _T2(sample_(1)), 
      _T2(sample_(2)) 
    );

    /* Apply undistortion transform to accel measurements
         mis_mat_ <<  _T(1)   , -mis_yz  ,  mis_zy  ,
                       mis_xz ,  _T(1)   , -mis_zx  ,  
                      -mis_xy ,  mis_yx  ,  _T(1)   ;
              
       scale_mat_ <<   s_x  ,   _T(0)  ,  _T(0) ,
                      _T(0) ,    s_y   ,  _T(0) ,  
                      _T(0) ,   _T(0)  ,   s_z  ;
                    
        bias_vec_ <<  b_x , b_y , b_z ;

        define:
           ms_mat_ = mis_mat_*scale_mat_

        then the undistortion transform:
                X' = T*K*(X - B)

        can be implemented as:

       unbias_data = ms_mat_*(raw_data - bias_vec_)

     * assume body frame same as accelerometer frame, 
     * so bottom left params in the misalignment matris are set to zero */
    CalibratedTriad_<_T2> calib_triad(                       //    12 个内参
      //
      // TODO: implement lower triad model here         初始化内参
      //
      // mis_yz, mis_zy, mis_zx:                          安装误差
      _T2(0), _T2(0), _T2(0),                                  //  因为使用的是没有转台的模式，所以可以省去安装误差3个参数
      // mis_xz, mis_xy, mis_yx:                  
      params[0], params[1], params[2],                           
      //    s_x,    s_y,    s_z:
      params[3], params[4], params[5],          //  标度因素  
      //    b_x,    b_y,    b_z: 
      params[6], params[7], params[8]           //  零偏
    );
```

### TODO 2

```CPP
    // TODO: implement lower triad model here         实现下三角模型
    //
      //origin   安装误差  上三角模型
    // acc_calib_params[0] = init_acc_calib_.misYZ();                      
    // acc_calib_params[1] = init_acc_calib_.misZY();
    // acc_calib_params[2] = init_acc_calib_.misZX();    

    /*new  按照课程的公式，推导，改为下三角模型*/
    acc_calib_params[0] = init_acc_calib_.misXZ();                      
    acc_calib_params[1] = init_acc_calib_.misXY();
    acc_calib_params[2] = init_acc_calib_.misYX();
    
    acc_calib_params[3] = init_acc_calib_.scaleX();                    //    标度因素 
    acc_calib_params[4] = init_acc_calib_.scaleY();
    acc_calib_params[5] = init_acc_calib_.scaleZ();
    
    acc_calib_params[6] = init_acc_calib_.biasX();                     //  零偏
    acc_calib_params[7] = init_acc_calib_.biasY();
    acc_calib_params[8] = init_acc_calib_.biasZ();
```

### TODO 3

```cpp
  acc_calib_ = CalibratedTriad_<_T>( 
    //
    // TODO: implement lower triad model here
    // 
    0,0,0,
    min_cost_calib_params[0],
    min_cost_calib_params[1],
    min_cost_calib_params[2],

    min_cost_calib_params[3],
    min_cost_calib_params[4],
    min_cost_calib_params[5],
    min_cost_calib_params[6],
    min_cost_calib_params[7],
    min_cost_calib_params[8] 
  );
```

## 3.ceres 非线性优化，求解加速度计内参

### 3.1 调用

使用宏的方法，若 #define  autograde 则使用 自动求导 ，反之  使用解析求导

```cpp
      #ifdef  autograde 
          ceres::CostFunction* cost_function = MultiPosAccResidual<_T>::Create ( 
            g_mag_, static_samples[i].data() 
          );
      #else 
          ceres::CostFunction *cost_function = new MultiPosAccResidual_Analytical<_T>(
            g_mag_, static_samples[i].data());
      #endif
```



### 3.2 自动求导   autograde

#### 3.2.1 ceres::CostFuntion

```cpp
  static ceres::CostFunction* Create ( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample )
  {
    return ( new ceres::AutoDiffCostFunction< MultiPosAccResidual, 1, 9 > (				//  残差维度 1  ，优化参数维度 9
               new MultiPosAccResidual<_T1>( g_mag, sample ) ) );
  }
```

#### 3.2.2 计算残差    MultiPosAccResidual

```cpp
template <typename _T1> struct MultiPosAccResidual
{
  MultiPosAccResidual( 
    const _T1 &g_mag, 
    const Eigen::Matrix< _T1, 3 , 1> &sample 
  ) : g_mag_(g_mag), sample_(sample){}
  
  template <typename _T2>
  bool operator() (
    const _T2* const params, 
    _T2* residuals 
  ) const {
    Eigen::Matrix<_T2, 3, 1> raw_samp( 
      _T2(sample_(0)), 
      _T2(sample_(1)), 
      _T2(sample_(2)) 
    );

    /* Apply undistortion transform to accel measurements
         mis_mat_ <<  _T(1)   , -mis_yz  ,  mis_zy  ,
                       mis_xz ,  _T(1)   , -mis_zx  ,  
                      -mis_xy ,  mis_yx  ,  _T(1)   ;
              
       scale_mat_ <<   s_x  ,   _T(0)  ,  _T(0) ,
                      _T(0) ,    s_y   ,  _T(0) ,  
                      _T(0) ,   _T(0)  ,   s_z  ;
                    
        bias_vec_ <<  b_x , b_y , b_z ;

        define:
           ms_mat_ = mis_mat_*scale_mat_

        then the undistortion transform:
                X' = T*K*(X - B)

        can be implemented as:

       unbias_data = ms_mat_*(raw_data - bias_vec_)

     * assume body frame same as accelerometer frame, 
     * so bottom left params in the misalignment matris are set to zero */
    CalibratedTriad_<_T2> calib_triad(                       //    12 个内参
      //
      // TODO: implement lower triad model here         初始化内参
      //
      // mis_yz, mis_zy, mis_zx:                          安装误差
      _T2(0), _T2(0), _T2(0),                                  //  因为使用的是没有转台的模式，所以可以省去安装误差3个参数
      // mis_xz, mis_xy, mis_yx:                  
      params[0], params[1], params[2],                           
      //    s_x,    s_y,    s_z:
      params[3], params[4], params[5],          //  标度因素  
      //    b_x,    b_y,    b_z: 
      params[6], params[7], params[8]           //  零偏
    );
    
    // apply undistortion transform:
    Eigen::Matrix< _T2, 3 , 1> calib_samp = calib_triad.unbiasNormalize( raw_samp );
    
    residuals[0] = _T2 (g_mag_) - calib_samp.norm();

    return true;
  }
  
  static ceres::CostFunction* Create ( const _T1 &g_mag, const Eigen::Matrix< _T1, 3 , 1> &sample )
  {
    return ( new ceres::AutoDiffCostFunction< MultiPosAccResidual, 1, 9 > (            
               new MultiPosAccResidual<_T1>( g_mag, sample ) ) );
  }
  
  const _T1 g_mag_;
  const Eigen::Matrix< _T1, 3 , 1> sample_;
};
```

#### 3.2.3 优化结果

![autograde1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8Dautograde1.png)

![autograde2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8Dautograde2.png)

### 3.3  解析求导   MultiPosAccResidual_Analytical

参考博客：[深蓝学院-多传感器融合定位-第5章作业]

#### 3.3.1 加速度内参模型(按照开源代码中的定义)   雅克比推导 

a.安装误差 T（代码原先使用上三角形式, 改为下三角形式）,刻度系数误差K,零偏B
$$
T=\left[\begin{array}{rrr}
1 & 0 & 0 \\
s_{x z} & 1 & 0 \\
-s_{x y} & s_{y x} & 1
\end{array}\right] \quad  =    \left[\begin{array}{rrr}
1 & 0 & 0 \\
s_{1} & 1 & 0 \\
-s_{2} & s_{3} & 1
\end{array}\right] \quad
$$

$$
K_{a}^{\prime}=\left[\begin{array}{lll}
K_{a x}^{\prime} & & \\
& K_{a y}^{\prime} & \\
& & K_{a z}^{\prime}
\end{array}\right]=K_{a}^{-1}=\left[\begin{array}{lll}
\frac{1}{K_{a x}} & & \\
& \frac{1}{K_{e v}} & \\
& & \frac{1}{K_{a z}}
\end{array}\right]  = \left[\begin{array}{ccc}
\mathrm{k}_{1} & 0 & 0 \\
0 & k_{2} & 0 \\
0 & 0 & k_{3}
\end{array}\right] \quad
$$


$$
T=\left[\begin{array}{rrr}
1 & 0 & 0 \\
s_{1} & 1 & 0 \\
-s_{2} & s_{3} & 1
\end{array}\right] \quad K=\left[\begin{array}{ccc}
\mathrm{k}_{1} & 0 & 0 \\
0 & k_{2} & 0 \\
0 & 0 & k_{3}
\end{array}\right] \quad B=\left[\begin{array}{c}
b_{x} \\
b_{y} \\
b_{z}
\end{array}\right]
$$
b.待估计参数
$$
\theta^{a c c}=\left[\begin{array}{lllllllll}
s_{1} & s_{2} & s_{3} & k_{1} & k_{2} & k_{3} & b_{x} & b_{y} & b_{z}
\end{array}\right]
$$

c.给定加速度读数为 X, 对应的真实值为 X^ , 其计算公式如下:
$$
X^{\prime}=T * K *(X-B)
$$
残差：
$$
f\left(\theta^{a c c}\right)=\|g\|_{2}-\left\|X^{\prime}\right\|_{2}
$$
雅可比, 按照链式求导分解
$$
\frac{\partial f}{\partial \theta^{a c c}}=\frac{\partial f}{\partial\left\|X^{\prime}\right\|_{2}} \frac{\partial\left\|X^{\prime}\right\|_{2}}{\partial X^{\prime}} \frac{\partial X^{\prime}}{\partial \theta^{a c c}}
$$

$$
X=\left[\begin{array}{c}
A_{x} \\
A_{y} \\
A_{z}
\end{array}\right] \quad X^{\prime}=T * K *(X-B)=\left[\begin{array}{c}
k_{1}\left(A_{x}-b_{x}\right) \\
s_{1} k_{1}\left(A_{x}-b_{x}\right)+k_{2}\left(A_{y}-b_{y}\right) \\
-s_{2} k_{1}\left(A_{x}-b_{x}\right)+s_{3} k_{2}\left(A_{y}-b_{y}\right)+k_{3}\left(A_{z}-b_{z}\right)
\end{array}\right]
$$

$$
\frac{\partial f}{\partial\left\|X^{\prime}\right\|_{2}}=\frac{\partial\left(\|g\|_{2}-\left\|X^{\prime}\right\|_{2}\right)}{\partial\left\|X^{\prime}\right\|_{2}}=-1
$$

$$
\frac{\partial \left\|X^{\prime}\right\|_{2}}{\partial X^{\prime}}=\frac{X^{\prime}}{\left\|X^{\prime}\right\|_{2}}
$$

$$
\frac{\partial X^{\prime}}{\partial \theta^{a cc}}=\left[\begin{array}{cccccccc}0 & 0 & 0 & A_{x}-b_{x} & 0 & 0 & -k_{1} & 0 & 0 \\ k_{1}\left(A_{x}-b_{x}\right) & 0 & 0 & s_{1}\left(A_{x}-b_{x}\right) & A_{y}-b_{y} & 0 & -s_{1} k_{1} & -k_{2} & 0 \\ 0 & -k_{1}\left(A_{x}-b_{x}\right) & k_{2}\left(A_{y}-b_{y}\right) & -s_{2}\left(A_{x}-b_{x}\right) & s_{3}\left(A_{y}-b_{y}\right) & A_{z}-b_{z} & s_{2} k_{1} & -s_{3} k_{2} & -k_{3}\end{array}\right]
$$



#### 3.3.2 具体代码

参照第三章   [多传感器融合定位 第三章 3D激光里程计2](https://blog.csdn.net/weixin_41281151/article/details/120016673)中ALOAM的ceres slover  解析求导写法。因为delta_x的更新符合加减运算法则，所以这里不需要自定义ceres 参数块。

使用ceres，需要给为class 模板

##### Evaluate()   更新方程

```cpp
template <typename _T1>  
class  MultiPosAccResidual_Analytical   :  public   ceres::SizedCostFunction<1, 9>   {       // 残差维度 residual[0]：1     优化参数维度：9
public:  
        const _T1 g_mag_;
        const Eigen::Matrix< _T1, 3 , 1> sample_;
        MultiPosAccResidual_Analytical( const  _T1 &g_mag, 
                                                                                                             const Eigen::Matrix<_T1,3,1>&sample) 
                        : g_mag_(g_mag),     sample_(sample)   { }

virtual   bool   Evaluate(double  const  *const   *parameters,
                                                                                                double   *residuals,
                                                                                                double   **jacobians )  const                                 //   定义残差模型
{

        Eigen::Matrix<double, 3, 1> raw_samp(             //  观测数据
      double(sample_(0)), 
      double(sample_(1)), 
      double(sample_(2)) 
    );

       CalibratedTriad_<double> calib_triad(                       //    12 个内参
      //
      // TODO: implement lower triad model here         初始化内参
      //
      // mis_yz, mis_zy, mis_zx:                          安装误差
      double(0), double(0), double(0),                                  //  因为使用的是没有转台的模式，所以可以省去安装误差3个参数
      // mis_xz, mis_xy, mis_yx:                  
      parameters[0][0], parameters[0][1], parameters[0][2],                           
      //    s_x,    s_y,    s_z:
      parameters[0][3], parameters[0][4], parameters[0][5],          //  标度因素  
      //    b_x,    b_y,    b_z: 
      parameters[0][6], parameters[0][7], parameters[0][8]           //  零偏
    );

      // apply undistortion transform:
    Eigen::Matrix< double, 3 , 1> calib_samp = calib_triad.unbiasNormalize( raw_samp );

    residuals[0] = (double)g_mag_  -   calib_samp.norm();      //  残差
    
    if(jacobians  !=   nullptr) 
    {
                  if (jacobians[0]  !=   nullptr)
                  {

                        /*计算雅克比*/
                              //   安装误差
                          double  S1 =     parameters[0][0];
                          double  S2 =     parameters[0][1];
                          double  S3 =     parameters[0][2];
                          //   标度因素
                          double  K1 =     parameters[0][3];
                          double  K2 =     parameters[0][4];
                          double  K3 =     parameters[0][5];
                          //    零偏
                          double  Bx =     parameters[0][6];
                          double  By =     parameters[0][7];
                          double  Bz =     parameters[0][8];

                          // 计算出的真值输出
                          double  Ax =  raw_samp[0];
                          double  Ay =  raw_samp[1];
                          double  Az =  raw_samp[2];

                           Eigen::Matrix< double, 1, 9> Jacobian ;
                          Eigen::Vector3d  samp_norm  =     calib_samp  /  (calib_samp.norm() )  ;    //  输入数据的单位向量
                          Eigen::Matrix< double, 3, 9> J_theta  =  Eigen::Matrix<double,  3,   9>::Zero();          

                          J_theta(0,3) =  (Ax - Bx);
                          J_theta(0,6)  =  -K1;

                          J_theta(1,0)  =  K1*(Ax - Bx);
                          J_theta(1,3)  =  S1*(Ax - Bx);
                          J_theta(1,4)  =  Ay - By;
                          J_theta(1,6)  =  -S1*K1;
                          J_theta(1,7)  =  -K2;

                          J_theta(2,1)  =  -K1*(Ax - Bx);
                          J_theta(2,2)  =  K2*(Ay - By);
                          J_theta(2,3)  =  -S2*(Ax - Bx);
                          J_theta(2,4)  =  S3*(Ay - By);
                          J_theta(2,5)  =  Az - Bz;
                          J_theta(2,6)  =  S2*K1;
                          J_theta(2,7)  =  -S3*K2;
                          J_theta(2,8)  =  -K3;
                          Jacobian    =   - samp_norm.transpose()  * J_theta  ; 

                          jacobians[0][0] = Jacobian(0,0);
                          jacobians[0][1] = Jacobian(0,1);
                          jacobians[0][2] = Jacobian(0,2);
                          jacobians[0][3] = Jacobian(0,3);
                          jacobians[0][4] = Jacobian(0,4);
                          jacobians[0][5] = Jacobian(0,5);
                          jacobians[0][6] = Jacobian(0,6);
                          jacobians[0][7] = Jacobian(0,7);
                          jacobians[0][8] = Jacobian(0,8);
                  }
    }
    return  true;
}                                                                                                
} ;
```

##### 调用  new MultiPosAccResidual_Analytical

```cpp
       ceres::CostFunction *cost_function = new MultiPosAccResidual_Analytical<_T>(        //  沿用自动求导的函数接口，多态性
       g_mag_, static_samples[i].data());

      problem.AddResidualBlock ( 
        cost_function,           /* error fuction */
        NULL,                    /* squared loss */
        acc_calib_params.data()  /* accel deterministic error params */
      ); 
```

#### 3.3.3 优化结果

优化结果和自动求导结果一样！！！

![analytical1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8Danalytical1.png)

![analytical2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E5%9B%9B%E7%AB%A0%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E6%9E%84%E5%BB%BA%E5%8F%8A%E5%9F%BA%E4%BA%8E%E7%82%B9%E4%BA%91%E5%9C%B0%E5%9B%BE%E5%AE%9A%E4%BD%8Danalytical2.png)

### 4. 课程答疑总结

#### 4.1 重定位问题

问：定位这一块,借助 gps 能大致获取 pose(这里假设 gps 初始比较到位),初始的 orientation 只能沿各个方向去搜索是吧。然后当定位
丢了,现在普遍采用什么样的形式去进行重定位,已经在运行中了,点云在不断的变化,已经很难去定位了吧。

答：在有惯导的前提下，可以使用惯导的姿态来做重定位的初始化。

#### 4.2 在建图启动初期,汽车速度是否最好为低速或静止?

答：是的，低速或静止的情况下，建图效果会更好，过高的动态会有影响。

### 4.3 在长直道时,发现即使慢速播放,也会跑飞,暂时分析由于,长直道特征点较少,并且车辆较多,导致特征匹配混乱导致。请
问老师,在解决长直道的建图定位,有什么好的 trick 吗(imu + RTK +Lidar) 框架下。

答：使用优势互补，公开道路可以使用gps，组合导航可以把很多问题都解决一下，或者添加轮速计，或者加入路标约束。

### 4.4  室外环境下,有 RTK 进行位姿矫正,还有必要进行回环检测么?

答：非常有必要，RTK本身也有误差，假设RTK本身的位置误差为10cm(已经非常小),那么一辆车建图时，向北10cm误差，向南10cm误差，地图拼接起来是20cm的误差，容易导致产生重影，消除地图重影是非常必要的，引入回环检测可以有效的消除地图重影。关于为什么要使用回环检测，可参考： https://zhuanlan.zhihu.com/p/110711975



​																																														edited by kaho 2021.9.21
