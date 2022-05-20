# 多传感器融合定位 三章 3D激光里程计2

参考博客：

[slam中ceres的常见用法总结](https://blog.csdn.net/qq_42700518/article/details/105898222)

参考代码：

[基于线面特征的激光里程计](https://zhuanlan.zhihu.com/p/348195351)

代码下载：

[https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%B8%89%E7%AB%A0%203D%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12/lidar_localization](https://github.com/kahowang/sensor-fusion-for-localization-and-mapping/tree/main/%E7%AC%AC%E4%B8%89%E7%AB%A0%203D%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12/lidar_localization)

## 环境配置问题：

### Sophus 编译

#### 问题1：系统找不到Sophus 库

![sophus2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%B8%89%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12sophus2.png)



#### 问题2：缺少 #include  <sophus/so3.hpp>

![](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%B8%89%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12sophus3.png)

#### 解决办法：

均是因为sophus库安装不对导致的，可以选择安装更加通用广泛的版本，[高博slam14讲中的sophus版本](https://github.com/gaoxiang12/slambook2/tree/master/3rdparty)

```bash
#下载高博slam14讲的sophus版本后
cd  Sophus
mkdir  build
cd  build
make -j8
sudo make install
```

#### 问题3：缺少 libglog.so

参考博客：

[make[2]: *** 没有规则可以创建“shared/libaplcommon-pdr.so”需要的目标“/usr/lib/x86_64-linux-gnu/libcrypto.so”。 停止。](https://blog.csdn.net/asia66/article/details/85284307)

![](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%B8%89%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12without%20libglog.so.png)

#### 解决办法：

```bash
sudo apt-get  install libgoogle-glog-dev
```

## 基于线面特征的位姿优化

#### 公式推导

注意：

1.这里的推导形式是分别更新R ，t  ，与  直接更新转换矩阵T的导数会有所不同

2.线面残差是一个数，为标量，而待优化量位移、姿态等是具有方向、大小，是矢量，在 线面残差(标量)求对应待优化变量(矢量)的jacobian时，链式法则时，需要求 标量对矢量的偏导，标量对矢量的偏导是一个单位向量，代表矢量的单位方向。

参考博客：[基于线面特征的激光里程计](https://zhuanlan.zhihu.com/p/348195351)

##### 点到线雅克比推导：

点到线的距离  :

$$
d_{\varepsilon}=\frac{\left|\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)\right|}{\left|p_{a}-p_{b}\right|}
$$
线特征残差雅可比：
$$
J_{\varepsilon}=\frac{\alpha d_{\varepsilon}}{\alpha T}=\frac{\alpha d_{\varepsilon}}{\alpha p_{i}^{\prime}} \frac{\alpha p_{i}^{\prime}}{\alpha T}
$$
对平移的雅可比：
$$
\frac{\alpha p_{i}^{\prime}}{\alpha t}=I
$$
对旋转的雅可比：
$$
\frac{\alpha p_{i}^{\prime}}{\alpha R}=-\left(R p_{i}\right)_{\times}
$$
求等号右边第一项的偏导数, 令
$$
X=\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)
$$
推导得：
$$
\begin{gathered}
\frac{\alpha d_{\varepsilon}}{\alpha p_{i}}=\frac{\alpha d_{\varepsilon}}{\alpha|X|} \frac{\alpha|X|}{\alpha X} \frac{\alpha X}{\alpha p_{i}} \\
=\frac{1}{\left|p_{a}-p_{b}\right|} \frac{\alpha|X|}{\alpha X} \frac{\alpha X}{\alpha p_{i}^{\prime}}=\frac{1}{\left|p_{a}-p_{b}\right|} \frac{X}{|X|} \frac{\alpha X}{\alpha p_{i}^{\prime}} \\
=\frac{1}{\mid p_{a}-p_{b}} \mid \frac{\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)}{\left|\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)\right|}\left(-\left(p_{i}^{\prime}-p_{a}\right)_{\times}+\left(p_{i}^{\prime}+p_{b}\right)_{\times}\right) \\
=\frac{\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)}{\left|\left(p_{i}^{\prime}-p_{b}\right) \times\left(p_{i}^{\prime}-p_{a}\right)\right|} \frac{\left(p_{a}-p_{b}\right)_{\times}}{\left|p_{a}-p_{b}\right|}
\end{gathered}
$$
其中,标量对矢量求导
$$
\quad \frac{\alpha|X|}{\alpha X}=\frac{\alpha\left(s \operatorname{qrt}\left(X^{T} X\right)\right)}{\alpha\left(X^{T} X\right)} \frac{\alpha\left(X^{T} X\right)}{\alpha(X)}=\frac{X}{|X|}
$$

##### 面到线雅可比推导：

点到面的距离：
$$
d_{H}=\left(p_{i}^{\prime}-p_{j}\right) \cdot \frac{\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)}{\left|\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)\right|}
$$
面特征残差雅可比：
$$
J_{H}=\frac{\alpha d_{H}}{\alpha T}=\frac{\alpha d_{H}}{\alpha p_{i}^{\prime}} \frac{\alpha p_{i}^{\prime}}{\alpha T}
$$
对平移和旋转的雅可比同上

对平移的雅可比：
$$
\frac{\alpha p_{i}^{\prime}}{\alpha t}=I
$$
对旋转的雅可比：
$$
\frac{\alpha p_{i}^{\prime}}{\alpha R}=-\left(R p_{i}\right)_{\times}
$$
求等号右边第一项的偏导数为：
$$
\frac{\alpha d_{H}}{\alpha p_{i}^{\prime}}=\frac{\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)}{\left|\left(p_{l}-p_{j}\right) \times\left(p_{m}-p_{j}\right)\right|}
$$

## Ceres 入门笔记

初级入门可参考高翔slam14讲，ceres 曲线拟合。

解析求导需要对雅克比进行赋值，因为ceres默认的更新形式是加减方式，而so3的四元数更新方式不同，所以需要自定义旋转参数块（顾名思义：自定义delta_q更新运算法则）

### 自定义旋转参数块：LocalParameterization参数化

参考博客： 

[Ceres详解（一） Problem类](https://blog.csdn.net/weixin_43991178/article/details/100532618)

[Ceres（二）LocalParameterization参数化](https://blog.csdn.net/hzwwpgmwy/article/details/86490556)

[优化库——ceres（二）深入探索ceres::Problem](https://blog.csdn.net/jdy_lyy/article/details/119360492)

​	对于四元数或者旋转矩阵这种使用过参数化表示旋转的方式，它们是不支持广义的加法（因为使用普通的加法就会打破其 constraint，比如旋转矩阵加旋转矩阵得到的就不再是旋转矩阵），所以我们在使用ceres对其进行迭代更新的时候就需要自定义其更新方式了，具体的做法是实现一个参数本地化的子类，需要继承于LocalParameterization，LocalParameterization是纯虚类，所以我们继承的时候要把所有的纯虚函数都实现一遍才能使用该类生成对象。

​	除了不支持广义加法要自定义参数本地化的子类外，如果你要对优化变量做一些限制也可以如法炮制，比如ceres中slam2d example中对角度范围进行了限制.

#### LocalParameterization   自定义实现

​	这里以四元数为例子，解释如何实现参数本地化，需要注意的是，QuaternionParameterization中表示四元数中四个量在内存中的存储顺序是[w, x, y, z]，而Eigen内部四元数在内存中的存储顺序是[x, y, z, w]，但是其构造顺序是[w, x, y, z]（不要被这个假象给迷惑），所以就要使用另一种参数本地化类，即EigenQuaternionParameterization，下面就以QuaternionParameterization为例子说明，如下：

```cpp
class CERES_EXPORT QuaternionParameterization : public LocalParameterization {
 public:
  virtual ~QuaternionParameterization() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const;
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};
```

##### 1.GlobalSize()

​	表示参数 x x*x* 的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵so3的自由度是3

##### 2.LocalSize()

​	表示 Δx 所在的正切空间（tangent space）的自由度，那么这个自由度是多少呢？下面进行解释，

正切空间是流形（manifold）中概念，对流形感兴趣的可以参考[2]，参考论文[3]，我们可以这么理解manifold：

```bash
A manifold is a mathematical space that is not necessarily Euclidean on a global scale, but can be seen as Euclidean on a local scale
```

​	上面的意思是说，SO3 空间是属于非欧式空间的，但是没关系，我们只要保证旋转的局部是欧式空间，就可以使用流形进行优化了. 比如用四元数表示的3D旋转是属于非欧式空间的，那么我们取四元数的向量部分作为优化过程中的微小增量（因为是小量，所以不存在奇异性）. 为什么使用四元数的向量部分？这部分可以参考[4]或者之前写的关于四元数的博客. 这样一来，向量部分就位于欧式空间了，也就得到了正切空间自由度是3.

##### 3.Plus()

​	自定义优化变量更新函数

##### 4.ComputeJacobian()

个人理解，是 定义待更新变量 和  待更新变量的delta 的雅克比

![ComputeJacobian](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%B8%89%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A12ComputeJacobian.png)

##### 5.Problem::AddParameterBlock()

自定义参数块后，需要通过   Problem::AddParameterBlock   自定义参数快加载

```cpp
void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)
```

### CostFunction  定义

重点参考博客：

[Ceres详解（一） Problem类](https://blog.csdn.net/weixin_43991178/article/details/100532618)

[Ceres详解（二） CostFunction](https://blog.csdn.net/weixin_43991178/article/details/100534230)

## 基于线面特征解析求导的实现

原工程移植了 aloam的自动求导，这里实现解析求导，公式推导如上“基于线面特征位姿优化/公式推导”所示

FILE:   lidar_localization/include/lidar_localization/models/loam/aloam_analytic_factor.hpp

#### 定义CostFunction

##### 解析求导-点线匹配 CostFunction

```cpp
class  EdgeAnalyticCostFunction   :  public   ceres::SizedCostFunction<1, 4,  3> {             // 优化参数维度：1     输入维度 ： q : 4   t : 3
public:
        double s;
        Eigen::Vector3d curr_point, last_point_a, last_point_b;
        EdgeAnalyticCostFunction(const Eigen::Vector3d curr_point_, const Eigen::Vector3d last_point_a_,
									   const Eigen::Vector3d last_point_b_, const double s_  )
                :  curr_point(curr_point_),   last_point_a(last_point_a_),   last_point_b(last_point_b_) ,  s(s_) {}

virtual  bool  Evaluate(double  const  *const  *parameters,  
                                                double  *residuals,  
                                                double  **jacobians) const                               //   定义残差模型
{     
        Eigen::Map<const  Eigen::Quaterniond>   q_last_curr(parameters[0]);               //   存放 w  x y z 
        Eigen::Map<const  Eigen::Vector3d>      t_last_curr(parameters[1]);
        Eigen::Vector3d  lp ;                           //   line point
        Eigen::Vector3d  lp_r ;
        lp_r =  q_last_curr*curr_point;
        lp     =   q_last_curr  * curr_point  + t_last_curr;   //   new point
        Eigen::Vector3d  nu =  (lp - last_point_a).cross(lp - last_point_b);
        Eigen::Vector3d  de = last_point_a  - last_point_b;

        residuals[0]  =  nu.norm()  /  de.norm();                              //  线残差

        //  归一单位化
        nu.normalize();

        if (jacobians !=  NULL)
        {
                if (jacobians[0]  !=  NULL)
                {
                        Eigen::Vector3d  re = last_point_b  -   last_point_a;
                        Eigen::Matrix3d  skew_re  =   skew(re);
                        Eigen::Matrix3d  skew_de  =   skew(de);

                        //  J_so3_Rotation
                        Eigen::Matrix3d   skew_lp_r  =  skew(lp_r);
                        Eigen::Matrix3d    dp_by_dr;
                        dp_by_dr.block<3,3>(0,0)  =  -skew_lp_r;
                        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> J_so3_r(jacobians[0]);
                        J_so3_r.setZero();
                        J_so3_r.block<1,3>(0,0)  =   nu.transpose()* skew_de * dp_by_dr / (de.norm()*nu.norm());
       
                        //  J_so3_Translation
                        Eigen::Matrix3d  dp_by_dt;
                        (dp_by_dt.block<3,3>(0,0)).setIdentity();
                        Eigen::Map<Eigen::Matrix<double,  1,  3,  Eigen::RowMajor>> J_so3_t(jacobians[1]);
                        J_so3_t.setZero();
                        J_so3_t.block<1,3>(0,0)  =   nu.transpose()  *  skew_de / (de.norm()*nu.norm());
                }
        }
        return  true;
}     
};
```

##### 解析求导-点面匹配 CostFunction

```cpp
class PlaneAnalyticCostFunction  :   public  ceres::SizedCostFunction<1, 4, 3>{
public:
	Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
	Eigen::Vector3d ljm_norm;
	double s;

        PlaneAnalyticCostFunction(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_,
					 Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_, double s_)
		: curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_),last_point_m(last_point_m_), s(s_){}

        virtual  bool  Evaluate(double  const  *const  *parameters, 
                                                         double  *residuals, 
                                                          double  **jacobians)const {      //   定义残差模型
                // 叉乘运算， j,l,m 三个但构成的平行四边面积(摸)和该面的单位法向量(方向)
                Eigen::Vector3d  ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
		ljm_norm.normalize();    //  单位法向量

                Eigen::Map<const Eigen::Quaterniond>  q_last_curr(parameters[0]);
                Eigen::Map<const Eigen::Vector3d> t_last_curr(parameters[1]);

                Eigen::Vector3d  lp;      // “从当前阵的当前点” 经过转换矩阵转换到“上一阵的同线束激光点”
                Eigen::Vector3d  lp_r = q_last_curr *  curr_point ;                        //  for compute jacobian o rotation  L: dp_dr
                lp = q_last_curr *  curr_point  +  t_last_curr;  

                // 残差函数
                double  phi1 =  (lp - last_point_j ).dot(ljm_norm);
                residuals[0]  =   std::fabs(phi1);

                if(jacobians != NULL)
                {
                        if(jacobians[0] != NULL)
                        {
                                phi1 = phi1  /  residuals[0];
                                //  Rotation
                                Eigen::Matrix3d  skew_lp_r  = skew(lp_r);
                                Eigen::Matrix3d  dp_dr;
                                dp_dr.block<3,3>(0,0) =  -skew_lp_r;
                                Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>>  J_so3_r(jacobians[0]);
                                J_so3_r.setZero();
                                J_so3_r.block<1,3>(0,0) =  phi1 *  ljm_norm.transpose() *  (dp_dr);

                                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>  J_so3_t(jacobians[1]);
                                J_so3_t.block<1,3>(0,0)  = phi1 * ljm_norm.transpose();                                                                                                                                                                                                                                            
                        }
                }
                return  true;
        }
};
```

##### Eigen 反对称矩阵函数定义

```cpp
Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1>& mat_in){                 //  反对称矩阵定义
    Eigen::Matrix<double,3,3> skew_mat;
    skew_mat.setZero();
    skew_mat(0,1) = -mat_in(2);
    skew_mat(0,2) =  mat_in(1);
    skew_mat(1,2) = -mat_in(0);
    skew_mat(1,0) =  mat_in(2);
    skew_mat(2,0) = -mat_in(1);
    skew_mat(2,1) =  mat_in(0);
    return skew_mat;
}
```

#### 定义旋转参数块

##### 使用ceres自带的四元数更新参数块 

通过观察自动求导的源码，可以发现，在自动求导中，使用了ceres的一个自定义四元数参数块

FILE:  lidar_localization/src/models/loam/aloam_registration.cpp

使用EigenQuaternionParameterization  参数块

```cpp
 config_.q_parameterization_ptr = new ceres::EigenQuaternionParameterization();    
```

加载参数块

```cpp
problem_.AddParameterBlock(param_.q, 4, config_.q_parameterization_ptr);            //  加载自定义旋转参数块
```

##### 自己定义四元数更新参数块

FILE:   lidar_localization/include/lidar_localization/models/loam/aloam_analytic_factor.hpp

参考博客 ：

[优化库——ceres（二）深入探索ceres::Problem](https://blog.csdn.net/jdy_lyy/article/details/119360492)

[[ceres-solver] From QuaternionParameterization to LocalParameterization](https://www.cnblogs.com/JingeTU/p/11707557.html)

这里需要注意的是自定义时，输入的 q :四元数 4维 ， 更新以 so3的形式更新 3 维 ， 所以GlobalSize：4  LocalSize：3  具体体现在  ComputeJacobian()  函数中

```cpp
//  自定义旋转残差块
//参考博客   https://blog.csdn.net/jdy_lyy/article/details/119360492
class PoseSO3Parameterization  :   public  ceres::LocalParameterization {                               //  自定义so3  旋转块
 public:       
        PoseSO3Parameterization()  { }

        virtual ~PoseSO3Parameterization() { }
  
        virtual bool Plus(const double* x,
                        const double* delta,
                        double* x_plus_delta) const          //参数正切空间上的更新函数
                { 
                        Eigen::Map<const  Eigen::Quaterniond>   quater(x);       //   待更新的四元数
                        Eigen::Map<const  Eigen::Vector3d>     delta_so3(delta);     //    delta 值,使用流形 so3 更新

                        Eigen::Quaterniond  delta_quater  =   Sophus::SO3d::exp(delta_so3).unit_quaternion();     //   so3 转换位 delta_p  四元数
                        
                        Eigen::Map<Eigen::Quaterniond>  quter_plus(x_plus_delta);    //   更新后的四元数

                        // 旋转更新公式
                        quter_plus =  (delta_quater*quater).normalized();        

                        return  true;
                }

        virtual bool ComputeJacobian(const double* x, double* jacobian) const  //  四元数对so3的偏导数
        {
                Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
                (j.topRows(3)).setIdentity();
                (j.bottomRows(1)).setZero();

                return true;
        }

        // virtual bool MultiplyByJacobian(const double* x,
        //                                 const int num_rows,
        //                                 const double* global_matrix,
        //                                 double* local_matrix) const;//一般不用

        virtual int GlobalSize() const  {return  4;} // 参数的实际维数
        virtual int LocalSize() const   {return  3;} // 正切空间上的参数维数
};

```

## 调用

FILE: lidar_localization/src/models/loam/aloam_registration.cpp

### 解析求导

为了方便调用，沿用自动求导的调用方式

定义宏

```cpp
#define  autograde                  //  使用自动求导  or  解析求导
```

点线匹配

```cpp
bool CeresALOAMRegistration::AddEdgeFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y,
    const double &ratio
) {

    /*自动求导*/
  #ifdef  autograde
    ceres::CostFunction *factor_edge = LidarEdgeFactor::Create(                  //   创建误差项
        source, 
        target_x, target_y, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_edge,                                    //   约束边   cost_function
        config_.loss_function_ptr,        //   鲁棒核函数  lost_function
        param_.q, param_.t                      //  关联参数
    );

    #else
   /*解析求导*/
   ceres::CostFunction *factor_analytic_edge =   new EdgeAnalyticCostFunction(
        source, 
        target_x, target_y, 
        ratio
   );

    problem_.AddResidualBlock(
        factor_analytic_edge,                                    //   约束边   cost_function
        config_.loss_function_ptr,        //   鲁棒核函数  lost_function
        param_.q, param_.t                      //  关联参数
    );

#endif
    return true;
}
```

点面匹配

```cpp
bool CeresALOAMRegistration::AddPlaneFactor(
    const Eigen::Vector3d &source,
    const Eigen::Vector3d &target_x, const Eigen::Vector3d &target_y, const Eigen::Vector3d &target_z,
    const double &ratio
) {

    /*自动求导*/
    #ifdef  autograde
    ceres::CostFunction *factor_plane = LidarPlaneFactor::Create(
        source, 
        target_x, target_y, target_z, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_plane,
        config_.loss_function_ptr, 
        param_.q, param_.t
    );

#else
   /*解析求导*/
    ceres::CostFunction *factor_analytic_plane =new PlaneAnalyticCostFunction(
        source, 
        target_x, target_y, target_z, 
        ratio
    );

    problem_.AddResidualBlock(
        factor_analytic_plane,
        config_.loss_function_ptr, 
        param_.q, param_.t
    );

#endif

    return true;
}
```



### 自定义旋转参数块

定义宏

```cpp
#define  maunual_block_loder                    //  使用ceres 自带EigenQuaternionParameterization  参数块   or  自定义 PoseSO3Parameterization 旋转参数块
```

```cpp
#ifdef  maunual_block_loder
	config_.q_parameterization_ptr =  new  PoseSO3Parameterization() ;                    //  自定义旋转参数块
#else 
	config_.q_parameterization_ptr = new ceres::EigenQuaternionParameterization();          //   SE3 转换矩阵/位姿参数化，
#endif
```

## evo 评价

### 自动求导 + Ceres 自带四元数参数块

```shell
whole mapping time 179.438701 ms +++++
```

<p align ="center"><img src="/home/lory/Pictures/2022-01-14 23-02-36 的屏幕截图.png" alt="2022-01-14 23-02-36 的屏幕截图" width = 40%  height =40%;" /></p>

#### evo_rpe

```shell
evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz
       max	32.674307
      mean	1.727705
    median	0.949400
       min	0.174067
      rmse	4.633799
       sse	3134.926234
       std	4.299667
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2023-05-44%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 23-05-44 的屏幕截图" width = 40%  height =40%;" /></p>

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2023-05-54%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 23-05-54 的屏幕截图" width = 40%  height =40%;" /></p>

#### evo_ape

```shell
 evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz
        max	49.216907
      mean	17.845561
    median	15.060993
       min	0.000000
      rmse	21.164576
       sse	6550216.305710
       std	11.378720
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2023-09-28%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 23-09-28 的屏幕截图" width = 40%  height =40%;" /></pp>

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2023-09-39%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 23-09-39 的屏幕截图" width = 40%  height =40%;" /></p>

### 解析求导 + Ceres 自带四元数参数块

```shell
whole mapping time 174.593543 ms +++++
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-27-06%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-27-06 的屏幕截图"  width = 40%  height =40%;" /></p>

#### evo_rpe

```shell
evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz

       max	32.674307
      mean	1.752770
    median	1.073263
       min	0.174067
      rmse	4.309392
       sse	1207.105821
       std	3.936833
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-15-35%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-15-35 的屏幕截图"  width = 40%  height =40%;" />


<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-16-38%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-16-38 的屏幕截图"  width = 40%  height =40%;" /></p>

#### evo_ape

```shell
 evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz
       max	49.216907
      mean	17.908280
    median	15.086003
       min	0.000000
      rmse	21.509767
       sse	3021235.643653
       std	11.914847
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-22-40%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-22-40 的屏幕截图"  width = 40%  height =40%;" /></p>

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-23-54%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-23-54 的屏幕截图" width = 40%  height =40%;" /></p>

### 解析求导  + 自定义参数块

```shell
whole mapping time 170.378546 ms +++++
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-40-50%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-40-50 的屏幕截图" width = 40%  height =40%;" /></p>

#### evo_rpe

```shell
    evo_rpe kitti ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot --plot_mode xyz
    max	32.674307
      mean	1.727784
    median	1.003377
       min	0.174067
      rmse	4.634043
       sse	2254.806880
       std	4.299897
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-42-47%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-42-47 的屏幕截图" width = 40%  height =40%;" /></p>

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-42-56%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-42-56 的屏幕截图"  width = 40%  height =40%;" /></p>

#### evo_ape

```shell
      evo_ape kitti ground_truth.txt laser_odom.txt -r full --plot --plot_mode xyz
      max	49.216907
      mean	17.860616
    median	15.072742
       min	0.000000
      rmse	21.251952
       sse	4769827.822403
       std	11.517112
```

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-46-11%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-46-11 的屏幕截图" width = 40%  height =40%;" /></p>

<p align ="center"><img src="https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A112022-01-14%2020-46-26%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png" alt="2022-01-14 20-46-26 的屏幕截图"width = 40%  height =40%;" /></p>

### 总结：

​       图中结果可以看出，缺乏回环矫正的情况下在拐弯的情况下由于实现流程和缺乏回环导致误差会偏大，解析求求导和自动求导的结果相似。总体来看轨迹的趋势还是正确的，算法应该没有问题，但是参数调整还有改进的余地。

​																																													                                                                                                              edit   by   kaho  2021.8.31



## 问题记录：

### 1.**为什么lego-loam 中，面特征优化的是roll pitch z, 线特征优化的是yaw  x y ?**

lego-loam中面特征指的是地面，线特征指的是竖直的线，从自由度的角度说，地面只能约束roll pitch z,而竖直的线只能约束yaw x y。将loam中的6自由度拆分为2个3自由度，可以加速算法运算的效率。

### 2.定位建图中，都需要把传感器输出的坐标系斗转换到导航系下吗？导航系指的是ENU？

只有融合了gnss的坐标系，才需要把坐标系归一到导航系下（ENU）

### 3.如果想要把客户才能的框架应用到自己的平台中，出了修改kitti点云去畸变这个部分，还有什么环节需要注意的吗？

还有**“外参标定**环节，kitti数据转为rosbag，外参是通过tf-tree往外发的，可以写一个外参的配置文件，加载外参。

### 4.雷达是否属于视觉？是不是还有广义的visual?

属于一部分

### 5.loam一般只用于实时里程计，因为容易收到视角的影响，如果是固定的一条路线，比如地铁，可不可以使用loam进行建图或者定位。

可以的，loam可以本质可以认为是 线面匹配的ICP

### 6.1前端里程计作用：

​			建图：进行点云拼接过程中，提供 边(相对位姿)

​			定位：融合定位过程中会加入里程计进行融合

目前主流的定位方案是基于先验地图匹配。

### 6.2 ndt icp  loam 点云配准建图的精度、效率比较？

#### 效率：ndt < icp < loam

​		**ndt**  最慢 ； **icp** 较快，当使用**icp_svd** 解析求导时，速度会更快；loam 最快，因为线面特征比点对要少。

#### 精度：取决于环境

​		**loam** 特征点比较丰富的地方，效果好，不容易陷入局部最优；

​		**icp** 环境不太恶劣，相对环境变化不大，效果比较好；

​		**ndt** 因为ndt是经过概率分布进行匹配，所以在复杂可变的环境下，鲁棒性较强。

#### 总结：

​	**loam** 适用在线面特征丰富的场景。

​	**icp** 环境变化不大的场景。

​	**ndt** 在环境变化较大的场景下，仍能保持比较高的鲁棒性。（such as 建图在第一车道，定位在第三车道，ndt会比较好）。

​	**建图以ICP为主，定位(基于地图定位)以NDT为主，里程计以LOAM为主**   01:17:26

###  7.AHRS

使用AHRS进行位置解算，并不合理，因为连锁反应会导致最终误差很大。AHRS应该用来解算位姿，AHRS+GPS解算位置或使用DR系统（AHRS+轮速里程计）进行解算。

### 8.针对自动驾驶环境，通常真值轨迹如何制作，是通过RTK方法吗？一般选择的设备厂家和价格大概是什么情况？

真值定义：比实际的定位需求高一个数量级就ok。譬如：自动驾驶中需要的精度要求是20cm，真值要求5cm就ok；矿山中，定位要求米级，对真值要求达到分米级即可。

### 9.在后端优化时使用位姿约束为什么效果会变差？

后端优化中，是否使用位姿约束，取决于获取位姿的传感器的精度大小，在kitti数据集中，进行最后的后端优化，效果并没有太差，原因是kitti数据集的roll pitch

yaw 精度比较高，当使用千元级别或者百元级别的惯导时，roll pitch yaw 姿态角就不太准，这是位姿约束效果并不会太好。

eg. 一般情况下最好还是不要加姿态先验约束，因为IMU不受外界影响反映的是系统内部情况，lidar受到外界影响反映的是外部的真是情况，建图和定位都是要根据外部环境的，所以理论上lidar的状态估计更能反映外部的真实情况。GNSS提供的姿态可以用在为配准提供intial guess 或者适当用插值修正激光里程计的roll pitch （LOAM LIO-SAM 好像有这样做，但乘的系数很小）。

​																																																					edit   by   kaho  2021.9.7
