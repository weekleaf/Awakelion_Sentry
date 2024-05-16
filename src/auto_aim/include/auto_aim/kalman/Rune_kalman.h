#ifndef _RUNE_KALMAN_H_
#define _RUNE_KALMAN_H_
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace std;
class Rune_Kalman {

private:
    static Eigen::Vector3d x_k1; // k-1时刻的滤波值，即是k-1时刻的值
    Eigen::Matrix3d K;    // Kalman增益
    Eigen::Matrix3d A;    // 转移矩阵
    Eigen::Matrix3d H;    // 观测矩阵
    Eigen::Matrix3d R;    // 预测过程噪声偏差的方差
    Eigen::VectorXd Q;    // 测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    static Eigen::Matrix3d P;    // 估计误差协方差

    double t;
public:
    Rune_Kalman() = default;

    Rune_Kalman(Eigen::Matrix3d A, Eigen::Matrix3d H, Eigen::Matrix3d R, Eigen::VectorXd Q, Eigen::Vector3d init, double t) {
        reset(A, H, R, Q, init, t);
    }

    void reset(Eigen::Matrix3d A, Eigen::Matrix3d H, Eigen::Matrix3d R, Eigen::VectorXd Q, Eigen::Vector3d init, double t) {
        this->A = A;
        this->H = H;
        this->P = Eigen::Matrix3d::Zero();
        this->R = R;
        this->Q = Q;
        x_k1 = init;
        this->t = t;
    }

    void reset(Eigen::Vector3d init, double t) {
        x_k1 = init;
        this->t = t;
    }

    void reset(double x1, double x2,double x3, double t) {
        x_k1<<x1,x2,x3;
        this->t = t;
    }

   Eigen::Vector3d update(Eigen::Vector3d z_k, double t) {
        // 设置转移矩阵中的时间项
        this->t=t;
        for (int i = 1; i < 3; ++i) {
            A(i - 1, i) = t;
        }
        A(0,2)=pow(t,2)/2;
#ifdef DEBUG_MODE
        cout<<"A:"<<A<<endl;
#endif

        // 预测下一时刻的值
        if(isnan(x_k1(0))||x_k1(0)==0&&x_k1(1)==0&&x_k1(2)==0)
            x_k1=z_k;
#ifdef DEBUG_MODE
        cout<<"x_k1 first:"<<x_k1<<endl;
#endif
        Eigen::Vector3d p_x_k = A * x_k1;   //x的先验估计由上一个时间点的后验估计值和输入信息给出
#ifdef DEBUG_MODE
        cout<<"p_x_k:"<<p_x_k<<endl;
#endif

        //求协方差
#ifdef DEBUG_MODE
        cout<<"P first:"<<P<<endl;
#endif
        P = A * P * A.transpose() + R;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q

        if(isnan(P(0,0))){
            this->P=Eigen::Matrix3d::Zero();

            std::cout<<"PPPPPPPPPPPPPPPP nan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout<<"is it recover??????????????????????????????????????????????????????????????????"<<std::endl;
            return z_k;
        }

#ifdef DEBUG_MODE
        cout<<"P second:"<<P<<endl;
#endif

        //计算kalman增益
        K = P * H.transpose() * (H * P * H.transpose()).inverse();  //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
        if(isnan(K(0,0))){
            this->K=Eigen::Matrix3d::Zero();

            std::cout<<"KKKKKKKKKKKKKK nan!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout<<"is it recover??????????????????????????????????????????????????????????????????"<<std::endl;
            return z_k;
        }
#ifdef DEBUG_MODE
        cout<<"K :"<<K<<endl;
#endif
        //修正结果，即计算滤波值
        x_k1 = p_x_k +
            K * (z_k - H * p_x_k);  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
#ifdef DEBUG_MODE
        cout<<"x_k1 second:"<<x_k1<<endl;
#endif
     //更新后验估计
        P = (Eigen::Matrix3d::Identity() - K * H) * P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
#ifdef DEBUG_MODE
        cout<<"P third:"<<P<<endl;
#endif

        return x_k1;
    }


};

#endif //_RUNE_KALMAN_H_

