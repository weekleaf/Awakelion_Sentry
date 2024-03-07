#include "auto_aim/kalman/armor_kalman.h"
armor_kalman::armor_kalman()
{
    Q_ = 0.01f;
    R_ = 0.02f;
    t_ = 1.0f;
    x_ = 0.0f;
    p_ = 0.01f;
}

armor_kalman::armor_kalman(float Q, float R, float t, float x0, float p0)
{
    Q_ = Q;
    R_ = R;
    t_ = t;
    x_ = x0;
    p_ = p0;
}

void armor_kalman::setParam(int _Q, int _R, int _t)
{
    if (_R < 1) _R = 1;
    if (_Q < 1) _Q = 1;
    if (_t < 1) _t = 1;
    R_ = static_cast<float>(_R) * 0.01f;
    Q_ = static_cast<float>(_Q) * 0.01f;
    t_ = static_cast<float>(_t);
}

float armor_kalman::run(float _data)
{
    x_pre_ = x_;                               // x(k|k-1) = AX(k-1|k-1)+BU(k)
    p_pre_ = p_ + Q_;                          // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_ = p_pre_ / (p_pre_ + R_);           // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_ = x_pre_ + kg_ * (_data - x_pre_);  // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_ = (1 - kg_) * p_pre_;               // p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
}

float armor_kalman::mergeRun(float _data1, float _data2)
{
    x_pre_ = _data1;
    p_pre_ = p_ + Q_;                           // p(k|k-1) = Ap(k-1|k-1)A'+Q
    kg_ = p_pre_ / (p_pre_ + R_);            // kg(k)    = p(k|k-1)H'/(Hp(k|k-1)'+R)
    x_ = x_pre_ + kg_ * (_data2 - x_pre_);  // x(k|k)   = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p_ = (1 - kg_) * p_pre_;                // p(k|k)   = (I-kg(k)H)P(k|k-1)

    return x_;
}

