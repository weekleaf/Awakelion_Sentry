class armor_kalman {
public:
    armor_kalman();
    armor_kalman(float Q, float R, float t, float x0, float p0);
    void setParam(int _Q, int _R, int _t);
    float run(float _data);
    float mergeRun(float _data1, float _data2);

private:
    float R_;
    float Q_;
    float p_pre_;
    float x_pre_;
    float x_;
    float p_;
    float kg_;
    float t_;
};
