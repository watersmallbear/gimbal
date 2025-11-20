#ifndef __PID_H__
#define __PID_H__

class Pid {
public:
    void Init(float kp, float ki, float kd, float dt);
    ~Pid() = default;
    void Control(float target,float current_data);
    float Result();
    void Angle_Normalization();
    void Derivative_Feedforward(float kff);
private:
    float kp_;
    float ki_;
    float kd_;
    float kff_;
    float last_error_;
    float dt_;
    float sum_error_;
    float result_;
    float max_output_;
    float min_output_;
    float error_;
    bool angle_flag_;
    float feedforward_;
    bool ff_flag_;
    float last_target_;
};

void Pid::Init(float kp, float ki, float kd, float dt) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    angle_flag_ = false;
    feedforward_ = 0;
    ff_flag_ = false;
    last_target_ = 0;
}

float Pid::Result(){

    return result_;
}

void Pid::Control(float target, float current_data) {
    // 1. 首先计算原始误差
    error_ = target - current_data;

    while (error_ > M_PI && angle_flag_)   error_ -= 2 * M_PI;
    while (error_ < -M_PI && angle_flag_)  error_ += 2 * M_PI;

    // 比例项
    float p_term = kp_ * error_;
    
    // 积分项（带抗饱和）
    float i_term = ki_ * sum_error_ * dt_;
    
    // 计算未限幅的输出
    float output = p_term + i_term + kd_ * (error_ - last_error_) / dt_;

    if (ff_flag_){

        feedforward_ = kff_ * (target - last_target_) / dt_;
    }
    
    sum_error_ += error_;
    
    result_ = output + feedforward_;
    last_error_ = error_;
    last_target_ = target;
}

void Pid::Angle_Normalization(){

    angle_flag_ = true;
}

void Pid::Derivative_Feedforward(float kff){

    ff_flag_ = true;
    kff_ = kff;
}

#endif