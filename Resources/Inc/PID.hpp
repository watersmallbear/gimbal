#ifndef __PID_H__
#define __PID_H__

class Pid {
public:
    void Init(float kp, float ki, float kd, float dt, 
    float max_output = 15000, float min_output = -15000);
    ~Pid() = default;
    void Control(float target,float current_data);
    float Result();
private:
    float kp_;
    float ki_;
    float kd_;
    float last_error_;
    float dt_;
    float sum_error_;
    float result_;
    float max_output_;
    float min_output_;
    float error_;
};

void Pid::Init(float kp, float ki, float kd, float dt, float max_output, float min_output) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dt_ = dt;
    max_output_ = max_output;
    min_output_ = min_output;
}

float Pid::Result(){

    return result_;
}

void Pid::Control(float target, float current_data) {
    // 1. 首先计算原始误差
    error_ = target - current_data;

    // 比例项
    float p_term = kp_ * error_;
    
    // 积分项（带抗饱和）
    float i_term = ki_ * sum_error_ * dt_;
    
    // 计算未限幅的输出
    float output = p_term + i_term + kd_ * (error_ - last_error_) / dt_;
    
    // 输出限幅
    if (output > max_output_) {
        output = max_output_;
        // 不累积积分项当输出饱和时 - 这是正确的抗饱和
    } else if (output < min_output_) {
        output = min_output_;
        // 不累积积分项当输出饱和时 - 这是正确的抗饱和
    } else {
        // 【修复】只有输出未饱和时才累积积分
        // 移除了 error_ <= 3000 的条件，因为误差已经归一化
        sum_error_ += error_;
    }
    
    result_ = output;
    last_error_ = error_;
}

#endif