#include "Robomaster.h"

Robomaster::Robomaster(motor_type type_, Controll_mode mode_, short *command_value, short *angle_, short *speed_, short *torque_, short *temp_, CalPID &sc, CalPID &ac)
{
    type = type_;
    mode = mode_;
    if(type_ == M2006){
        current = command_value;
        setGearRatio(36); //デフォルトのギアヘッドのギア比
        setPowerMax(9500);
    }
    else if(type_ == M3508){
        current = command_value;
        setGearRatio(3591.0/187); //デフォルトのギアヘッドのギア比
        setPowerMax(16000);
    }
    // else {
    //     voltage = command_value;
    //     setGearRatio(1); //デフォルトのギアヘッドのギア比
    //     setPowerMax(29000);
    // }
    r = angle_;
    s = speed_;
    torque = torque_;
    temperature = temp_;
    sc_pid = &sc;
    ac_pid = &ac;
    setDeltaTime(0.001); //周期が0.001sでない場合はmain文で変更
    setAccelMax(200);
    setTemperatureMax(80);
    pre_target_omega = 0;
    angle_initial = 0;
    angle = 0;
    pre_r = 0;
}

void Robomaster::setGearRatio(float r){
    gear_ratio = r;
}

void Robomaster::calAngle(){
    float r_rad = (*s)*0.1047198; //角速度rad/s
    angle += (r_rad + pre_r) * delta_t * 0.5; //台形による面積近似

    float rem = fmod(angle,2*PI); //余り
    if(rem < 0) rem += 2*PI; //fmodは負の数で負の余りを返す
    angle -= rem;

    float now_r = *r;
    //angle_initial(angleの初期値)による修正
    if(now_r < angle_initial) now_r = 8192 - angle_initial + now_r;
    else now_r -= angle_initial;

    float angle_encoder = now_r * 0.00076699; //radに変換　r/8192*2π

    //encoder値との差がπ/2以内であればencoder値で補正
    if(angle_encoder - rem < PI*0.5 && angle_encoder - rem > -PI*0.5) rem = angle_encoder;
    angle += rem;

    pre_r = r_rad;
}

float Robomaster::getAngle(){
    //モードごとの戻り値の計算
    float angle_return = 0;
    if(mode == mode1_normal){ //モード１
        angle_return = angle;
    }
    if(mode == mode2_gear){ //モード２
        angle_return = fmod(angle/gear_ratio, 2*PI);
        if(angle_return < 0) angle_return += 2*PI;
    } 
    if(mode == mode3_gear_limitless){ //モード３
        angle_return = angle/gear_ratio;
    }
    return angle_return;
}

float Robomaster::getOmega(){
    float omega;
    if(mode == mode1_normal){ //モード１
        omega = (*s)*0.1047198;
    }
    if(mode == mode2_gear){ //モード２
        omega =  (*s) * 0.1047198 / gear_ratio;
        /////(*s)×(2π/60)/gear_ratio
    } 
    if(mode == mode3_gear_limitless){ //モード３
        omega =  (*s) * 0.1047198 / gear_ratio;
        /////(*s)×(2π/60)/gear_ratio
    }
    return omega;
}

float Robomaster::calSc(float target_omega_input){
    /////加速度制限
    float target_omega = target_omega_input;
    if ((target_omega_input - pre_target_omega) > acceleration_max)
    {
        target_omega = pre_target_omega + acceleration_max;
    }
    else if ((target_omega_input - pre_target_omega) < -acceleration_max)
    {
        target_omega = pre_target_omega - acceleration_max;
    }
    pre_target_omega = target_omega;

    /////PID計算
    float error = target_omega - getOmega();

    float current_ = sc_pid -> calPID(error);

    /////出力計算
    current_ = limitValue(current_, current_max, -current_max);
    return current_;
}

void Robomaster::Sc(float target_omega){
    float current_ = calSc(target_omega);
    turn(current_);
}

void Robomaster::Ac(float target_angle){
    calAngle();
    float error = target_angle - getAngle();

    if(mode == mode2_gear){
        if(error > PI) error -= 2*PI;
        if(error < -PI) error += 2*PI; 
    }

    float omega = ac_pid -> calP_D(error,getOmega());

    turn(calSc(omega));
}

void Robomaster::turn(float current_){
    if(*temperature < temperature_max){
        *current = limitValue(current_, current_max, -current_max);
    }
    else stop();
}

void Robomaster::stop(){
    *current = 0;
}

void Robomaster::reset(){
    angle_initial = (*r);
    angle = 0;
    pre_r = 0;
}

float Robomaster::limitValue(float value, float max, float min)
{
    float value_return = value;
    if (value_return > max)
        value_return = max;
    else if (value_return < min)
        value_return = min;
    return value_return;
}

void Robomaster::setPowerMax(float power_max)
{
    if(type == M2006){
        if (0 <= power_max && power_max <= 10000)
            current_max = power_max;
        else 
            current_max = 9500;
    }
    else if(type == M3508){
        if (0 <= power_max && power_max <= 16384)
            current_max = power_max;
        else 
            current_max = 16000;
    }
}

void Robomaster::setMaxScPID(float pid_max_sc){
    sc_pid->setMaxValue(pid_max_sc);
}

void Robomaster::setMaxAcPD(float pid_max_ac){
    ac_pid->setMaxValue(pid_max_ac);
}

void Robomaster::setPIDSc(float kp, float ki, float kd){
    sc_pid->setParameter(kp, ki, kd);
}

void Robomaster::setPDAc(float kp, float kd){
    ac_pid->setParameter(kp, 0, kd);
}

void Robomaster::setDeltaTime(float dt){
    delta_t = dt;
    sc_pid->setDELTA_T(dt);
    ac_pid->setDELTA_T(dt);
}

void Robomaster::setAccelMax(float a_max){
    acceleration_max = a_max;
}

void Robomaster::setTemperatureMax(float temp){
    if(0 < temp && temp < 125)
        temperature_max = temp;
    else
        temperature_max = 80;
}