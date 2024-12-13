#ifndef ROBOMASTER_H
#define ROBOMASTER_H
/*
Robomasterのモーター(M2006,M3508,GMM6020)のためのライブラリ
*/

#include "mbed.h"
#include "CalPID.h"

enum motor_type{
    M2006,
    M3508,
    //GM6020,　未実装
};
/*
モード切替
mode1:元のモーターの速度、角度を制御する。ギアは考慮しない。角度制御は減速’前’の角度を-∞ ~ ∞(rad)で指定する。
mode2:ギアによる減速先の速度、角度を制御する。角度制御は減速先の角度を0-2πで指定する。
    　例）無限回転独ステのステア
mode3:ギアによる減速先の速度、角度を制御する。角度指令値がlimitless。角度制御は減速先の角度を-∞ ~ ∞(rad)で指定する。
    　例）有限回転独ステのステア,モーター投擲

※モードによってPIDのゲインは大きく異なるので注意
*/
enum Controll_mode{
    mode1_normal,
    mode2_gear,
    mode3_gear_limitless,
};

class Robomaster
{
    public:
        Robomaster(motor_type type_, Controll_mode mode_, short *command_value, short *angle_, short *speed_, short *torque_, short *temp_, CalPID &sc, CalPID &ac);

        void calAngle();
        float getAngle();
        float getOmega();
        void setGearRatio(float r);
        void setEquation(float slope_f, float intercept_f, float slope_b, float intercept_b); //GM6020用
        void Sc(float target_omega);
        void Ac(float target_angle);
        void turn(float current_);
        void stop();
        void reset();   //0点合わせ用　角度のリセット

        
        //設定用関数
        void setPowerMax(float power_max);                // 最終的な出力の最大値設定
        void setMaxScPID(float pid_max_sc);                  // CalPIDによるScのpid最大値設定
        void setMaxAcPD(float pid_max_ac);                   // CalPIDによるAcのpid最大値設定
        void setPIDSc(float kp, float ki, float kd);         //速度制御のPDゲイン設定用関数
        void setPDAc(float kp, float kd);                    //角度制御のPDゲイン設定用関数
        void setDeltaTime(float dt);                         //制御周期の設定用関数
        void setAccelMax(float a_max);
        void setTemperatureMax(float temp);

    private:
        motor_type type;
        Controll_mode mode;

        /////送信用変数
        short *current; //電流（m2006,m3508用）
        short *voltage; //電圧（gm6020用）
        /////受信用変数
        short *torque, *temperature;
        short *r, *s; //angle,speed

        float angle,pre_r;
        float angle_initial;
        float gear_ratio;
        float delta_t;

        float current_max;
        float voltage_max;
        float acceleration_max;
        float pre_target_omega;
        short temperature_max;

        float slope_forward_rotation,intercept_forward_rotation; //GM6020用
        float slope_backward_rotation,intercept_backward_rotation;

        CalPID *sc_pid;
        CalPID *ac_pid;

        const float PI = 3.141592;

        float limitValue(float value, float max, float min);
        float calSc(float target_omega);

};

#endif