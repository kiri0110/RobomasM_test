#pragma once
#include "CalPID.h"
#include "Robomaster.h"

enum Type {m2006 , m3508 , gm6020 };

class RobomasM {
private:
    Type type;

    static constexpr float DELTA_T = 0.001; //制御周期 1kHz
    static constexpr float DUTY_MAX = 10000; //dutyの絶対値の上限を決めて暴走を防ぐ
    static constexpr float OMEGA_MAX = 100; //角速度を格納する配列の要素数
    static constexpr short NUM_DATA = 200;
    static constexpr short SAVE_COUNT_THRESHOLD = 0; //データ保存の周期調整用。1kHzで保存すると1秒で1000個もデータが溜まってしまう

    float target_speed = 0.0;

    int omega_save_count = 0;
    //角速度を保存する変数と関数
    short omega_saved[NUM_DATA] = {}; //デバッグ用の保存配列だが、RAM圧迫しすぎ... 必要に応じてメモリ確保したい
    int omega_count = 0;

    float target_angle = 0.0;

    int angle_save_count = 0;
    //角度を保存する変数と関数
    short angle_saved[NUM_DATA] = {};
    int angle_count = 0;

protected:
public:
    //送信用変数
    short current = 0; //目標出力電流 
    //受信用変数
    short angle = 0; // 角度
    short speed = 0; //回転速度
    short torque = 0; //トルク電流
    short temperature = 0; //モーターの温度(m2006では温度は読めない)

    CalPID speed_pid;
    CalPID angle_omega_pid;
    Robomaster motor;

    RobomasM(motor_type type_, Controll_mode mode_);

    void speedControll(); //モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存。割り込みで回す。
    void setTargetSpeed(float targetspeed_); //角速度目標値設定
    void saveOmega();                        //角速度保存
    void displayData(); //保存したデータをシリアルプロッタに表示
    void angleControll(); //モーターを目標角速度で動かそうとし、一定の間隔で角度を配列に保存。割り込みで回す。
    void setTargetAngle(float targetangle_); //角度目標値設定
    void saveAngle();                        //角度保存
    void displayAngleData(); //保存したデータをシリアルプロッタに表示
    void reset();
    void ECcheck();                     //回転方向をLEDで表示。
    void test(float TargetSpeed_);      // PIDゲイン調整時等を想定。
    void angleTest(float TargetAngle_); // PIDゲイン調整時等を想定。

    void calcData(CANMessage* msg_); //CANから受信した情報を処理
    short getCurrent(); //目標出力電流を返す
    char getCurrent1(); //目標出力電流の上位1byteを返す（CAN送信用）
    char getCurrent2(); //目標出力電流の下位1byteを返す（CAN送信用）
};