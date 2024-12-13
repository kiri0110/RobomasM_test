#include "RobomasM.hpp"

RobomasM::RobomasM(motor_type type_, Controll_mode mode_)
    : speed_pid(CalPID(0.0000, 0.0000, 0.0000, DELTA_T, DUTY_MAX)),
      angle_omega_pid(CalPID(0.0000, 0.0000, 0.0000, DELTA_T, OMEGA_MAX)),
      motor(Robomaster(type_, mode_, &current, &angle, &speed, &torque, &temperature, speed_pid, angle_omega_pid)) {}


// //各種調整は↓で行う
// void RobomasM::init(int unitNum_) {
//     printf("RobomasM init...\r\n");

//     switch (unitNum_) {
//         case 0:
//             speed_pid.setParameter(100.0, 0.0, 0.1);
//             angle_omega_pid.setParameter(0.0, 0.0, 0.0);
//             break;
//         case 1:
//             speed_pid.setParameter(100.0, 0.0, 0.1);
//             angle_omega_pid.setParameter(0.0, 0.0, 0.0);
//             break;
//         case 2:
//             speed_pid.setParameter(100.0, 0.0, 0.1);
//             angle_omega_pid.setParameter(0.0, 0.0, 0.0);
//             break;
//         case 3:
//             speed_pid.setParameter(100.0, 0.0, 0.1);
//             angle_omega_pid.setParameter(0.0, 0.0, 0.0);
//             break;
//     }

//     motor.setAccelMax(200);

//     // //速度制御のPID 前3つが係数なのでこれを調整 P→D→Iの順で調整がオススメ。
//     // speed_pid.setParameter(100.0, 0.0, 0.1);

//     // //角度制御のPID
//     // angle_omega_pid.setParameter(0.0, 0.0, 0.0);

//     // //モーターの加速度の上限
//     // motor.setAccelMax(200);

//     // // ギア比 タイヤ:エンコーダ＝１:rとしたときのrの値
//     // motor.setGearRatio((float) 7/4);

// }

//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存
void RobomasM::speedControll() {
    motor.Sc(target_speed);
    omega_save_count++;
    if (omega_save_count > SAVE_COUNT_THRESHOLD) {
        saveOmega();
        omega_save_count = 0;
    }
}

void RobomasM::setTargetSpeed(float targetspeed_) { //こんな値の渡し方でいいのか...
    target_speed = targetspeed_;
}

void RobomasM::saveOmega() {
    if (omega_count < NUM_DATA) {
        omega_saved[omega_count] = (short)motor.getOmega() * 100;
        omega_count++;
        // printf("%d, %f\r\n", omega_count, motor.getOmega());
    }
}

void RobomasM::displayData() //保存したデータをシリアルプロッタに表示
{
    printf("omega\r\n");
    for (int i = 0; i < omega_count; i++) {
        printf("%f,\r\n", omega_saved[i] / 100.0); //角速度を記録
        thread_sleep_for(5); // printf重いのでマイコンが落ちないように
    }
    omega_count = 0;
}

//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存
void RobomasM::angleControll() {
    motor.Ac(target_angle);
    angle_save_count++;
    if (angle_save_count > SAVE_COUNT_THRESHOLD) {
        saveAngle();
        angle_save_count = 0;
    }
}

void RobomasM::setTargetAngle(float targetangle_) { //こんな値の渡し方でいいのか...
    target_angle = targetangle_;
}

void RobomasM::saveAngle() {
    if (angle_count < NUM_DATA) {
        angle_saved[angle_count] = (short)motor.getAngle() * 100;
        angle_count++;
    }
}

void RobomasM::displayAngleData() //保存したデータをシリアルプロッタに表示
{
    printf("angle\r\n");
    for (int i = 0; i < angle_count; i++) {
        printf("%f,\r\n", angle_saved[i] / 100.0); //角速度を記録
        thread_sleep_for(5); // printf重いのでマイコンが落ちないように
    }
    angle_count = 0;
}

void RobomasM::reset() {
    motor.reset();
}


void RobomasM::test(float TargetSpeed_) {
    //事前にプログラムをマイコンに書き込んでおく
    //緊急停止スイッチで切った状態で電源をつなぎ、マイコンをリセットする
    printf("Motor test...\r\n");
    //このタイミングで緊急停止スイッチを解除する
    thread_sleep_for(3000);
    // STARTが書かれた時点で開始する
    printf("\r\nSTART\r\n");
    omega_count = 0;
    setTargetSpeed(TargetSpeed_);

    thread_sleep_for(3000); //モーターが割り込みでまわり3秒分のデータを取る
    setTargetSpeed(0);
    thread_sleep_for(3000);
    motor.stop(); //モーターを止める
    printf("STOP\r\n");
    displayData(); //角速度のデータが表示される
    thread_sleep_for(3000);

    printf("\r\nSTART\r\n");
    omega_count = 0;
    setTargetSpeed(-TargetSpeed_);

    thread_sleep_for(3000); //モーターが割り込みでまわり3秒分のデータを取る
    setTargetSpeed(0);
    thread_sleep_for(3000);
    motor.stop(); //モーターを止める
    printf("STOP\n");
    displayData();
}

void RobomasM::angleTest(float TargetAngle_) {
    //事前にプログラムをマイコンに書き込んでおく
    //緊急停止スイッチで切った状態で電源をつなぎ、マイコンをリセットする
    printf("Motor angle test...\r\n");
    //このタイミングで緊急停止スイッチを解除する
    thread_sleep_for(3000);
    //STARTが書かれた時点で開始する
    printf("\r\nSTART\r\n");
    angle_count = 0;
    setTargetAngle(TargetAngle_);

    thread_sleep_for(3000); //モーターが割り込みでまわり3秒分のデータを取る
    setTargetAngle(0);
    thread_sleep_for(3000);
    motor.stop(); //モーターを止める
    printf("STOP\r\n");
    displayAngleData(); //角速度のデータが表示される
    thread_sleep_for(3000);

    printf("\r\nSTART\r\n");
    angle_count = 0;
    setTargetAngle(-TargetAngle_);

    thread_sleep_for(3000); //モーターが割り込みでまわり3秒分のデータを取る
    setTargetAngle(0);
    thread_sleep_for(3000);
    motor.stop(); //モーターを止める
    printf("STOP\n");
    displayAngleData();
}


void RobomasM::calcData(CANMessage* msg_){
    //CANデータの読み取り <-128~127>に変換
    angle = (short)(msg_->data[0] << 8 | msg_->data[1]);
    speed = (short)(msg_->data[2] << 8 | msg_->data[3]);
    torque = (short)(msg_->data[4] << 8 | msg_->data[5]);
    temperature = (short)msg_->data[6];
}

short RobomasM::getCurrent(){
    return current;
}
char RobomasM::getCurrent1(){
    return current >> 8;
}
char RobomasM::getCurrent2(){
    return current & 255;
}