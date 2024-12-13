#include "mbed.h"
#include "RobomasM.hpp"

#define TARGET_CAN_ID 0x200   //コントロール識別子（モーター１～４）
#define RECIEVE_CAN_ID1 0x201   //フィードバック識別子（モーター１,m2006）
#define CAN_Hz_RMM 1000000 //CANに使用するクロック周波数[Hz] CAN通信相手と共通させる

CAN can_rmm(PA_11, PA_12, CAN_Hz_RMM); //CAN_RD, CAN_TD, CAN_Hzの順

BufferedSerial pc(USBTX, USBRX, 115200);

RobomasM rmm(M2006, mode3_gear_limitless);

DigitalOut led1(LED1);

Ticker tic_motor;

void setup();

int main()
{
    setup();

    float target_speed = 3.14;
    rmm.setTargetSpeed(target_speed);

    while(true){
        printf("target_speed:%f\n", target_speed);
        printf("current:%d\n\n",rmm.getCurrent());
    }

}

void motorCallback(){
    rmm.speedControll();

    char can_data[2] = {}; //送りたいデータを入れる箱
    can_data[0] = rmm.getCurrent1();
    can_data[1] = rmm.getCurrent2();

    CANMessage msg(TARGET_CAN_ID, can_data, 2); //CANプロトコルに変換
    can_rmm.write(msg); //CANデータを送信
}

void CAN_recieve(){
    CANMessage msg;
    if(can_rmm.read(msg)){  
        if(msg.id == RECIEVE_CAN_ID1){
            rmm.calcData(&msg);
        }
    }
}

void motor_init() {
    rmm.speed_pid.setParameter(600, 150, 0);
    rmm.angle_omega_pid.setParameter(0, 0, 0);
    rmm.motor.setPowerMax(2500);
    rmm.motor.stop();
}

void setup(){
    can_rmm.attach(CAN_recieve, CAN::RxIrq);

    motor_init();

    tic_motor.attach(&motorCallback,1ms); //割り込みタイマーをオンにする
    thread_sleep_for(50);

}
