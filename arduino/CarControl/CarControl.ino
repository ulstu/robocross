#include "src/eur2.h"
#include "src/gearbox.h"
#include "src/Engine.h"
#include "src/GearboxMotor.h"
#include "src/PedalMotor.h"
#include "src/RC_receiver.h"

Motor motor_y(8, 28, true);
Motor motor_x(9, 29, true);

int action;
int motor_state = 0;
//ось y: верх 49, низ 48, средний инверт 45
//ось x: лево 47, право 46, средний инверт 44
GearboxMotor motor_gearbox_x(motor_x, 47, 44, 46);
GearboxMotor motor_gearbox_y(motor_y, 49, 45, 48);
Gearbox gearbox = Gearbox(motor_gearbox_x, motor_gearbox_y);
int gear_num = 0;


Wheel wheel; // руль
Engine engine(22, 23, 24);

//48 - 53 концевики
//пин концевики(2) скорость нажать отжать
PedalMotor clutch_motor(11, 10, 270, 31, 0, NULL, true);
PedalMotor brake_motor(10, 120, 260, 30, 1, 33, false);

void setup() {
    pinMode(43, INPUT_PULLUP);
    Serial.begin(115200);

    wheel.init(); // Инициализация руля
    
    int pins[] = { 2, 3, 18, 19, 20, 21 };
    RC_receiver::Setup(pins);
}

void loop() {
    if (RC_receiver::Available()) { //кнопка нажата
    //if (RC_receiver::Available() && digitalRead(43) != HIGH) { //кнопка нажата
        Serial.println("work");
        
        wheel.setPWM(int(((RC_receiver::eur_value - 994) / 1000.0) * 195 + 30));  // Установка напряжения на руле
        
        //int clutchPosition = (RC_receiver::clutch_value - 1490) / 2;
        /*if (clutchPosition < 0){
            clutchPosition = 0;                                                   // Установка положения на актуатор сцеления
        }
        clutch_motor.set(260 - clutchPosition);
        //Serial.println(260 - clutchPosition);*/

        int clutchPosition = RC_receiver::clutch_value;
        if (clutchPosition > 1850) {
            clutch_motor.set(0);
        }
        else if (clutchPosition > 1600) {
            clutch_motor.set(clutch_motor.getPosition());
        }
        else {
            clutch_motor.set(260);
        }
         
        //int brakePosition = (RC_receiver::brake_value - 1510) / 1.32;
        int brakePosition = (RC_receiver::clutch_value - 1450) / 2;
        if (brakePosition > 0){
            brakePosition = 0;                                                   // Установка положения на актуатор тормоза
        }
        brake_motor.set(abs(brakePosition));
        //Serial.println(abs(brakePosition));

        if (RC_receiver::gear_value > 1750) {                                   // Управление кпп+
            gear_num = 6;
        }
        else if (RC_receiver::gear_value > 1250) {
            gear_num = 0;
        }
        else {
            gear_num = 1;
        }

        Serial.println(motor_state);
        if (RC_receiver::egn_value < 1500) {
            if (motor_state == 1) {
                Serial.println("Shutdown engine");
                engine.stop();
                motor_state = 0;
            }
        }
        else {
            if (motor_state == 0) {
                Serial.println("launch engine");
                engine.start();
                motor_state = 1;
            }
        }
    }
    else {
        Serial.println("STOP");
        gear_num = 0;
        clutch_motor.set(260);
        brake_motor.set(260);
        engine.stop();
        motor_state = 0;
    }

    action = 0;
    /*if (Serial.available() > 0) {         // Управлеение по COM
        action = Serial.parseInt();
        if (action == 69) {            // Калибровка
            wheel.calibrate();
            delay(50000);
            Serial.println("Success calibration");
        }
    }*/

    Serial.println(RC_receiver::clutch_value);
    
    clutch_motor.update();
    brake_motor.update();
    gearbox.set_gear(gear_num);
    gearbox.update();
    engine.update();
    delay(50);
}
