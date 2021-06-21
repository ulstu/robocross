#include "src/eur2.h"
#include "src/gearbox.h"
//#include "src/Engine.h"
#include "src/GearboxMotor.h"
#include "src/PedalMotor.h"
#include "src/RC_receiver.h"

Motor motor_x(14, 15);
Motor motor_y(16, 17);

int action;

GearboxMotor motor_gearbox_x(motor_x, 34, 32, 33);
GearboxMotor motor_gearbox_y(motor_y, 36, 31, 35);
Gearbox gearbox = Gearbox(motor_gearbox_x, motor_gearbox_y);
int gear_num = 0;


Wheel wheel; // руль
//Engine engine(26, 25, 24);

//48 - 53 концевики
//пин концевики(2) скорость нажать отжать
PedalMotor clutch_motor(11, 10, 270, 31, 0, NULL, true);
PedalMotor brake_motor(10, 85, 260, 30, 1, 33, false);

void setup() {
    pinMode(43, INPUT_PULLUP);
    Serial.begin(115200);

    wheel.init(); // Инициализация руля
    
    int pins[] = { 2, 3, 18, 19, 20, 21 };
    RC_receiver::Setup(pins);
}

int state = 0;
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
        if (clutchPosition > 1600) {
            clutch_motor.set(0);
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
            gear_num = 1;
        }
        else if (RC_receiver::gear_value > 1250) {
            gear_num = 0;
        }
        else {
            gear_num = 6;
        }
        
        /*
        
        if (RC_receiver::brake_value < 1250) {
            brake_motor.push(50);
        }
        else if (RC_receiver::brake_value < 1750) {
            brake_motor.stop();
        }
        else {
            brake_motor.release(25);
        }

        if (RC_receiver::egn_value < 1500) {
            if (state == 1) {
                Serial.println("Shutdown engine");
                state = 0;
                engine.shutdown();
            }
        }
        else {
            if (state == 0) {
                Serial.println("launch engine");
                //eur.reboot();
                engine.launch();
                state = 1;
            }
        }
        */
    }
    else {
        Serial.println("STOP");
        /*
        gear_num = 0;
        engine.shutdown();
        brake_motor.push(100);
        clutch_motor.push(100);
        */
    }

    action = 0;
    if (Serial.available() > 0) {         // Управлеение по COM
        action = Serial.parseInt();
        if (action == 1) {            // Калибровка
            wheel.calibrate();
            delay(32000);
            Serial.println("Clutch calibration min");
            delay(15000);
            eeprom_update_word(0, clutch_motor.getPosition());
            Serial.println("Clutch calibration max");
            delay(15000);
            eeprom_update_word(2, clutch_motor.getPosition());
            Serial.println("Brake calibration min");
            delay(15000);
            eeprom_update_word(4, brake_motor.getPosition());
            Serial.println("Brake calibration max");
            delay(15000);
            eeprom_update_word(6, brake_motor.getPosition());
            Serial.println("Success calibration");
        }
    }
    clutch_motor.update();
    brake_motor.update();
    gearbox.set_gear(gear_num);
    //delay(200);
}
