#include "FlySkyIBus.h"
#include "src/eur2.h"
#include "src/gearbox.h"
#include "src/Engine.h"
#include "src/GearboxMotor.h"
#include "src/PedalMotor.h"

Motor motor_y(8, 28, true);
Motor motor_x(9, 29, true);

int action;
int counter = 0;
int motor_state = 0;
//ось y: верх 49, низ 48, средний инверт 45
//ось x: лево 47, право 46, средний инверт 44
GearboxMotor motor_gearbox_x(motor_x, 47, 44, 46);
GearboxMotor motor_gearbox_y(motor_y, 49, 45, 48);
Gearbox gearbox = Gearbox(motor_gearbox_x, motor_gearbox_y);
int gear_num = 0;
byte first = 0x00;
byte second = 0x00;
byte third = 0x00;

float last_angle = 0, new_angle = 0;

Wheel wheel; // руль
Engine engine(22, 23, 24);

//48 - 53 концевики
//пин концевики(2) скорость нажать отжать
PedalMotor clutch_motor(11, 10, 270, 31, 0, NULL, true);
PedalMotor brake_motor(10, 120, 260, 30, 1, 33, false);

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(1);
    Serial2.begin(115200);
    gearbox.init();
    delay(500);
    TCCR2B = TCCR2B & B11111000 | B00000001;
    TCCR1B = TCCR1B & B11111000 | B00000001;
    pinMode(32, INPUT_PULLUP);
    pinMode(26, OUTPUT);
    pinMode(13, OUTPUT);
    digitalWrite(26, LOW);
    wheel.init(); // Инициализация руля
    IBus.begin(Serial1);
}

void loop() {
    IBus.loop();
    if (IBus.available() && digitalRead(32) != HIGH) { //кнопка нажата
      if (IBus.readChannel(8) < 1900) {
        Serial.println("work");

        while (Serial2.available() > 0) {
          byte t = Serial2.read();
        }
        
        wheel.setPWM(127 + (int(IBus.readChannel(0)) - 1500) / 4000.0 * (int(IBus.readChannel(6)) - 1000));  // Установка напряжения на руле
        int clutchPosition = IBus.readChannel(1);
        if (clutchPosition > 1850) {
            clutch_motor.set(0);
        }
        else if (clutchPosition > 1600) {
            clutch_motor.set(clutch_motor.getPosition());
        }
        else {
            clutch_motor.set(260);
        }
         
        int brakePosition = (clutchPosition - 1500) / 6;
        if (brakePosition > 0){
            brakePosition = 0;                                                   // Установка положения на актуатор тормоза
        }
        brake_motor.set(abs(brakePosition) + 120);

        if (IBus.readChannel(4) < 900) {                                   // Управление кпп+
            gear_num = 0;
        }
        else if (IBus.readChannel(4) > 1750) {                                   // Управление кпп+
            gear_num = 6;
        }
        else if (IBus.readChannel(4) > 1250) {
            gear_num = 0;
        }
        else {
            gear_num = 1;
        }

        if (IBus.readChannel(5) < 1500) {
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

        if (IBus.readChannel(9) < 1500){
            digitalWrite(26, HIGH);
        }
        else {
            digitalWrite(26, LOW);
        }
      }
      else {
        //Serial.println("AUTO");
        
        if (IBus.readChannel(5) < 1500) {
            if (motor_state == 1) {
                engine.stop();
                motor_state = 0;
            }
        }
        else {
            if (motor_state == 0) {
                engine.start();
                motor_state = 1;
            }
        }
        
        if (IBus.readChannel(9) < 1500){
            digitalWrite(26, HIGH);
        }
        else {
            digitalWrite(26, LOW);
        }

        if (IBus.readChannel(4) < 900) {                                   // Управление кпп+
            gear_num = 0;
        }
        else if (IBus.readChannel(4) > 1750) {                                   // Управление кпп+
            gear_num = 6;
        }
        else if (IBus.readChannel(4) > 1250) {
            gear_num = 0;
        }
        else {
            gear_num = 1;
        }
        Serial.println("1");

        int clutchPosition = IBus.readChannel(1);
        if (clutchPosition > 1850) {
            clutch_motor.set(0);
        }
        else if (clutchPosition > 1600) {
            clutch_motor.set(clutch_motor.getPosition());
        }
        else {
            clutch_motor.set(260);
        }
         
        int brakePosition = (clutchPosition - 1500) / 6;
        if (brakePosition > 0){
            brakePosition = 0;                                                   // Установка положения на актуатор тормоза
        }
        brake_motor.set(abs(brakePosition) + 120);

        while (Serial2.available() > 0) {
          byte t = Serial2.read();
            if (t / 128 == 1) {
              first = t;
              second = 0x00;
              third = 0x00;
              counter = 0;
              
            }
            else if (counter == 0) {
              second = t;
              counter++;
            }
            else if (counter == 1) {
              third = t;
              byte cmd = first << 1;
              int result;
              cmd = cmd >> 3;
              result = third;
              result += second << 7;
              result += first << 14;
              if (cmd == 16) {
                new_angle = result;
                digitalWrite(13, 1);
                delay(15);
                digitalWrite(13, 1);
              }
              counter++;
            }
         }
         if (last_angle < new_angle) {
          last_angle += 0.1;
         }
         else if (last_angle > new_angle) {
          last_angle -= 0.1;
         }
         wheel.setAngleNow(int(last_angle));
       }
    }
    else {
        Serial.println("STOP");
        gear_num = 0;
        clutch_motor.set(260);
        brake_motor.set(260);
        engine.stop();
        motor_state = 0;
        wheel.setPWM(127);
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

    clutch_motor.update();
    brake_motor.update();
    gearbox.set_gear(gear_num);
    gearbox.update();
    engine.update();
    //delay(50);
}
