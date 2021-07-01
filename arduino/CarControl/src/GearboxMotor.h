#pragma once
#include "Motor.h"

class GearboxMotor {
public:
    enum Position
    {
        up = 1,
        middle = 0,
        down = -1,
    };
    
    enum State{
        moving = 1,
        fine_tuning = 2,
        idle = 0
    };

    Position target_position = Position::middle;

    GearboxMotor(Motor motor, int l_up, int l_middle, int l_down):
        motor(motor),
        lim_up(l_up),
        lim_down(l_down),
        lim_middle(l_middle)
    {
        pinMode(l_up, INPUT_PULLUP);
        pinMode(l_down, INPUT_PULLUP);
        pinMode(l_middle, INPUT_PULLUP);
    }

    Position get_position(){
        return current_position;
    }

    int getpos()
    {
        int pos = 0;
        if (digitalRead(lim_up) == HIGH) {
            pos += 1;
        }
        else if (digitalRead(lim_middle) == LOW) {
            pos += 10;
        }
        else if (digitalRead(lim_down) == HIGH) {
            pos += 100;
        }
        return pos;
    }

    void update(){
        Position position;
        if (digitalRead(lim_up) == HIGH) {
            position = Position::up;
        }
        else if (digitalRead(lim_middle) == LOW) {
            position = Position::middle;
        }
        else if (digitalRead(lim_down) == HIGH) {
            position = Position::down;
        }
        // ожидаем новое целевое положение мотора
        // как только целевое положение изменено
        // начинаем двигать мотор в нужную сторону 
        if (state == State::idle) {
            //Serial.print("ready ");
            //Serial.print(current_position);
            //Serial.print(" ");
            //Serial.println(target_position);
            if (current_position != target_position) {
                state = State::moving;
                if (current_position < target_position) {
                    motor.run(255, Motor::Direction::forward);
                }
                else{
                    motor.run(255, Motor::Direction::backward);
                }
            }
        }
        // мотор движется в сторону целевого положения
        // до тех пор, пока оно не будет достигнуто
        else if (state == State::moving) {
            //Serial.println("moving");
            if (position == target_position) {
                state = State::fine_tuning;
                position_switch_time = millis() + 150;
            }
        }
        // мотор движется еще некоторое время для доводки
        else if (state == State::fine_tuning) {
            //Serial.println("ft");
            if (position_switch_time < millis()) {
                state = State::idle;
                current_position = position;
                motor.run(0, Motor::Direction::neutral);
            }
        }
        motor.update();
    }

private:
    State state = State::idle;
    Motor motor;
    int lim_up, lim_down, lim_middle;
    unsigned long position_switch_time;
    Position current_position = Position::middle;
};
