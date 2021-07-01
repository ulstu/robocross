#pragma once
#include "GearboxMotor.h"

class Gearbox {
public:

    //  -1 0 1 x
    //-1 1 3 5
    // 0 0-0-0
    // 1 2 4 6
    // y
    GearboxMotor::Position target_x, target_y;

    enum State {
        ready = 0, //ожидание (коробка находится в одной из скоростей и не переключает скорость)
        run_x = 1, //движение x
        run_y = 2 //движение y
    };

    State state = State::ready;
    GearboxMotor motor_x, motor_y;
    unsigned long finish_time;
    const int finish_delay = 200;
    Gearbox(GearboxMotor x, GearboxMotor y) :motor_x(x), motor_y(y) {
        target_x = GearboxMotor::Position::middle;
        target_y = GearboxMotor::Position::middle;
    }

    void set_gear(int target) {
    	//Serial.println(target);

        if (state == State::ready) {
            if (target == 0) {
                target_y = GearboxMotor::Position::middle;
                target_x = motor_x.get_position();
            }
            else if (target == 1) {
                target_x = GearboxMotor::Position::down;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 2) {
                target_x = GearboxMotor::Position::down;
                target_y = GearboxMotor::Position::up;
            }
            else if (target == 3) {
                target_x = GearboxMotor::Position::middle;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 4) {
                target_x = GearboxMotor::Position::middle;
                target_y = GearboxMotor::Position::up;
            }
            else if (target == 5) {
                target_x = GearboxMotor::Position::up;
                target_y = GearboxMotor::Position::down;
            }
            else if (target == 6) {
                target_x = GearboxMotor::Position::up;
                target_y = GearboxMotor::Position::up;
            }
        }
    }

    void update() {
    	//Serial.println(".");
        if (state == State::ready) {
        	//Serial.println("ready");
            //не совпадает х, у не в нейтралке
            //переводим на нейтралку
            if (motor_x.get_position() != target_x && motor_y.get_position() != GearboxMotor::Position::middle) {
                motor_y.target_position = GearboxMotor::Position::middle;
                state = State::run_y;
            }
            //не совпадает х, у в нейтралке
            //переключаем по х
            else if (motor_x.get_position() != target_x && motor_y.get_position() == GearboxMotor::Position::middle) {
                motor_x.target_position = target_x;
                state = State::run_x;
            }
            //совпадает х, не совпадает у
            //переключаем по у
            else if (motor_x.get_position() == target_x && motor_y.get_position() != target_y) {
                motor_y.target_position = target_y;
                state = State::run_y;
            }
        }

        //совпадают обе оси
        if (state == State::run_y && motor_y.get_position() == motor_y.target_position) {
        	state = State::ready;
        }
        else if (state == State::run_x && motor_x.get_position() == motor_x.target_position) {
        	state = State::ready;
        }
        

        /*Serial.print("x ");
        Serial.print(motor_x.get_position());
        Serial.print(" ");
        Serial.println(motor_x.target_position);
        Serial.println(motor_x.getpos());
        Serial.print("y ");
        Serial.print(motor_y.get_position());
        Serial.print(" ");
        Serial.println(motor_y.target_position);
        Serial.println(motor_y.getpos());*/

        motor_x.update();
        motor_y.update();
    }
};
