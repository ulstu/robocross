#pragma once

class PedalMotor {

private:
    int pwmPin, dirPin, analogPin, limitSwitch;
    int min, max;
    int targetPosition;
    bool inversed;

public:

    PedalMotor(int pwmPin, int min, int max, int dirPin, int analogPin, int limitSwitch, bool inversed) {
        this->pwmPin      = pwmPin;
        this->dirPin      = dirPin;
        this->analogPin   = analogPin;
        this->min         = min;
        this->max         = max;
        this->limitSwitch = limitSwitch;
        this->inversed    = inversed;
        targetPosition    = 0;
        pinMode(pwmPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        if (limitSwitch != NULL) {
        	pinMode(limitSwitch, INPUT_PULLUP);
        }
    }

    void set(int position) {
    	targetPosition = position * (max / 255.0);
    	if (targetPosition > max) {
    		targetPosition = max;
    	}
    	else if (targetPosition < min) {
    		targetPosition = min;
    	}
    }

    void update() {
        int currentPosition = analogRead(analogPin);
        int speed = 5 * abs(currentPosition - targetPosition);
        if (speed > 255) {
        	speed = 255;
        }
        if(currentPosition > targetPosition) {
    		digitalWrite(dirPin, inversed? 0 : 1);
        }
        else if (currentPosition < targetPosition) {
        	digitalWrite(dirPin, inversed? 1 : 0);
        }

        if (limitSwitch != NULL) {
        	if (!digitalRead(limitSwitch) && currentPosition < targetPosition) {
        		speed = 0;
        		Serial.print("break stuck!");
        		Serial.println(limitSwitch);
        	}
        }
        else if (currentPosition < targetPosition) {
        	if (currentPosition > (0.8 * targetPosition)) speed = 255;
        	else if (currentPosition > (0.4 * targetPosition)) speed = 140;
        	else speed = 255;
        }
        //Serial.println(speed);
        //Serial.println(currentPosition);
        analogWrite(pwmPin, speed);
    }

    int getPosition() {
    	return analogRead(analogPin);
    }

    /*
    void push(int speed = 100) {
        speed *= 2.5;
        this->speed = speed;
        direction = (inverse_direction ? 0 : 1);
    }

    void release(int speed = 100) {
        speed *= 2.5;
        this->speed = speed;
        direction = (inverse_direction ? 1 : 0);
    }

    void stop() {
        this->speed = 0;
    }
*/
};
