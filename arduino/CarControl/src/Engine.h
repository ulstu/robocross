enum MotorState{
    shutdown,
    launching,
    work
};

class Engine {
private:
    // starter pins
    int p1, p2, p3;
  
    MotorState state;
    unsigned long start_time;
public:

    Engine(int p1, int p2, int p3) :p1(p1), p2(p2), p3(p3) {
        pinMode(p1, OUTPUT);
        pinMode(p2, OUTPUT);
        pinMode(p3, OUTPUT);
    }

    void update() {
        if (state == MotorState::shutdown) {
            Serial.println("slutdown");
            digitalWrite(p1, HIGH);
            digitalWrite(p2, HIGH);
            digitalWrite(p3, HIGH);
        }
        if (state == MotorState::launching) {
            Serial.println("launching");
            int cureent_time = millis();
            if (cureent_time - start_time < 1000) {
                digitalWrite(p1, LOW);
                digitalWrite(p2, LOW);
            }
            if (cureent_time - start_time >= 1000) {
                digitalWrite(p3, LOW);
            }
            if (cureent_time - start_time >= 2000) {
                digitalWrite(p1, HIGH);
                state = MotorState::work;
            }
        }
        if (state == MotorState::work) {
            Serial.println("bodraching");
        }
    }

    void start() {
        start_time = millis();
        state = MotorState::launching;
    }

    void stop() {
        state = MotorState::shutdown;
    }
};
