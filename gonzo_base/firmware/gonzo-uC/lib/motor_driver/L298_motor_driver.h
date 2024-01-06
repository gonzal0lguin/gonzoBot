//
// Created by Gonzalo Olguin on 07-08-23.
//

#ifndef LIBRARIES_L298_MOTOR_DRIVER_H
#define LIBRARIES_L298_MOTOR_DRIVER_H
#include <math.h>
#include <Arduino.h>

class L298Motor
{
public:
    L298Motor(uint8_t IN_1_pin, uint8_t IN_2_pin, uint8_t IN_3_pin,
          uint8_t IN_4_pin, uint8_t EN_a_pin, uint8_t EN_b_pin);

    void set_dir(uint8_t motor_id, uint8_t dir);
    void move_pwm(uint8_t dir, uint16_t pwm);
    void set_speed_pwm(uint8_t motor, int pwm);
    void get_speed_rads();

private:
    uint8_t _IN_1_pin;
    uint8_t _IN_2_pin;
    uint8_t _IN_3_pin;
    uint8_t _IN_4_pin;
    uint8_t _EN_a_pin;
    uint8_t _EN_b_pin;

    uint8_t _dir = 0;
    int64_t _speed_pwm = 0;
};

// definitions

L298Motor::L298Motor(uint8_t IN_1_pin, uint8_t IN_2_pin, uint8_t IN_3_pin,
                  uint8_t IN_4_pin, uint8_t EN_a_pin, uint8_t EN_b_pin)
{
    _IN_1_pin = IN_1_pin;
    _IN_2_pin = IN_2_pin;
    _IN_3_pin = IN_3_pin;
    _IN_4_pin = IN_4_pin;
    _EN_a_pin = EN_a_pin;
    _EN_b_pin = EN_b_pin;

    pinMode(_IN_1_pin, OUTPUT);
    pinMode(_IN_2_pin, OUTPUT);
    pinMode(_IN_3_pin, OUTPUT);
    pinMode(_IN_4_pin, OUTPUT);
    pinMode(_EN_a_pin, OUTPUT);
    pinMode(_EN_b_pin, OUTPUT);
}

void L298Motor::set_dir(uint8_t motor_id, uint8_t dir)
{
    // por ahora solo trabajamos con un motor
    // dir 1
    if (dir) {
        digitalWrite(_IN_3_pin, HIGH);
        digitalWrite(_IN_4_pin, LOW);
    }

    else {
        digitalWrite(_IN_3_pin, LOW);
        digitalWrite(_IN_4_pin, HIGH);
    }
}

void L298Motor::set_speed_pwm(uint8_t motor, int pwm){
    int dir = pwm >= 0 ? 0 : 1;
    this->set_dir(motor, dir);
    analogWrite(_EN_b_pin, abs(pwm));
}

#endif //LIBRARIES_L298_MOTOR_DRIVER_H
