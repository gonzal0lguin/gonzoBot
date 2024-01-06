#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>

#define PWMA  21
#define AIN2  19
#define AIN1  20
// #define STDBY 
#define BIN2  17
#define BIN1  18
#define PWMB  16

#define CHA_M1 15
#define CHB_M1 14
#define CHA_M2 13
#define CHB_M2 12

class MotorDriver {
public:
    // MotorDriver(uint8_t pwm_pin, uint8_t dir_pin, uint8_t encoderA_pin, uint8_t encoderB_pin);
    explicit MotorDriver(bool debug);

    void set_dir(int motor, bool dir);

    //void update_encoders(uint64_t current_time);
    void update_encoders(ros::NodeHandle &nh_);
    void enable_motors();
    void disable_motors();
    void motor_left_isr();
    void motor_right_isr();
    void reset_encoders();

    int get_encoder_resolution();

    sensor_msgs::JointState get_joint_states(ros::NodeHandle &nh_);

    void set_speed_pwm(int mot, int pwm);

    double ticks_to_angle(const int64_t &ticks) const;
    int mps_to_pwm(float mps);

    int64_t pos_left = 0;
    int64_t pos_right = 0;
    int64_t last_pos_left = 0;
    int64_t last_pos_right = 0;
private:

    volatile int64_t _pos_left = 0;
    volatile int64_t _pos_right = 0;
    double speed_left_mps = 0.0;
    double speed_right_mps = 0.0;

    //uint64_t _last_update_encoder_ms = 0;
    ros::Time _last_update_encoder;
    double encoder_update_rate = 100; // Hz

    bool _dir = false;

    uint8_t _pwm_pin;
    uint8_t _dir_pin; 
    uint8_t _encoder_pin;

    int _min_speed_duty = 1024;
    int _max_speed_duty = 0;
    bool _debug = false;

    double _wheel_radius = 0.0325;
    int _encoder_resolution = 334 * 4;   // ticks x revolution, TODO: revisar realmente cuantos son

    // ROS
    // ros::NodeHandle nh_;
    ros::Time prev_update_time_;
    sensor_msgs::JointState joint_states_;
};


//// ------ definitions -------

MotorDriver::MotorDriver(bool debug)
    : _debug(debug)
{
    pinMode(AIN1,   OUTPUT);
    pinMode(AIN2,   OUTPUT);
    pinMode(BIN1,   OUTPUT);
    pinMode(BIN1,   OUTPUT);
    pinMode(PWMA,   OUTPUT);
    pinMode(PWMB,   OUTPUT);
    // pinMode(STDBY,  OUTPUT);
    pinMode(CHA_M1, INPUT_PULLUP);
    pinMode(CHB_M1, INPUT_PULLUP);
    pinMode(CHA_M2, INPUT_PULLUP);
    pinMode(CHB_M2, INPUT_PULLUP);

    joint_states_.position = (float*)malloc(sizeof(float) * 2);
    joint_states_.position_length = 2;
    joint_states_.velocity = (float*)malloc(sizeof(float) * 2);
    joint_states_.velocity_length = 2;
}

// void MotorDriver::enable_motors()
// {
//     digitalWrite(STDBY, HIGH);
// }

// void MotorDriver::disable_motors()
// {
//     digitalWrite(STDBY, LOW);
// }

void MotorDriver::set_dir(int motor, bool dir)
{
    uint8_t dir1 = motor==0 ? BIN1 : AIN1; 
    uint8_t dir2 = motor==0 ? BIN2 : AIN2; 

    if (dir) 
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
    }
    else 
    {
        digitalWrite(dir2, LOW);
        digitalWrite(dir1, HIGH);
    }
}

void MotorDriver::set_speed_pwm(int mot, int pwm)
{
    uint8_t mot_pwm_pin = mot == 0 ? PWMA : PWMB;
    int dir = pwm >= 0 ? 0 : 1;
    this->set_dir(mot, dir);
    analogWrite(mot_pwm_pin, pwm);
}

void MotorDriver::update_encoders(ros::NodeHandle &nh_)
{
    // TODO: ROS time instead of millis()?
    ros::Duration update_dt = nh_.now() - _last_update_encoder;
    //if (current_time - _last_update_encoder_ms >= 10)
    if (update_dt.toSec() >= 1 / encoder_update_rate)
    {
        noInterrupts(); // Disable interrupts while reading counts
        pos_left  = _pos_left; // read both encoder counts
        pos_right = _pos_right;
        interrupts();

        // Calculate motor velocities and positions here
        if (_debug) { // TODO: this should use logInfo instead
            //Serial.print("L ticks: ");
            //Serial.print(pos_left);
            //Serial.print("R ticks: ");
            //Serial.println(pos_right);
            nh_.loginfo("aca va un mensaje de de debug");
        }

        //_last_update_encoder_ms = current_time;
        _last_update_encoder = nh_.now();
    }
}

void MotorDriver::reset_encoders()
{
    noInterrupts();
    _pos_left  = 0;
    _pos_right = 0;
    pos_left   = 0;
    pos_right  = 0;
    last_pos_left = 0;
    last_pos_right = 0;
    joint_states_.position[0] = 0;
    joint_states_.position[1] = 0;
    interrupts();
}

void MotorDriver::motor_left_isr()
{
    static uint8_t last_left_cha_state = LOW;
    uint8_t left_cha_state = digitalRead(CHA_M1);
    uint8_t left_chb_state = digitalRead(CHB_M1);

    if (left_cha_state != last_left_cha_state)
    {
        if (left_chb_state == left_cha_state)
        {
            _pos_left++;
        }
        else
        {
            _pos_left--;
        }
    }

    last_left_cha_state = left_cha_state;
}


void MotorDriver::motor_right_isr()
{
    static uint8_t last_right_cha_state = LOW;
    uint8_t right_cha_state = digitalRead(CHA_M2);
    uint8_t right_chb_state = digitalRead(CHB_M2);

    if (right_cha_state != last_right_cha_state)
    {
        if (right_chb_state == right_cha_state)
        {
            _pos_right++;
        }
        else
        {
            _pos_right--;
        }
    }

    last_right_cha_state = right_cha_state;
}

sensor_msgs::JointState MotorDriver::get_joint_states(ros::NodeHandle &nh_)
{
    ros::Time current_time = nh_.now();
    ros::Duration dt = current_time - prev_update_time_;
    auto dts = (float )dt.toSec();

    //calculate wheel's speed (RPM)
    int64_t d_ticks_left  = pos_left - last_pos_left;
    int64_t d_ticks_right = pos_right - last_pos_right;
    double d_angle_left  = ticks_to_angle(d_ticks_left);
    double d_angle_right = ticks_to_angle(d_ticks_right);

    joint_states_.position[0] += d_angle_left;
    joint_states_.position[1] += d_angle_right;

    joint_states_.velocity[0] = d_angle_left / dts;
    joint_states_.velocity[1] = d_angle_right / dts;

    prev_update_time_ = current_time;
    last_pos_left  = pos_left;
    last_pos_right = pos_right;

    return joint_states_;
}

int MotorDriver::get_encoder_resolution()
{
    return _encoder_resolution;
}

double MotorDriver::ticks_to_angle(const int64_t &ticks) const {
    double angle = (double )ticks * (2.0 * M_PI / _encoder_resolution);
    return angle;
}

#endif // MOTOR_DRIVER_H