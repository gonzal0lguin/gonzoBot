#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H


#include <Arduino.h>
#include <ros.h>
#include <WheelCmdStamped.h>
#include <BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <motor_driver.h>
#include <batteries.h>



class BaseController {
        public:
            explicit BaseController(MotorDriver *m_driver, Battery *bat);

            /*
                * Setup function: Initializes the node and advertises publishers and
                * subscribers. Allocates memory for the published messages.
                */
            void setup(ros::NodeHandle &nh_);

            /*
                * Init function zeros the encoders and gets the parameters
                * from the parameter server.
                */
            void init(ros::NodeHandle &nh_);

            /*
                * Reads the current joint states and then publishes the message.
                */
            void read(ros::NodeHandle &nh_);

            /*measured_battery_state_
                * Reads the current commanded message and writes it to the
                * motor driver.
                */
            void write();

            /**
             * Emergency brake
            */
            void stop();

            void check_battery_state(ros::Time stamp);

            // Subscriber callbacks
            void reset_encoders_cb(const std_msgs::Empty& reset_msg);
            void command_cb(const gonzo_msgs::WheelCmdStamped& cmd_msg);

            double control_rate = 20.0; // Hz
            bool blink = true;

        private:
            // Parameters
            float wheel_radius_         = 0.0;
            float max_linear_velocity_  = 0.0;
            float max_angular_velocity_ = 0.0;

            float wheel_cmd_left_  = .0;
            float wheel_cmd_right_ = .0;

            // Motor Driver
            MotorDriver* m_driver_;
            Battery* bat_;

            int motor_cmd_left_  = 0;
            int motor_cmd_right_ = 0;

            // ROS variables and handles
            //ros::NodeHandle nh_;

            // ROS messages
            sensor_msgs::JointState measured_joint_state_;
            //gonzo_msgs::EncodersStamped encoder_msg_;
            gonzo_msgs::BatteryState measured_battery_state_;

            // Subscribers
            ros::Subscriber<std_msgs::Empty, BaseController> reset_encoders_sub_;
            ros::Subscriber<gonzo_msgs::WheelCmdStamped, BaseController> cmd_vel_sub_;

            // Publishers
            ros::Publisher joint_state_pub_;
            // ros::Publisher encoder_ticks_pub_;

            ros::Publisher battery_state_pub_;
};


BaseController::BaseController(MotorDriver *m_driver, Battery *bat)
    : joint_state_pub_("measured_joint_states", &measured_joint_state_)
    , battery_state_pub_("battery_state", &measured_battery_state_)
    , reset_encoders_sub_("reset_encoders", &BaseController::reset_encoders_cb, this)
    , cmd_vel_sub_("wheel_cmd_vel", &BaseController::command_cb, this)
    //, encoder_ticks_pub_("/encoder_ticks", &encoder_msg_)
{
    m_driver_ = m_driver;
    bat_      = bat;
}

void BaseController::setup(ros::NodeHandle &nh_) {
    // nh_.initNode(); // this should go in the main.cpp

    measured_joint_state_.position = (float*)malloc(sizeof(float) * 2);
    measured_joint_state_.position_length = 2;
    measured_joint_state_.velocity = (float*)malloc(sizeof(float) * 2);
    measured_joint_state_.velocity_length = 2;
    measured_battery_state_.cell_voltage = (float*)malloc(sizeof(float) * 3);
    measured_battery_state_.cell_voltage_length = 3;


    nh_.advertise(joint_state_pub_);
    nh_.advertise(battery_state_pub_);
    //nh_.advertise(encoder_ticks_pub_);

    nh_.subscribe(reset_encoders_sub_);
    nh_.subscribe(cmd_vel_sub_);

    while (!nh_.connected())
    {
        nh_.spinOnce();
    }
}

void BaseController::init(ros::NodeHandle &nh_) {
    nh_.loginfo("Obtaining parameters from param server");
    //nh_.getParam("/gonzo/wheel_radius"   , &wheel_radius_);
    //nh_.getParam("/gonzo/max_linear_vel" , &max_linear_velocity_);
    //nh_.getParam("/gonzo/encoder_resolution" , &encoder_resolution_);
    //nh_.getParam("/gonzo/pid_base/P

    // Reset encoders
    std_msgs::Empty reset_msg;
    this->reset_encoders_cb(reset_msg);
    delay(1);

    max_angular_velocity_ = max_linear_velocity_ / wheel_radius_;

    // add another delay for initializing?
    delay(100);
    // m_driver_->enable_motors();
}

void BaseController::read(ros::NodeHandle &nh_)
{
    measured_joint_state_ = m_driver_->get_joint_states(nh_);
    joint_state_pub_.publish(&measured_joint_state_);
}

void BaseController::write()
{
    motor_cmd_left_  = (int )wheel_cmd_left_;  // Ideally use a PID here
    motor_cmd_right_ = (int )wheel_cmd_right_; // Ideally use a PID her

    // map the velocity and constrain it first
    m_driver_->set_speed_pwm(0, motor_cmd_left_);
    m_driver_->set_speed_pwm(1, motor_cmd_right_);
}

void BaseController::command_cb(const gonzo_msgs::WheelCmdStamped& cmd_msg){
    wheel_cmd_left_  = cmd_msg.wheel_cmd.angular_velocities.joint_velocity[0];
    wheel_cmd_right_ = cmd_msg.wheel_cmd.angular_velocities.joint_velocity[1];
    //String log = "received commands: left= " + String(wheel_cmd_left_) +
    //             "   right= " + String(wheel_cmd_right_);
    //nh_.loginfo(log.c_str());
}

void BaseController::stop()
{
    wheel_cmd_left_  = 0;
    wheel_cmd_right_ = 0;
}

void BaseController::reset_encoders_cb(const std_msgs::Empty& reset_msg) {
    m_driver_->reset_encoders();
    // this->nh_.loginfo("Resetting encoders");
}


void BaseController::check_battery_state(ros::Time stamp)
{
    bat_->update_cell_voltages();
    
    measured_battery_state_.header.stamp = stamp; 
    measured_battery_state_.battery_voltage = bat_->battery_voltage;
    measured_battery_state_.percentage = bat_->battery_percentage;
    measured_battery_state_.cell_voltage[0] = bat_->cell_voltage[0];
    measured_battery_state_.cell_voltage[1] = bat_->cell_voltage[1];
    measured_battery_state_.cell_voltage[2] = bat_->cell_voltage[2];

    battery_state_pub_.publish(&measured_battery_state_);
}

#endif // BASE_CONTROLLER_H