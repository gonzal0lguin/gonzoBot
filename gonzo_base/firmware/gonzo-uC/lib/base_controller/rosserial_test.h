//
// Created by Gonzalo Olguin on 28-07-23.
//

#ifndef LIBRARIES_ROSSERIAL_TEST_H
#define LIBRARIES_ROSSERIAL_TEST_H

#include <ros.h>
#include <std_msgs/Empty.h>
//#include <std_msgs/UInt16.h>

class Test {
public:
    explicit Test(ros::NodeHandle& nh);
    void callback(const std_msgs::Empty& msg);
    void init();
    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber<std_msgs::Empty, Test> sub_;
    float rate_ = 1.;
    bool active = true;
    ros::Time last_t;
};

// DEFINITIONS

/**
 *
 * @param nh
 */
Test::Test(ros::NodeHandle &nh)
    : nh_(nh)
    , sub_("test_topic", &Test::callback, this)
{
    pinMode(LED_BUILTIN, OUTPUT);
}

/**
 *
 * @param msg
 */
void Test::callback(const std_msgs::Empty &msg) {
    nh_.loginfo("Received a message!");
    active = !active;
}

void Test::init() {
    //nh_.initNode();
    nh_.subscribe(sub_);

    while (!nh_.connected())
    {
        nh_.spinOnce();
    }

    last_t = nh_.now();
}

void Test::run() {
    ros::Duration dt = nh_.now() - last_t;
    if ((dt.toSec() >= 1 / rate_) && active)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        last_t = nh_.now();
    }
}

#endif //LIBRARIES_ROSSERIAL_TEST_H
