#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "../include/interface/Joystick_Interface.h"


class TestJoystickInterface : public ::testing::Test {
protected:
    void SetUp() override {
        ros::NodeHandle nh;
        joystickInterface = new JoystickInterface(nh); //will also subscribe to /joy topic
    }

    void TearDown() override {
        delete joystickInterface;
    }

    JoystickInterface* joystickInterface;
};

// Test case to check button state 
TEST_F(TestJoystickInterface, ButtonState) {
    sensor_msgs::Joy joy_msg;
    joy_msg.buttons = {1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1};
    
    ros::NodeHandle nh;
    auto pub = nh.advertise<sensor_msgs::Joy>("/joy", 1);
    ros::Rate loop_rate(10);

    while (pub.getNumSubscribers() < 1) {
        loop_rate.sleep();
    }
    pub.publish(joy_msg);
    
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_TRUE(joystickInterface->getButtonState("X"));
    EXPECT_FALSE(joystickInterface->getButtonState("A"));
    EXPECT_TRUE(joystickInterface->getButtonState("B"));
    EXPECT_FALSE(joystickInterface->getButtonState("Y"));
    EXPECT_FALSE(joystickInterface->getButtonState("LeftBumper"));
    EXPECT_TRUE(joystickInterface->getButtonState("RightBumper"));
    EXPECT_FALSE(joystickInterface->getButtonState("LeftTrigger"));
    EXPECT_FALSE(joystickInterface->getButtonState("RightTrigger"));
    EXPECT_TRUE(joystickInterface->getButtonState("Back"));
    EXPECT_FALSE(joystickInterface->getButtonState("Start"));
    EXPECT_FALSE(joystickInterface->getButtonState("LeftStickClick"));
    EXPECT_TRUE(joystickInterface->getButtonState("RightStickClick"));
    
}

// Run all the tests that were declared with TEST_F()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_joystick_interface");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
 }
