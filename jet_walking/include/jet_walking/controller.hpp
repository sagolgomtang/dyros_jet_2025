#ifndef JET_WALKING_CONTROLLER_HPP
#define JET_WALKING_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "motion.hpp"
#include "sensor.hpp"
#include "input.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/string.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller();
    ~Controller();
    bool motor_flag;

private:
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<Motion> motion_node;
    //std::shared_ptr<Sensor> sensor_node;
    std::shared_ptr<Input> input_node;
    std::thread executor_thread;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_subscriber;  // 🔹 키 입력을 받을 구독자 추가
    
    void scanMotors();  // 🔹 Dynamixel 초기화
    void setInitialConfiguration();
    void inputCallback(const std_msgs::msg::String::SharedPtr msg);  // 🔹 키 입력 콜백 함수
    void disableMotor();
    void shutDown();
};

#endif // JET_WALKING_CONTROLLER_HPP